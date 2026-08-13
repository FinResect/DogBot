#pragma once

#include <rclcpp/rclcpp.hpp>
#include <dogbot_msg/msg/gamepad_state.hpp>

#include <fcntl.h>
#include <linux/joystick.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace dogbot_core::hardware::device {

// 手柄（USB 游戏手柄）设备类。
// 通过经典 Linux joystick API（/dev/input/jsX）读取，对外仅暴露 init() 与 update()，
// 状态经 dogbot_msg/GamepadState 话题对外发布（仅状态变化时发布）。
class Gamepad {
public:
    // 连接状态（与 GamepadState.status 字段枚举一致）
    enum class Status : uint8_t { Unknown = 0, Connected = 1, Disconnected = 2 };

    Gamepad() = default;
    Gamepad(const Gamepad&)            = delete;
    Gamepad& operator=(const Gamepad&) = delete;

    ~Gamepad() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
    }

    // 初始化：传入节点与前缀，创建 prefix + "/gamepad" 话题发布器（GamepadState）；
    // device 为手柄设备路径，默认 /dev/input/js0
    void init(rclcpp::Node* node, const std::string& prefix,
              const std::string& device = "/dev/input/js0") {
        node_      = node;
        device_    = device;
        state_pub_ =
            node_->create_publisher<dogbot_msg::msg::GamepadState>(prefix + "/gamepad", 10);
    }

    // 从 USB 更新手柄状态：
    // - 读取所有待处理事件，状态量变化时发布 CONNECTED 全量消息
    // - 设备缺失时发布一次 UNKNOWN；断开/重连时发布对应的状态迁移消息
    // - 设备热插拔：打开失败静默，下次调用自动重试
    void update() {
        if (fd_ < 0) {
            if (tryOpen()) {
                status_ = Status::Connected;
            } else {
                if (status_ == Status::Unknown && !unknown_published_) {
                    unknown_published_ = true;
                    publish(Status::Unknown);
                }
                return;
            }
        }

        bool changed = false;
        struct js_event ev;
        while (true) {
            const ssize_t n = ::read(fd_, &ev, sizeof(ev));
            if (n == static_cast<ssize_t>(sizeof(ev))) {
                changed |= handleEvent(ev);
                continue;
            }
            if (n < 0 && errno == EAGAIN) {
                break;
            }
            // read 失败（EIO/ENODEV）说明设备已拔出
            ::close(fd_);
            fd_ = -1;
            if (status_ != Status::Disconnected) {
                status_ = Status::Disconnected;
                publish(Status::Disconnected);
            }
            return;
        }

        if (changed || status_ != Status::Connected) {
            status_ = Status::Connected;
            publish(Status::Connected);
        }
    }

private:
    // 经典 joystick API 轴序（标准手柄布局假设）：
    //   axes[0]=LX 左摇杆 x   axes[1]=LY 左摇杆 y
    //   axes[2]=RX 右摇杆 x   axes[3]=RY 右摇杆 y
    //   axes[4]=L2 左扳机     axes[5]=R2 右扳机
    //   axes[6]=十字键 x      axes[7]=十字键 y
    // 归一化：轴 value/32767 → [-1,1]；扳机再映射到 [0,1]；十字键取整为 -1/0/1
    static constexpr int kAxisLeftX    = 0;
    static constexpr int kAxisLeftY    = 1;
    static constexpr int kAxisRightX   = 2;
    static constexpr int kAxisRightY   = 3;
    static constexpr int kAxisTriggerL = 4;
    static constexpr int kAxisTriggerR = 5;
    static constexpr int kAxisHatX     = 6;
    static constexpr int kAxisHatY     = 7;

    bool tryOpen() {
        fd_ = ::open(device_.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd_ < 0) {
            return false;
        }
        axes_.clear();
        buttons_.clear();
        return true;
    }

    // 处理单个 joystick 事件，返回状态量是否有变化
    bool handleEvent(const struct js_event& ev) {
        switch (ev.type & ~JS_EVENT_INIT) {
        case JS_EVENT_BUTTON:
            if (ev.number >= buttons_.size()) {
                buttons_.resize(ev.number + 1, 0);
            }
            if (buttons_[ev.number] == ev.value) {
                return false;
            }
            buttons_[ev.number] = ev.value;
            return true;
        case JS_EVENT_AXIS: {
            if (ev.number >= axes_.size()) {
                axes_.resize(ev.number + 1, 0.0f);
            }
            const float value = static_cast<float>(ev.value) / 32767.0f;
            if (axes_[ev.number] == value) {
                return false;
            }
            axes_[ev.number] = value;
            return true;
        }
        default:
            return false;
        }
    }

    float axis(int index) const {
        return index < static_cast<int>(axes_.size()) ? axes_[index] : 0.0f;
    }

    // 扳机：[-1,1] → [0,1]
    float trigger(int index) const { return (axis(index) + 1.0f) * 0.5f; }

    // 十字键：取整为 -1/0/1
    int8_t hat(int index) const {
        const float v = axis(index);
        return static_cast<int8_t>(v > 0.5f ? 1 : (v < -0.5f ? -1 : 0));
    }

    void publish(Status status) {
        if (!state_pub_) {
            return;
        }
        auto msg = dogbot_msg::msg::GamepadState();
        msg.status          = static_cast<uint8_t>(status);
        msg.joystick_left.x = axis(kAxisLeftX);
        msg.joystick_left.y = axis(kAxisLeftY);
        msg.joystick_right.x = axis(kAxisRightX);
        msg.joystick_right.y = axis(kAxisRightY);
        msg.trigger_left   = trigger(kAxisTriggerL);
        msg.trigger_right  = trigger(kAxisTriggerR);
        msg.hat_x          = hat(kAxisHatX);
        msg.hat_y          = hat(kAxisHatY);
        for (const int32_t b : buttons_) {
            msg.buttons.push_back(b);
        }
        state_pub_->publish(msg);
    }

    rclcpp::Node* node_ = nullptr;
    std::string device_;
    rclcpp::Publisher<dogbot_msg::msg::GamepadState>::SharedPtr state_pub_;

    int fd_                     = -1;
    Status status_              = Status::Unknown;
    bool unknown_published_     = false;
    std::vector<float> axes_;
    std::vector<int32_t> buttons_;
};
} // namespace dogbot_core::hardware::device
