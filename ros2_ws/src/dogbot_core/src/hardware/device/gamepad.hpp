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
// 状态经 dogbot_msg/GamepadState 话题对外发布（连接状态下每次 update() 发布全量状态）。
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
    // - 读取所有待处理事件；连接状态下每次 update() 发布一次 CONNECTED 全量状态
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

        struct js_event ev;
        while (true) {
            const ssize_t n = ::read(fd_, &ev, sizeof(ev));
            if (n == static_cast<ssize_t>(sizeof(ev))) {
                handleEvent(ev);
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

        status_ = Status::Connected;
        publish(Status::Connected);
    }

private:
    // 经典 joystick API 轴序（jstest 实测 USB WirelessGamepad，6 轴）：
    //   axes[0]=LX 左摇杆 x   axes[1]=LY 左摇杆 y
    //   axes[2]=RX(Z) 右摇杆 x   axes[3]=RY(Rz) 右摇杆 y
    //   axes[4]=Hat0X 十字键 x   axes[5]=Hat0Y 十字键 y
    // 归一化：轴 value/32767 → [-1,1]；本机无模拟扳机轴（L2/R2 为按键）
    static constexpr int kAxisLeftX    = 0;
    static constexpr int kAxisLeftY    = 1;
    static constexpr int kAxisRightX   = 2;
    static constexpr int kAxisRightY   = 3;
    static constexpr int kAxisHatX     = 4;
    static constexpr int kAxisHatY     = 5;

    // 经典 joystick API 按键编号（实体实测 USB WirelessGamepad，13 键，PS2 位序）：
    //   A=2  B=1  X=3  Y=0    面键（js0=Y、js1=B、js2=A、js3=X）
    //   L1=4  R1=5  L2=6  R2=7  肩键
    //   SELECT=8  START=9  L3=10  R3=11  MODE=12
    // 注意：jstest 显示的 BtnA/BtnC/BtnZ 等名称是 USB 描述符误标，实体无 C/Z 键，
    // 实体 L3/R3 摇杆按下在 js 10/11；L2/R2 为按键（无模拟扳机轴），扳机取键值
    static constexpr int kBtnA      = 2;
    static constexpr int kBtnB      = 1;
    static constexpr int kBtnX      = 3;
    static constexpr int kBtnY      = 0;
    static constexpr int kBtnL1     = 4;
    static constexpr int kBtnR1     = 5;
    static constexpr int kBtnL2     = 6;
    static constexpr int kBtnR2     = 7;
    static constexpr int kBtnSelect = 8;
    static constexpr int kBtnStart  = 9;
    static constexpr int kBtnL3     = 10;
    static constexpr int kBtnR3     = 11;
    static constexpr int kBtnMode   = 12;

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

    // 按键开关态：1=按下，0=未按下
    uint8_t button(int index) const {
        return index < static_cast<int>(buttons_.size()) && buttons_[index] != 0 ? 1 : 0;
    }

    // 十字键：轴值取整为 -1/0/1 后转为方向开关；Hat0Y 上推为负（实测）
    uint8_t hatLeft() const { return axis(kAxisHatX) < -0.5f ? 1 : 0; }
    uint8_t hatRight() const { return axis(kAxisHatX) > 0.5f ? 1 : 0; }
    uint8_t hatUp() const { return axis(kAxisHatY) < -0.5f ? 1 : 0; }
    uint8_t hatDown() const { return axis(kAxisHatY) > 0.5f ? 1 : 0; }

    void publish(Status status) {
        if (!state_pub_) {
            return;
        }
        auto msg = dogbot_msg::msg::GamepadState();
        msg.status = static_cast<uint8_t>(status);

        msg.buttons.a      = button(kBtnA);
        msg.buttons.b      = button(kBtnB);
        msg.buttons.c      = 0;
        msg.buttons.x      = button(kBtnX);
        msg.buttons.y      = button(kBtnY);
        msg.buttons.z      = 0;
        msg.buttons.l3     = button(kBtnL3);
        msg.buttons.r3     = button(kBtnR3);
        msg.buttons.select = button(kBtnSelect);
        msg.buttons.start  = button(kBtnStart);
        msg.buttons.mode   = button(kBtnMode);

        // 摇杆 y 轴符号与消息语义相反（实测推上为负），发布时取反
        msg.sticks.joystick_left.x  = axis(kAxisLeftX);
        msg.sticks.joystick_left.y  = -axis(kAxisLeftY);
        msg.sticks.joystick_right.x = axis(kAxisRightX);
        msg.sticks.joystick_right.y = -axis(kAxisRightY);

        msg.shoulders.l1            = button(kBtnL1);
        msg.shoulders.r1            = button(kBtnR1);
        msg.shoulders.l2            = button(kBtnL2);
        msg.shoulders.r2            = button(kBtnR2);
        msg.shoulders.trigger_left  = button(kBtnL2);
        msg.shoulders.trigger_right = button(kBtnR2);

        msg.dpad.up    = hatUp();
        msg.dpad.down  = hatDown();
        msg.dpad.left  = hatLeft();
        msg.dpad.right = hatRight();

        state_pub_->publish(msg);
    }

    rclcpp::Node* node_ = nullptr;
    std::string device_;
    rclcpp::Publisher<dogbot_msg::msg::GamepadState>::SharedPtr state_pub_;

    int fd_                 = -1;
    Status status_          = Status::Unknown;
    bool unknown_published_ = false;
    std::vector<float> axes_;
    std::vector<int32_t> buttons_;
};
} // namespace dogbot_core::hardware::device
