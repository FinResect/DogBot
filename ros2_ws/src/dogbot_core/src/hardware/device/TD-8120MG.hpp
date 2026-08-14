#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>

namespace dogbot_core::hardware::device {

// TD-8120MG 270° 舵机：通过树莓派 5 硬件 PWM（sysfs /sys/class/pwm）驱动。
// 仅两个状态 IDLE / ENABLE，由 <prefix>/enable 话题（std_msgs::Bool）切换：
// true → ENABLE，false → IDLE。每个状态对应一个角度（角度制），在 update()
// 中换算为 PWM 占空比写入 sysfs。
class TD_8120MG {
public:
    TD_8120MG()                            = default;
    TD_8120MG(const TD_8120MG&)            = delete;
    TD_8120MG& operator=(const TD_8120MG&) = delete;

    // 初始化：创建 <prefix>/enable 订阅者（true=ENABLE，false=IDLE），并在
    // 指定 GPIO 上启用树莓派 5 硬件 PWM。gpio 仅支持 PWM 功能脚
    // （12/13/18/19），非法输入抛 std::invalid_argument。
    void init(rclcpp::Node* node, const std::string& prefix, int gpio) {
        node_    = node;
        channel_ = gpio_to_channel(gpio);
        base_    = std::string(kPwmChipPath) + "/pwm" + std::to_string(channel_);

        if (!std::filesystem::exists(base_)) {
            write_sysfs(std::string(kPwmChipPath) + "/export", std::to_string(channel_));
        }
        write_sysfs(base_ + "/period", std::to_string(static_cast<int64_t>(kPeriodUs * 1000.0)));
        write_sysfs(base_ + "/duty_cycle", "0");
        write_sysfs(base_ + "/enable", "1");

        using std::placeholders::_1;
        enable_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
            prefix + "/enable", 10, std::bind(&TD_8120MG::enable_callback, this, _1));
    }

    // 设定两个状态的角度（角度制），内部换算为 PWM 值并 clamp 到 [0°, 270°]
    void set_angle(double idle_angle_deg, double enable_angle_deg) {
        idle_pwm_us_   = angle_to_pwm(idle_angle_deg);
        enable_pwm_us_ = angle_to_pwm(enable_angle_deg);
    }

    // 将当前状态对应的 PWM 占空比写入硬件
    void update() {
        if (base_.empty()) {
            return;
        }
        const double duty_us = state_ == State::ENABLE ? enable_pwm_us_ : idle_pwm_us_;
        if (duty_us == written_duty_us_) {
            return;
        }
        write_duty(duty_us);
        written_duty_us_ = duty_us;
    }

    // 析构时关闭 PWM 输出，避免舵机在无控制下保持驱动
    ~TD_8120MG() {
        if (!base_.empty()) {
            try {
                write_sysfs(base_ + "/enable", "0");
            } catch (const std::exception&) {}
        }
    }

private:
    // 舵机固有参数（500–2500µs 对应 0°–270°，50Hz 周期）
    static constexpr double kPwmMinUs    = 500.0;
    static constexpr double kPwmMaxUs    = 2500.0;
    static constexpr double kAngleMin    = 0.0;
    static constexpr double kAngleMax    = 270.0;
    static constexpr double kPeriodUs    = 20000.0; // 50Hz
    static constexpr char kPwmChipPath[] = "/sys/class/pwm/pwmchip0";

    enum class State { IDLE, ENABLE };

    // 树莓派 5（RP1）PWM 功能脚与 pwmchip0 通道的对应关系
    static int gpio_to_channel(int gpio) {
        switch (gpio) {
        case 12: return 0;
        case 13: return 1;
        case 18: return 2;
        case 19: return 3;
        default:
            throw std::invalid_argument(
                "GPIO " + std::to_string(gpio) + " 不支持硬件 PWM(仅 12/13/18/19)");
        }
    }

    // 角度制 → PWM 脉宽（µs），线性映射 [0°,270°] → [500,2500]µs
    static double angle_to_pwm(double angle_deg) {
        const double clamped = std::clamp(angle_deg, kAngleMin, kAngleMax);
        return kPwmMinUs + clamped / kAngleMax * (kPwmMaxUs - kPwmMinUs);
    }

    static void write_sysfs(const std::string& path, const std::string& value) {
        const int fd = ::open(path.c_str(), O_WRONLY);
        if (fd < 0) {
            throw std::runtime_error("open " + path + " failed: " + std::strerror(errno));
        }
        const ssize_t n = ::write(fd, value.c_str(), value.size());
        ::close(fd);
        if (n != static_cast<ssize_t>(value.size())) {
            throw std::runtime_error("write " + path + " failed: " + std::strerror(errno));
        }
    }

    // 写入占空比（µs）。bcm2835-pwm（Pi 4）在使能状态下直接写 duty_cycle 会
    // 返回 EBUSY，回退为先写 period 再写 duty_cycle 解锁；RP1（Pi 5）无此
    // 限制，首次写入即成功
    void write_duty(double duty_us) const {
        const std::string duty_ns = std::to_string(static_cast<int64_t>(duty_us * 1000.0));
        try {
            write_sysfs(base_ + "/duty_cycle", duty_ns);
        } catch (const std::exception&) {
            write_sysfs(
                base_ + "/period", std::to_string(static_cast<int64_t>(kPeriodUs * 1000.0)));
            write_sysfs(base_ + "/duty_cycle", duty_ns);
        }
    }

    void enable_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        state_ = msg->data ? State::ENABLE : State::IDLE;
    }

    rclcpp::Node* node_ = nullptr;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;

    State state_ = State::IDLE;
    int channel_ = -1;
    std::string base_;
    double idle_pwm_us_     = kPwmMinUs;
    double enable_pwm_us_   = kPwmMinUs;
    double written_duty_us_ = -1.0;
};
} // namespace dogbot_core::hardware::device
