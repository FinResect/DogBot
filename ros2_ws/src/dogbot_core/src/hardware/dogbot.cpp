#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "device/IMU660RB.hpp"
#include "device/TD-8120MG.hpp"
#include "device/ZX30S.hpp"
#include "device/gamepad.hpp"
#include "serial/serial.hpp"

namespace dogbot_core::hardware {
class DogBot : public rclcpp::Node {
public:
    explicit DogBot(const rclcpp::NodeOptions& options)
        : Node("dogbot", options)
        , serial_("/dev/ttyAMA0", 1000000) {

        gamepad_.init(this, "/remote", "/dev/input/js0");

        imu_.init(this, "/imu");
        imu_.set_coordinate_mapping(
            [](double x, double y, double z) { return std::make_tuple(-y, x, +z); });

        thrower_left_.init(this, "/thrower/left", 18);
        thrower_right_.init(this, "/thrower/right", 12);
        thrower_left_.set_angle(15, 110);
        thrower_right_.set_angle(155, 80);

        knee_[0].init(this, "left_front_knee", 4, 0.0, 270.0);
        knee_[0].setPWMOffset(0);

        knee_[1].init(this, "left_back_knee", 6, 0.0, 270.0);
        knee_[1].setPWMOffset(0);

        knee_[2].init(this, "right_back_knee", 0, 0.0, 270.0);
        knee_[2].setPWMOffset(0);

        knee_[3].init(this, "right_front_knee", 2, 0.0, 270.0);
        knee_[3].setPWMOffset(0);

        for (uint8_t i = 0; i < 4; i++) {
            knee_[i].setPWMLimits(1100, 1600);
        }

        hip_[0].init(this, "left_front_hip", 5, 0.0, 270.0);
        hip_[0].setPWMOffset(-25);

        hip_[1].init(this, "left_back_hip", 7, 0.0, 270.0);
        hip_[1].setPWMOffset(0); // 舵盘不同导致需要加偏置

        hip_[2].init(this, "right_back_hip", 1, 0.0, 270.0);
        hip_[2].setPWMOffset(0);

        hip_[3].init(this, "right_front_hip", 3, 0.0, 270.0);
        hip_[3].setPWMOffset(-125);
        for (uint8_t i = 0; i < 4; i++) {
            hip_[i].setPWMLimits(830, 2200);
        }

        for (auto& s : knee_) {
            serial_.writeString(s.generateRestoreTorque());
            RCLCPP_INFO(this->get_logger(), "Servo ID=%d initialized", s.getServoId());
        }
        for (auto& s : hip_) {
            serial_.writeString(s.generateRestoreTorque());
            RCLCPP_INFO(this->get_logger(), "Servo ID=%d initialized", s.getServoId());
        }

        using namespace std::chrono_literals;
        timer_      = this->create_wall_timer(2ms, std::bind(&DogBot::update, this));
        imu_thread_ = std::thread(&DogBot::imu_loop, this);

        RCLCPP_INFO(
            this->get_logger(), "DogBot ready, 8 servos on %s", serial_.getDevice().c_str());
    }

    ~DogBot() override {
        imu_stop_.store(true);
        if (imu_thread_.joinable()) {
            imu_thread_.join();
        }
    }

private:
    void update() {
        command_update();
        thrower_update();
        gamepad_.update();
    }

    void command_update() {
        auto clock = this->get_clock();
        for (auto& s : knee_) {
            try {
                serial_.writeString(s.generateAngleCommand());
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *clock, 1000, "Write knee ID=%d failed: %s", s.getServoId(),
                    e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        for (auto& s : hip_) {
            try {
                serial_.writeString(s.generateAngleCommand());
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *clock, 1000, "Write hip ID=%d failed: %s", s.getServoId(),
                    e.what());
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
    }

    void thrower_update() {
        thrower_left_.update();
        thrower_right_.update();
    }

    // IMU 采集独立线程：200Hz 采样，与 500Hz 控制定时器解耦，
    // I2C 读取/重试阻塞只影响本线程，不影响舵机控制频率
    void imu_loop() {
        using namespace std::chrono_literals;
        while (!imu_stop_.load()) {
            const auto start = std::chrono::steady_clock::now();
            try {
                imu_.update();
                // RCLCPP_INFO(
                //     get_logger(), "pitch:%lf \t yaw:%lf \t roll:%lf \t", imu_.get_pitch(),
                //     imu_.get_yaw(), imu_.get_roll());
            } catch (const std::exception& e) {
                RCLCPP_ERROR_THROTTLE(
                    this->get_logger(), *this->get_clock(), 1000, "IMU update failed: %s",
                    e.what());
            }
            // 固定周期扣除本次耗时，I2C 变慢时避免循环漂移堆积
            const auto elapsed = std::chrono::steady_clock::now() - start;
            const auto sleep   = std::chrono::milliseconds(kImuPeriodMs) - elapsed;
            if (sleep > 0ms) {
                std::this_thread::sleep_for(sleep);
            }
        }
    }

    SerialPort serial_;
    device::Gamepad gamepad_;
    device::IMU660RB imu_;
    device::TD_8120MG thrower_left_;
    device::TD_8120MG thrower_right_;
    dogbot_core::hardware::device::ZX30S knee_[4];
    dogbot_core::hardware::device::ZX30S hip_[4];
    rclcpp::TimerBase::SharedPtr timer_;
    std::thread imu_thread_;
    std::atomic<bool> imu_stop_{false};
    static constexpr int kImuPeriodMs = 5; // 200Hz 采样（陀螺 ODR 208Hz）
};
} // namespace dogbot_core::hardware

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::hardware::DogBot)
