#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <memory>
#include <string>
#include <thread>

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

        knee_[0].init(this, "left_front_knee", 0, 0.0, 270.0);
        knee_[1].init(this, "left_back_knee", 2, 0.0, 270.0);
        knee_[2].init(this, "right_back_knee", 4, 0.0, 270.0);
        knee_[3].init(this, "right_front_knee", 6, 0.0, 270.0);
        for (uint8_t i = 0; i < 4; i++) {
            knee_[i].setPWMLimits(1100, 1600);
        }

        hip_[0].init(this, "left_front_hip", 1, 0.0, 270.0);
        hip_[1].init(this, "left_back_hip", 3, 0.0, 270.0);
        hip_[2].init(this, "right_back_hip", 5, 0.0, 270.0);
        hip_[3].init(this, "right_front_hip", 7, 0.0, 270.0);
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
        timer_ = this->create_wall_timer(2ms, std::bind(&DogBot::update, this));

        RCLCPP_INFO(
            this->get_logger(), "DogBot ready, 8 servos on %s", serial_.getDevice().c_str());
    }

private:
    void update() {
        command_update();
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

    SerialPort serial_;
    device::Gamepad gamepad_;
    dogbot_core::hardware::device::ZX30S knee_[4];
    dogbot_core::hardware::device::ZX30S hip_[4];
    rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace dogbot_core::hardware

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::hardware::DogBot)
