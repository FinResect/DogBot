#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "device/ZX30S.hpp"
#include "serial/serial.hpp"

class DogBot : public rclcpp::Node {
public:
    explicit DogBot(const rclcpp::NodeOptions& options)
        : Node("dogbot", options)
        , serial_("/dev/ttyAMA0", 921600) {

        knee_[0].init(this, "left_front_knee", 1, 0.0, 270.0);
        knee_[1].init(this, "left_back_knee", 3, 0.0, 270.0);
        knee_[2].init(this, "right_back_knee", 5, 0.0, 270.0);
        knee_[3].init(this, "right_front_knee", 7, 0.0, 270.0);

        hip_[0].init(this, "left_front_hip", 0, 0.0, 270.0);
        hip_[1].init(this, "left_back_hip", 2, 0.0, 270.0);
        hip_[2].init(this, "right_back_hip", 4, 0.0, 270.0);
        hip_[3].init(this, "right_front_hip", 6, 0.0, 270.0);

        for (auto& s : knee_) {
            serial_.writeString(s.generateRestoreTorque());
            RCLCPP_INFO(this->get_logger(), "Servo ID=%d initialized", s.getServoId());
        }
        for (auto& s : hip_) {
            serial_.writeString(s.generateRestoreTorque());
            RCLCPP_INFO(this->get_logger(), "Servo ID=%d initialized", s.getServoId());
        }

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(50ms, std::bind(&DogBot::update, this));

        RCLCPP_INFO(
            this->get_logger(), "DogBot ready, 8 servos on %s", serial_.getDevice().c_str());
    }

private:
    void update() {
        for (auto& s : knee_) {
            serial_.writeString(s.generateCommand(s.getTargetPWM(), 0));
        }
        for (auto& s : hip_) {
            serial_.writeString(s.generateCommand(s.getTargetPWM(), 0));
        }
    }

    SerialPort serial_;
    dogbot_core::hardware::device::ZX30S knee_[4];
    dogbot_core::hardware::device::ZX30S hip_[4];
    rclcpp::TimerBase::SharedPtr timer_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(DogBot)
