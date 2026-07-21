#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <string>

namespace dogbot_core::hardware::device {

class ZX30S {
public:
    ZX30S() = default;

    void init(
        rclcpp::Node* node, const std::string& prefix, int servo_id, double angle_min,
        double angle_max, double angle_max_deg = 270.0) {
        node_          = node;
        servo_id_      = servo_id;
        angle_min_     = angle_min;
        angle_max_     = angle_max;
        angle_max_deg_ = angle_max_deg;

        using std::placeholders::_1;
        angle_pub_  = node_->create_publisher<std_msgs::msg::Float64>(prefix + "/angle", 10);
        torque_pub_ = node_->create_publisher<std_msgs::msg::Float64>(prefix + "/torque", 10);
        control_angle_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
            prefix + "/control_angle", 10, std::bind(&ZX30S::controlAngleCallback, this, _1));
    }

    double clampAngle(double angle) const { return std::clamp(angle, angle_min_, angle_max_); }

    uint16_t angleToPWM(double angle) const {
        double clamped = clampAngle(angle);
        return static_cast<uint16_t>(clamped / angle_max_deg_ * 2000.0 + 500.0);
    }

    std::string generateCommand(uint16_t pwm, uint16_t time_ms = 0) const {
        return makeCommand(formatPWM(pwm) + "T" + formatTime(time_ms));
    }

    uint16_t getTargetPWM() const { return target_pwm_; }

    int getServoId() const { return servo_id_; }

    double pwmToAngle(uint16_t pwm) const {
        return static_cast<double>(pwm - 500) / 2000.0 * angle_max_deg_;
    }

    void publishAngle(double angle) {
        auto msg = std_msgs::msg::Float64();
        msg.data = angle;
        angle_pub_->publish(msg);
    }

    void publishTorque(double torque) {
        auto msg = std_msgs::msg::Float64();
        msg.data = torque;
        torque_pub_->publish(msg);
    }

    std::string generateReadVersion() const { return makeCommand("VER"); }

    std::string generateReadID() const { return makeCommand("ID"); }

    std::string generateSetID(int new_id) const {
        std::ostringstream body;
        body << "ID" << std::setw(3) << std::setfill('0') << new_id;
        return makeCommand(body.str());
    }

    std::string generateReleaseTorque() const { return makeCommand("ULK"); }

    std::string generateRestoreTorque() const { return makeCommand("ULR"); }

    std::string generateReadMode() const { return makeCommand("MOD"); }

    std::string generateSetMode(int mode) const {
        return makeCommand("MOD" + std::to_string(mode));
    }

    std::string generateReadPosition() const { return makeCommand("RAD"); }

    std::string generatePause() const { return makeCommand("DPT"); }

    std::string generateContinue() const { return makeCommand("DCT"); }

    std::string generateStop() const { return makeCommand("DST"); }

    std::string generateSetBaudrate(int baud) const {
        std::string code;
        switch (baud) {
        case 9600: code = "1"; break;
        case 19200: code = "2"; break;
        case 38400: code = "3"; break;
        case 57600: code = "4"; break;
        case 115200: code = "5"; break;
        case 128000: code = "6"; break;
        case 256000: code = "7"; break;
        case 1000000: code = "8"; break;
        default: throw std::invalid_argument("Unsupported baud rate: " + std::to_string(baud));
        }
        return makeCommand("BD" + code);
    }

    std::string generateCalibrate() const { return makeCommand("SCK"); }

    std::string generateSetStartPosition() const { return makeCommand("CSD"); }

    std::string generateSetStartupMode(int mode) const {
        return makeCommand("CSM" + std::to_string(mode));
    }

    std::string generateReadStartupMode() const { return makeCommand("CSM"); }

    std::string generateSetMinPosition() const { return makeCommand("SMI"); }

    std::string generateSetMaxPosition() const { return makeCommand("SMX"); }

    std::string generateFactoryResetPartial() const { return makeCommand("CLE0"); }

    std::string generateFactoryResetFull() const { return makeCommand("CLE"); }

    std::string generateReadTempVoltage() const { return makeCommand("RTV"); }

private:
    void controlAngleCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        double clamped = clampAngle(msg->data);
        target_pwm_    = angleToPWM(clamped);
        publishAngle(clamped);
    }

    std::string formatId() const {
        std::ostringstream oss;
        oss << std::setw(3) << std::setfill('0') << servo_id_;
        return oss.str();
    }

    std::string formatPWM(uint16_t pwm) const {
        std::ostringstream oss;
        oss << std::setw(4) << std::setfill('0') << pwm;
        return oss.str();
    }

    std::string formatTime(uint16_t time_ms) const {
        std::ostringstream oss;
        oss << std::setw(4) << std::setfill('0') << time_ms;
        return oss.str();
    }

    std::string makeCommand(const std::string& body) const {
        return "#" + formatId() + "P" + body + "!";
    }

    rclcpp::Node* node_ = nullptr;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr torque_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr control_angle_sub_;

    int servo_id_         = 0;
    double angle_min_     = 0.0;
    double angle_max_     = 270.0;
    double angle_max_deg_ = 270.0;
    uint16_t target_pwm_  = 1500;
};
} // namespace dogbot_core::hardware::device