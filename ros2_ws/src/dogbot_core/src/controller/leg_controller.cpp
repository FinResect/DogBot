#include "controller/gait.hpp"
#include "controller/leg_solver.hpp"
#include "controller_mode.hpp"

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstdint>
#include <dogbot_msg/msg/gamepad_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <numbers>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/detail/bool__struct.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>

namespace dogbot_core::controller {

class LegController : public rclcpp::Node {
public:
    explicit LegController(const rclcpp::NodeOptions& options)
        : Node("leg_controller", options) {

        double thigh_len = this->declare_parameter("thigh_length", 0.053);
        double calf_len  = this->declare_parameter("calf_length", 0.053);
        double hip_offs  = this->declare_parameter("hip_offset", 0.752);
        double L3        = this->declare_parameter("L3", 0.04);
        double L4        = this->declare_parameter("L4", 0.03);
        double L5        = this->declare_parameter("L5", 0.045);
        double r_arm     = this->declare_parameter("servo_arm", 0.015);
        double delta     = this->declare_parameter("rocker_offset", 1.57);
        bool fork_branch = this->declare_parameter("fork_branch", true);

        solver_ = std::make_unique<LegSolver>(
            thigh_len, calf_len, hip_offs, L3, L4, L5, r_arm, delta, fork_branch);

        gamepad_state_sub_ = create_subscription<dogbot_msg::msg::GamepadState>(
            "/remote/gamepad", 10,
            [this](const dogbot_msg::msg::GamepadState::SharedPtr msg) { gamepad_state_ = *msg; });

        theta_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/vision/following/theta", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { theta_ = msg->data; });

        pub_thrower_left_  = create_publisher<std_msgs::msg::Bool>("/thrower/left/enable", 10);
        pub_thrower_right_ = create_publisher<std_msgs::msg::Bool>("/thrower/right/enable", 10);

        static constexpr const char* kLegNames[4] = {
            "left_front", "left_back", "right_back", "right_front"};
        for (int i = 0; i < 4; ++i) {
            pub_hip_[i] = this->create_publisher<std_msgs::msg::Float64>(
                std::string(kLegNames[i]) + "_hip/control_angle", 10);
            pub_knee_[i] = this->create_publisher<std_msgs::msg::Float64>(
                std::string(kLegNames[i]) + "_knee/control_angle", 10);
        }

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(2ms, std::bind(&LegController::update, this));
    }

private:
    void update() {

        std_msgs::msg::Bool left_msg;
        std_msgs::msg::Bool right_msg;

        left_msg.data  = false;
        right_msg.data = false;

        pub_thrower_left_->publish(left_msg);
        pub_thrower_right_->publish(right_msg);

        using namespace dogbot_msg::msg;

        auto button_x       = gamepad_state_.buttons.x;
        auto button_y       = gamepad_state_.buttons.y;
        auto button_a       = gamepad_state_.buttons.a;
        auto button_b       = gamepad_state_.buttons.b;
        auto button_up      = gamepad_state_.dpad.up;
        auto button_start   = gamepad_state_.buttons.start;
        auto button_mode    = gamepad_state_.buttons.mode;
        auto joystick_left  = gamepad_state_.sticks.joystick_left;
        auto joystick_right = gamepad_state_.sticks.joystick_right;

        if (gamepad_state_.status == GamepadState::UNKNOWN
            || gamepad_state_.status == GamepadState::DISCONNECTED
            || (button_mode && !last_button_mode)) {
            reset_all_controller();
            controller_mode_ = dogbot_msg::ControllerMode::None;
            return;
        }

        if (button_start && !last_button_start) {
            gait_.set_gait_type(GaitType::Stand);
            controller_mode_ = dogbot_msg::ControllerMode::Auto;
        } else if (button_x && !last_button_x) {
            gait_.set_gait_type(GaitType::Trot);
            controller_mode_ = dogbot_msg::ControllerMode::Auto;
        } else if (button_y && !last_button_y) {
            gait_.set_gait_type(GaitType::TrotSpinMix);
            controller_mode_ = dogbot_msg::ControllerMode::Manual;
        } else if (button_a && !last_button_a) {
            gait_.set_gait_type(GaitType::Walk);
            controller_mode_ = dogbot_msg::ControllerMode::Manual;
        } else if (button_b && !last_button_b) {
            gait_.set_gait_type(GaitType::Spin);
            controller_mode_ = dogbot_msg::ControllerMode::Auto;
        } else if (button_up && !last_button_up) {
            gait_.set_gait_type(GaitType::Climb);
            controller_mode_ = dogbot_msg::ControllerMode::Auto;
        }

        switch (controller_mode_) {
        case dogbot_msg::ControllerMode::Auto: solver_update(feet_update(0.2, 2.0)); break;
        case dogbot_msg::ControllerMode::Manual:
            solver_update(feet_update(0.2 * joystick_left.y, -5.0 * joystick_right.x));
            break;
        default: reset_all_controller(); break;
        }

        last_button_a     = button_a;
        last_button_b     = button_b;
        last_button_x     = button_x;
        last_button_y     = button_y;
        last_button_up    = button_up;
        last_button_start = button_start;
        last_button_mode  = button_mode;
    }

    void reset_all_controller() {
        gait_.reset();
        for (int i = 0; i < 4; ++i) {

            std_msgs::msg::Float64 hip_msg;
            std_msgs::msg::Float64 knee_msg;
            hip_msg.data  = NAN;
            knee_msg.data = NAN;

            pub_hip_[i]->publish(hip_msg);
            pub_knee_[i]->publish(knee_msg);
        }
    }

    std::array<Eigen::Vector3d, 4> feet_update(double vx, double omega) {

        gait_.set_gait_param(vx, omega);
        return gait_.update(dt_);
    }

    void solver_update(std::array<Eigen::Vector3d, 4> feet) {
        for (int i = 0; i < 4; ++i) {
            const auto angles = solver_->solve(feet[i]);

            std_msgs::msg::Float64 hip_msg;
            std_msgs::msg::Float64 knee_msg;
            hip_msg.data  = angles.hip * 180.0 / std::numbers::pi;
            knee_msg.data = angles.knee * 180.0 / std::numbers::pi;
            if (!i) {
                hip_msg.data += 5.0;
            }

            pub_hip_[i]->publish(hip_msg);
            pub_knee_[i]->publish(knee_msg);
        }
    }

    dogbot_msg::msg::GamepadState gamepad_state_;
    Gait gait_;
    std::unique_ptr<LegSolver> solver_;
    dogbot_msg::ControllerMode controller_mode_{dogbot_msg::ControllerMode::None};

    uint8_t last_button_x;
    uint8_t last_button_y;
    uint8_t last_button_a;
    uint8_t last_button_b;
    uint8_t last_button_up;
    uint8_t last_button_start;
    uint8_t last_button_mode;

    double theta_               = 0.0;
    static constexpr double dt_ = 0.002;

    rclcpp::Subscription<dogbot_msg::msg::GamepadState>::SharedPtr gamepad_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr theta_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_hip_[4];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_knee_[4];
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_thrower_left_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_thrower_right_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace dogbot_core::controller

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::controller::LegController)
