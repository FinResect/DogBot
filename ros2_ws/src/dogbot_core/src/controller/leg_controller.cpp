#include "controller/gait.hpp"
#include "controller/leg_solver.hpp"

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <numbers>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
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

        theta_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/vision/following/theta", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { theta_ = msg->data; });

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
        static bool flag = true;
        if (flag) {
            gait_.setGaitType(GaitType::TrotSpinMix);
            flag = !flag;
        }
        solver_update(feet_update());
    }

    std::array<Eigen::Vector3d, 4> feet_update() {

        gait_.setGaitParam(0.2 * std::cos(theta_), 2.0 * std::sin(theta_));
        return gait_.update(dt_);
    }

    void solver_update(std::array<Eigen::Vector3d, 4> feet) {
        for (int i = 0; i < 4; ++i) {
            const auto angles = solver_->solve(feet[i]);

            std_msgs::msg::Float64 hip_msg;
            std_msgs::msg::Float64 knee_msg;
            hip_msg.data  = angles.hip * 180.0 / std::numbers::pi;
            knee_msg.data = angles.knee * 180.0 / std::numbers::pi;

            pub_hip_[i]->publish(hip_msg);
            pub_knee_[i]->publish(knee_msg);
        }
    }

    Gait gait_;
    std::unique_ptr<LegSolver> solver_;

    double theta_               = 0.0;
    static constexpr double dt_ = 0.002;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr theta_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_hip_[4];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_knee_[4];
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

} // namespace dogbot_core::controller

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::controller::LegController)
