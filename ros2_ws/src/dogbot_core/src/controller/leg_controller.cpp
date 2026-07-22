#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>

#include <memory>
#include <string>

#include "controller/leg_solver.hpp"

namespace dogbot_core::controller {

class LegController : public rclcpp::Node {
public:
    explicit LegController(const rclcpp::NodeOptions& options)
        : Node("leg_controller", options) {

        double thigh_len  = this->declare_parameter("thigh_length", 0.053);
        double calf_len   = this->declare_parameter("calf_length", 0.053);
        double hip_offs   = this->declare_parameter("hip_offset", 0.752);
        double L3         = this->declare_parameter("L3", 0.04);
        double L4         = this->declare_parameter("L4", 0.03);
        double L5         = this->declare_parameter("L5", 0.045);
        double r_arm      = this->declare_parameter("servo_arm", 0.015);
        double delta      = this->declare_parameter("rocker_offset", 1.57);
        bool fork_branch  = this->declare_parameter("fork_branch", true);

        solver_ = std::make_unique<LegSolver>(
            thigh_len, calf_len, hip_offs, L3, L4, L5, r_arm, delta, fork_branch);

        pub_hip_  = this->create_publisher<std_msgs::msg::Float64>(
            "left_front_hip/control_angle", 10);
        pub_knee_ = this->create_publisher<std_msgs::msg::Float64>(
            "left_front_knee/control_angle", 10);

        sub_foot_ = this->create_subscription<geometry_msgs::msg::Point>(
            "foot_target", 10,
            std::bind(&LegController::footCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "LegController ready (L1=%.4f L2=%.4f L3=%.4f L4=%.4f L5=%.4f r=%.4f δ=%.4f fork=%d)",
            thigh_len, calf_len, L3, L4, L5, r_arm, delta, fork_branch);
    }

private:
    void footCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        Eigen::Vector3d foot(msg->x, msg->y, msg->z);
        auto angles = solver_->solve(foot);

        auto hip_msg  = std_msgs::msg::Float64();
        auto knee_msg = std_msgs::msg::Float64();
        hip_msg.data  = angles.hip * 180.0 / M_PI;
        knee_msg.data = angles.knee * 180.0 / M_PI;

        pub_hip_->publish(hip_msg);
        pub_knee_->publish(knee_msg);
    }

    std::unique_ptr<LegSolver> solver_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_hip_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_knee_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_foot_;
};

} // namespace dogbot_core::controller

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::controller::LegController)
