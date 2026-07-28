#include "controller/leg_solver.hpp"

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
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

        stance_y_    = this->declare_parameter("stance_y", 0.0);
        stance_z_    = this->declare_parameter("stance_z", 0.13);
        step_length_ = this->declare_parameter("step_length", 0.04);
        step_height_ = this->declare_parameter("step_height", 0.03);
        gait_period_ = this->declare_parameter("gait_period", 0.6);

        solver_ = std::make_unique<LegSolver>(
            thigh_len, calf_len, hip_offs, L3, L4, L5, r_arm, delta, fork_branch);

        static constexpr const char* kLegNames[4] = {
            "left_front", "left_back", "right_back", "right_front"};
        for (int i = 0; i < 4; ++i) {
            pub_hip_[i] = this->create_publisher<std_msgs::msg::Float64>(
                std::string(kLegNames[i]) + "_hip/control_angle", 10);
            pub_knee_[i] = this->create_publisher<std_msgs::msg::Float64>(
                std::string(kLegNames[i]) + "_knee/control_angle", 10);
        }

        sub_foot_ = this->create_subscription<geometry_msgs::msg::Point>(
            "foot_target", 10,
            std::bind(&LegController::footCallback, this, std::placeholders::_1));

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(2ms, std::bind(&LegController::update, this));

        RCLCPP_INFO(
            this->get_logger(),
            "LegController ready (L1=%.4f L2=%.4f stance_z=%.3f step=%.3f/%.3f T=%.2fs 1000Hz)",
            thigh_len, calf_len, stance_z_, step_length_, step_height_, gait_period_);
    }

private:
    void update() {
        const auto feet = walk();
        t_ += dt_;

        for (int i = 0; i < 4; ++i) {
            const auto angles = solver_->solve({0, 0, 0});

            std_msgs::msg::Float64 hip_msg;
            std_msgs::msg::Float64 knee_msg;
            hip_msg.data  = angles.hip * 180.0 / std::numbers::pi;
            knee_msg.data = angles.knee * 180.0 / std::numbers::pi;

            pub_hip_[i]->publish(hip_msg);
            pub_knee_[i]->publish(knee_msg);
        }
    }

    std::array<Eigen::Vector3d, 4> walk() {
        std::array<Eigen::Vector3d, 4> foot;
        // Trot: LF(0)+RB(2) phase 0; LB(1)+RF(3) phase 0.5
        static constexpr double kPhase[4] = {0.0, 0.5, 0.0, 0.5};

        const double period = std::max(gait_period_, 1e-3);
        for (int i = 0; i < 4; ++i) {
            double s = std::fmod(t_ / period + kPhase[i], 1.0);
            if (s < 0.0) {
                s += 1.0;
            }

            double y = stance_y_;
            double z = stance_z_;
            if (s < 0.5) {
                const double u = 2.0 * s;
                y              = stance_y_ + step_length_ * (2.0 * u - 1.0);
                z              = stance_z_ + step_height_ * std::sin(std::numbers::pi * u);
            } else {
                const double v = 2.0 * (s - 0.5);
                y              = stance_y_ + step_length_ * (1.0 - 2.0 * v);
                z              = stance_z_;
            }
            foot[i] = Eigen::Vector3d(0.0, y, z);
        }
        return foot;
    }

    void footCallback(const geometry_msgs::msg::Point::SharedPtr msg) { (void)msg; }

    std::unique_ptr<LegSolver> solver_;

    double stance_y_            = 0.0;
    double stance_z_            = -0.08;
    double step_length_         = 0.02;
    double step_height_         = 0.012;
    double gait_period_         = 1.0;
    double t_                   = 0.0;
    static constexpr double dt_ = 0.002;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_hip_[4];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_knee_[4];
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_foot_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace dogbot_core::controller

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::controller::LegController)
