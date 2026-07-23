#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int8.hpp>

#include <array>
#include <memory>
#include <string>

#include "controller/gait_config.hpp"
#include "controller/gait_engine.hpp"
#include "controller/leg_solver.hpp"

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

        double stance_y  = this->declare_parameter("stance_y", 0.0);
        double stance_z  = this->declare_parameter("stance_z", -0.10);

        for (int i = 0; i < kLegCount; ++i) {
            solvers_[i] = std::make_unique<LegSolver>(
                thigh_len, calf_len, hip_offs, L3, L4, L5, r_arm, delta, fork_branch);

            legs_[i] = LegState{stance_y, stance_z};

            std::string prefix = std::string(kLegNames[i]);
            for (int j = 0; j < kServosPerLeg; ++j) {
                std::string topic = prefix + "/" + kJointSuffixes[j] + "/control_angle";
                pubs_[i][j] = this->create_publisher<std_msgs::msg::Float64>(topic, 10);
            }
        }

        GaitParams params = makeTrotParams();
        engine_.configure(params, legs_);
        engine_.setGaitType(GaitType::Trot);

        sub_vel_ = this->create_subscription<std_msgs::msg::Float64>(
            "cmd_vel", 10,
            std::bind(&LegController::velCallback, this, std::placeholders::_1));

        sub_gait_ = this->create_subscription<std_msgs::msg::Int8>(
            "gait_type", 10,
            std::bind(&LegController::gaitCallback, this, std::placeholders::_1));

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(10ms, std::bind(&LegController::controlLoop, this));

        RCLCPP_INFO(this->get_logger(),
            "LegController ready (L1=%.4f L2=%.4f, 4 legs, gait=%s)",
            thigh_len, calf_len, gaitName(engine_.gaitType()));
    }

private:
    void velCallback(const std_msgs::msg::Float64::SharedPtr msg) {
        engine_.setTargetVelocity(msg->data);
    }

    void gaitCallback(const std_msgs::msg::Int8::SharedPtr msg) {
        auto type = static_cast<GaitType>(msg->data);
        engine_.setGaitType(type);
        RCLCPP_INFO(this->get_logger(), "Gait switched to %s", gaitName(type));
    }

    void controlLoop() {
        engine_.step(0.01);

        const auto& feet = engine_.feet();
        for (int i = 0; i < kLegCount; ++i) {
            auto angles = solvers_[i]->solve(feet[i]);

            auto hip_msg  = std_msgs::msg::Float64();
            auto knee_msg = std_msgs::msg::Float64();
            hip_msg.data  = angles.hip * 180.0 / M_PI;
            knee_msg.data = angles.knee * 180.0 / M_PI;

            pubs_[i][0]->publish(hip_msg);
            pubs_[i][1]->publish(knee_msg);
        }
    }

    std::array<std::unique_ptr<LegSolver>, kLegCount> solvers_;
    std::array<LegState, kLegCount> legs_;
    std::array<std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, kServosPerLeg>, kLegCount> pubs_;

    GaitEngine engine_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_vel_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_gait_;
    rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace dogbot_core::controller

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::controller::LegController)
