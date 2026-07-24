#include "controller/gait_config.hpp"
#include "controller/gait_engine.hpp"
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

        double step_height = this->declare_parameter("step_height", 0.03);
        double gait_period = this->declare_parameter("gait_period", 0.6);

        GaitParams gait_params  = makeTrotParams(gait_period);
        gait_params.z_clearance = step_height;
        gait_type_ = parseGaitType(this->declare_parameter("gait_mode", std::string("trot")));

        double stance_y = this->declare_parameter("stance_y", 0.0);
        double stance_z = this->declare_parameter("stance_z", 0.13);
        std::array<LegState, kLegCount> legs{};
        for (auto& leg : legs) {
            leg.base_y   = stance_y;
            leg.z_stance = stance_z;
        }

        ClimbConfig climb_cfg;
        climb_cfg.step_height = this->declare_parameter("climb_step_height", 0.03);
        climb_cfg.reach       = this->declare_parameter("climb_reach", 0.04);
        climb_cfg.phase_time  = this->declare_parameter("climb_phase_time", 0.8);
        climb_cfg.steps       = this->declare_parameter("climb_steps", 1);

        gait_engine_.configure(gait_params, legs);
        gait_engine_.configureClimb(climb_cfg);
        gait_engine_.setGaitType(gait_type_);

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                for (const auto& p : params) {
                    if (p.get_name() == "gait_mode") {
                        gait_type_ = parseGaitType(p.as_string());
                        gait_engine_.setGaitType(gait_type_);
                        need_ik_reset_ = true;
                        RCLCPP_INFO(
                            this->get_logger(), "Gait switched to %s", p.as_string().c_str());
                    }
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });

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

        sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&LegController::cmdVelCallback, this, std::placeholders::_1));

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(2ms, std::bind(&LegController::update, this));

        RCLCPP_INFO(
            this->get_logger(), "LegController ready (L1=%.4f L2=%.4f stance_z=%.3f T=%.2fs 500Hz)",
            thigh_len, calf_len, stance_z, gait_engine_.cycleTime());
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (gait_type_ != GaitType::Climb) {
            gait_engine_.setTargetTwist(msg->linear.x, msg->angular.z);
        }
    }

    void update() {
        gait_engine_.step(dt_);

        if (gait_type_ == GaitType::Climb) {
            auto feet = gait_engine_.feet();
            for (int i = 0; i < 4; ++i) {
                auto angles      = solver_->solve(feet[i]);
                current_hip_[i]  = angles.hip;
                current_knee_[i] = angles.knee;
            }
            if (gait_engine_.climbDone()) {
                gait_type_ = GaitType::Stand;
                gait_engine_.setGaitType(gait_type_);
                RCLCPP_INFO(this->get_logger(), "Climb completed, switched to Stand");
            }
        } else {
            if (need_ik_reset_) {
                auto feet = gait_engine_.feet();
                for (int i = 0; i < 4; ++i) {
                    auto angles      = solver_->solve(feet[i]);
                    current_hip_[i]  = angles.hip;
                    current_knee_[i] = angles.knee;
                }
                need_ik_reset_ = false;
            } else {
                auto feet     = gait_engine_.feet();
                auto feet_vel = gait_engine_.feetVelocities();
                for (int i = 0; i < 4; ++i) {
                    auto vel = solver_->solveVelocity(feet[i], feet_vel[i]);
                    current_hip_[i] += vel.hip * dt_;
                    current_knee_[i] += vel.knee * dt_;
                }
            }
        }

        for (int i = 0; i < 4; ++i) {
            std_msgs::msg::Float64 hip_msg;
            std_msgs::msg::Float64 knee_msg;
            hip_msg.data  = current_hip_[i] * 180.0 / std::numbers::pi;
            knee_msg.data = current_knee_[i] * 180.0 / std::numbers::pi;

            pub_hip_[i]->publish(hip_msg);
            pub_knee_[i]->publish(knee_msg);
        }
    }

    static GaitType parseGaitType(const std::string& s) {
        if (s == "trot") {
            return GaitType::Trot;
        }
        if (s == "amble") {
            return GaitType::Amble;
        }
        if (s == "walk") {
            return GaitType::Walk;
        }
        if (s == "stand") {
            return GaitType::Stand;
        }
        if (s == "climb") {
            return GaitType::Climb;
        }
        return GaitType::Trot;
    }

    std::unique_ptr<LegSolver> solver_;
    GaitEngine gait_engine_;
    GaitType gait_type_ = GaitType::Trot;

    double current_hip_[4]      = {};
    double current_knee_[4]     = {};
    bool need_ik_reset_         = true;
    static constexpr double dt_ = 0.002;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_hip_[4];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_knee_[4];
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

} // namespace dogbot_core::controller

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::controller::LegController)
