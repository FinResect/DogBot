#include "controller/gait.hpp"
#include "controller/leg_solver.hpp"
#include "controller/spin_gait.hpp"

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

        double stance_y    = this->declare_parameter("stance_y", 0.0);
        double stance_z    = this->declare_parameter("stance_z", 0.13);
        double step_length = this->declare_parameter("step_length", 0.04);
        double step_height = this->declare_parameter("step_height", 0.03);
        double gait_period = this->declare_parameter("gait_period", 0.6);
        gait_mode_         = parseGaitType(this->declare_parameter("gait_mode", std::string("trot")));

        gait_ = std::make_unique<Gait>();
        gait_->setParams(stance_y, stance_z, step_length, step_height, gait_period);
        gait_->setClimbParams(
            this->declare_parameter("climb_step_height", 0.03),
            this->declare_parameter("climb_reach", 0.04),
            this->declare_parameter("climb_phase_time", 0.8),
            this->declare_parameter("climb_steps", 1));

        double spin_gain = this->declare_parameter("spin_gain", 1.0);
        spin_omega_ = this->declare_parameter("spin_omega", 0.0);
        trot_vx_ = this->declare_parameter("trot_vx", 0.0);
        spin_gait_ = std::make_unique<SpinGait>();
        spin_gait_->setParams(stance_y, stance_z, step_height, gait_period, spin_gain);

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

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                for (const auto& p : params) {
                    if (p.get_name() == "gait_mode") {
                        gait_mode_ = parseGaitType(p.as_string());
                        gait_->reset();
                        spin_gait_->reset();
                        RCLCPP_INFO(
                            this->get_logger(), "Gait switched to %s", p.as_string().c_str());
                    } else if (p.get_name() == "spin_omega") {
                        spin_omega_ = p.as_double();
                    } else if (p.get_name() == "trot_vx") {
                        trot_vx_ = p.as_double();
                    }
                }
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });

        using namespace std::chrono_literals;
        timer_ = this->create_wall_timer(2ms, std::bind(&LegController::update, this));

        RCLCPP_INFO(
            this->get_logger(),
            "LegController ready (L1=%.4f L2=%.4f stance_z=%.3f step=%.3f/%.3f T=%.2fs)",
            thigh_len, calf_len, stance_z, step_length, step_height, gait_period);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        vx_      = msg->linear.x;
        omega_z_ = msg->angular.z;
    }

    void update() {
        std::array<Eigen::Vector3d, 4> feet;

        const double vx = std::abs(trot_vx_) > 1e-6 ? trot_vx_ : vx_;

        if (gait_mode_ == GaitType::Spin) {
            const double omega = std::abs(spin_omega_) > 1e-6 ? spin_omega_ : omega_z_;
            feet = spin_gait_->step(dt_, omega);
        } else if (gait_mode_ == GaitType::TrotSpinMix) {
            const double omega = std::abs(spin_omega_) > 1e-6 ? spin_omega_ : omega_z_;
            feet = gait_->stepSpinMix(*spin_gait_, dt_, vx, omega);
        } else {
            feet = gait_->step(gait_mode_, dt_, vx, omega_z_);

            if (gait_mode_ == GaitType::Climb && gait_->climbDone()) {
                gait_mode_ = GaitType::Stand;
                gait_->reset();
                RCLCPP_INFO(this->get_logger(), "Climb completed, switched to Stand");
            }
        }

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

    static GaitType parseGaitType(const std::string& s) {
        if (s == "stand") {
            return GaitType::Stand;
        }
        if (s == "amble") {
            return GaitType::Amble;
        }
        if (s == "walk") {
            return GaitType::Walk;
        }
        if (s == "climb") {
            return GaitType::Climb;
        }
        if (s == "spin") {
            return GaitType::Spin;
        }
        if (s == "trot_spin_mix") {
            return GaitType::TrotSpinMix;
        }
        return GaitType::Trot;
    }

    std::unique_ptr<Gait> gait_;
    std::unique_ptr<SpinGait> spin_gait_;
    std::unique_ptr<LegSolver> solver_;
    GaitType gait_mode_ = GaitType::Trot;

    double vx_                = 0.0;
    double omega_z_           = 0.0;
    double spin_omega_        = 0.0;
    double trot_vx_           = 0.0;
    static constexpr double dt_ = 0.002;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_hip_[4];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_knee_[4];
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

} // namespace dogbot_core::controller

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::controller::LegController)
