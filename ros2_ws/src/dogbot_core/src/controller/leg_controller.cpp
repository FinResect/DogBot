#include "action/action.hpp"
#include "controller/gait.hpp"
#include "controller/leg_solver.hpp"
#include "controller_mode.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <dogbot_msg/msg/gamepad_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <numbers>
#include <rclcpp/logging.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int64.hpp>
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

        vision_timeout_ = this->declare_parameter("vision_timeout", 0.5);
        vision_cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/vision/following/controller", 10,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                vision_twist_     = *msg;
                last_vision_time_ = this->now();
            });

        turn_omega_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/vision/color/turn_omega", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { turn_omega_ = msg->data; });

        thrower_controller_sub_ = create_subscription<std_msgs::msg::Int64>(
            "/vision/color/thrower_controller", 10,
            [this](const std_msgs::msg::Int64::SharedPtr msg) { thrower_controller_ = msg->data; });

        place_controller_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/vision/color/place_controller", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { place_controller_ = msg->data; });

        climb_controller_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/vision/color/climb_controller", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) { climb_controller_ = msg->data; });

        // 动态抬腿高度参数（上坡/下坡时按 IMU pitch 调整摆动腿抬脚高度）：
        //  - pitch_lift_gain：加高增益 (m/rad)，每 1 弧度 pitch 增加多少抬脚高度；
        //    例 10° 坡 ≈ 0.175 rad × 0.2 ≈ +35mm
        //  - max_extra_lift：单腿加高上限 (m)，防止大坡度抬脚过高超出舵机速度能力
        //  - pitch_deadband_deg：死区（度），低于此角度的 pitch 视为平地不加高，
        //    滤除行走振动与平地微小倾斜
        //  - pitch_ema_alpha：IMU pitch 低通系数 (0~1)，越大响应越快越抖、越小越平滑越滞后
        //  - pitch_sign：符号修正（±1），pitch 正负对应上坡/下坡依赖 IMU 安装方向，
        //    实机若发现该加高的腿反了则置 -1
        //  - stride_reduce_gain：高度↑ 步幅↓ 折算系数，
        //    stride_scale = clamp(1 − gain·ΔH/kDefaultStepHeight, 0.5, 1.0)
        pitch_lift_gain_ = this->declare_parameter("pitch_lift_gain", 0.0);
        max_extra_lift_  = this->declare_parameter("max_extra_lift", 0.02);
        pitch_deadband_ =
            this->declare_parameter("pitch_deadband_deg", 2.0) * std::numbers::pi / 180.0;
        pitch_ema_alpha_    = this->declare_parameter("pitch_ema_alpha", 0.2);
        pitch_sign_         = this->declare_parameter("pitch_sign", -1.0);
        stride_reduce_gain_ = this->declare_parameter("stride_reduce_gain", 0.5);

        // yaw 航向纠偏参数（IMU yaw 为弧度制，参考航向 = 指令角速度纯积分）：
        //  - yaw_stab_gain：P 增益 (1/s)，误差→纠正角速度的换算系数
        //  - yaw_stab_max：纠正角速度上限 (rad/s)，稳态最大偏航 ≈ yaw_stab_max/gain
        //  - yaw_stab_timeout：参考航向失同步超时 (s)，超过则重新吸附到实际 yaw
        //  - yaw_sign：符号修正（±1），IMU 安装方向反时置 -1
        yaw_stab_gain_    = this->declare_parameter("yaw_stab_gain", 2.0);
        yaw_stab_max_     = this->declare_parameter("yaw_stab_max", 0.8);
        yaw_stab_timeout_ = this->declare_parameter("yaw_stab_timeout", 0.2);
        yaw_sign_         = this->declare_parameter("yaw_sign", 1.0);

        imu_pitch_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/imu/pitch", 10, [this](const std_msgs::msg::Float64::SharedPtr msg) {
                pitch_filt_ = pitch_ema_alpha_ * msg->data + (1.0 - pitch_ema_alpha_) * pitch_filt_;
            });
        imu_yaw_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/imu/yaw", 10,
            [this](const std_msgs::msg::Float64::SharedPtr msg) { imu_yaw_ = msg->data; });

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

        using namespace dogbot_msg::msg;

        auto button_x     = gamepad_state_.buttons.x;
        auto button_y     = gamepad_state_.buttons.y;
        auto button_a     = gamepad_state_.buttons.a;
        auto button_b     = gamepad_state_.buttons.b;
        auto button_up    = gamepad_state_.dpad.up;
        auto button_left  = gamepad_state_.dpad.left;
        auto button_right = gamepad_state_.dpad.right;
        auto button_start = gamepad_state_.buttons.start;
        auto button_mode  = gamepad_state_.buttons.mode;
        auto button_l1    = gamepad_state_.shoulders.l1;

        static std_msgs::msg::Bool left_msg;
        static std_msgs::msg::Bool right_msg;

        if (gamepad_state_.status == GamepadState::UNKNOWN
            || gamepad_state_.status == GamepadState::DISCONNECTED
            || (button_mode && !last_button_mode)) {
            reset_all_controller();
            controller_mode_ = dogbot_msg::ControllerMode::None;
            return;
        }

        if (button_left && !last_button_left) {
            left_msg.data = !left_msg.data;
            pub_thrower_left_->publish(left_msg);
        } else if (button_right && !last_button_right) {
            right_msg.data = !right_msg.data;
            pub_thrower_right_->publish(right_msg);
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
        } else if (button_l1 && !last_button_l1) {
            gait_.set_gait_type(GaitType::TrotSpinMix);
            controller_mode_ = dogbot_msg::ControllerMode::Vision;
        }

        controller_update();

        last_button_a     = button_a;
        last_button_b     = button_b;
        last_button_x     = button_x;
        last_button_y     = button_y;
        last_button_up    = button_up;
        last_button_left  = button_left;
        last_button_right = button_right;
        last_button_start = button_start;
        last_button_mode  = button_mode;
        last_button_l1    = button_l1;
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

        // IMU pitch → 逐腿摆动抬脚高度 + 步幅缩放：
        // 上坡（抬头，pitch_sign 修正后为正）重心靠后，后腿 (LB=1, RB=2) 抬脚被
        // 压缩，加高 ΔH；下坡（低头）前腿 (LF=0, RF=3) 加高。ΔH 死区 + 限幅。
        // 高度↑ 步幅↓：摆动相峰值速度 = 2*stride/T_swing（水平）+ π*H/T_swing
        // （竖直），抬脚加高后按比例收窄步幅上限，保住舵机速度余量。
        const double pitched = pitch_sign_ * pitch_filt_;
        double extra         = 0.0;
        if (std::abs(pitched) > pitch_deadband_) {
            extra = std::clamp(pitch_lift_gain_ * std::abs(pitched), 0.0, max_extra_lift_);
        }

        std::array<double, 4> lift;
        lift.fill(Gait::kDefaultStepHeight);
        if (extra > 0.0) {
            if (pitched > 0.0) {
                lift[1] += extra;
                lift[2] += extra;
            } else {
                lift[0] += extra;
                lift[3] += extra;
            }
        }
        const double stride_scale =
            std::clamp(1.0 - stride_reduce_gain_ * (extra / Gait::kDefaultStepHeight), 0.5, 1.0);

        gait_.set_swing_profile(lift, stride_scale);
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

            pub_hip_[i]->publish(hip_msg);
            pub_knee_[i]->publish(knee_msg);
        }
    }

    void thrower_controller(bool left, bool right) {
        std_msgs::msg::Bool left_msg;
        std_msgs::msg::Bool right_msg;

        left_msg.data  = left;
        right_msg.data = right;

        pub_thrower_left_->publish(left_msg);
        pub_thrower_right_->publish(right_msg);
    }

    // IMU yaw → 机头航向纠偏：参考航向 yaw_ref_ 为指令角速度的纯积分，
    // 实际 yaw 偏离参考时叠加比例纠正角速度把机体拉回。
    // 纯比例、无死区：任意非零误差都产生纠正量（小误差小纠正、大误差饱和），
    // 无 bang-bang 阈值跳变；纠正量限幅 yaw_stab_max_，
    // 稳态最大偏航 ≈ yaw_stab_max_ / yaw_stab_gain_。
    // 转弯指令使参考同步积分，因此只抵制非指令性漂移/打滑，不干扰正常转向；
    // 失同步（长时间未调用/模式切换/断连）时参考重新吸附到实际 yaw，避免回拉冲击。
    double yaw_stabilize(double omega_cmd) {
        const auto now = std::chrono::steady_clock::now();
        if (std::chrono::duration<double>(now - last_yaw_stab_time_).count() > yaw_stab_timeout_) {
            yaw_ref_ = imu_yaw_;
        }
        last_yaw_stab_time_ = now;

        yaw_ref_ += omega_cmd * dt_;
        const double err = action::wrap_angle(imu_yaw_ - yaw_ref_);
        const double correction =
            std::clamp(-yaw_sign_ * yaw_stab_gain_ * err, -yaw_stab_max_, yaw_stab_max_);
        return omega_cmd + correction;
    }

    void controller_update() {
        switch (controller_mode_) {
        case dogbot_msg::ControllerMode::Auto: solver_update(feet_update(-0.2, -5.0)); break;
        case dogbot_msg::ControllerMode::Manual:
            solver_update(feet_update(
                -0.4 * gamepad_state_.sticks.joystick_left.y,
                yaw_stabilize(10.0 * gamepad_state_.sticks.joystick_right.x)));
            break;
        case dogbot_msg::ControllerMode::Vision: {
            bool stale   = (this->now() - last_vision_time_).seconds() > vision_timeout_;
            double vx    = stale ? 0.0 : -vision_twist_.linear.x;
            double omega = stale ? 0.0 : yaw_stabilize(vision_twist_.angular.z);

            if (turn_omega_ != 0.0) {
                vx    = 0.0;
                omega = yaw_stabilize(turn_omega_);
            } else if (thrower_controller_ == 0) {
                thrower_controller(false, false);
            } else if (thrower_controller_ == 1) {
                thrower_controller(true, false);
                vx    = 0.0;
                omega = 0.0;
            } else if (thrower_controller_ == 2) {
                thrower_controller(false, true);
                vx    = 0.0;
                omega = 0.0;
            } else if (place_controller_) {
                action_.start_place(imu_yaw_);
                action_.update(imu_yaw_, pitch_filt_, vx, omega);
                yaw_stabilize(omega);
            } else if (climb_controller_) {
                action_.start_climb(imu_yaw_);
                action_.update(imu_yaw_, pitch_filt_, vx, omega);
            } else {
                action_.abort();
                thrower_controller(false, false);
                yaw_stabilize(omega);
            }

            solver_update(feet_update(vx, omega));
            break;
        }
        default: reset_all_controller(); break;
        }
    }

    dogbot_msg::msg::GamepadState gamepad_state_;
    Gait gait_;
    std::unique_ptr<LegSolver> solver_;
    dogbot_msg::ControllerMode controller_mode_{dogbot_msg::ControllerMode::None};
    action::Action action_;

    uint8_t last_button_x     = 0;
    uint8_t last_button_y     = 0;
    uint8_t last_button_a     = 0;
    uint8_t last_button_b     = 0;
    uint8_t last_button_up    = 0;
    uint8_t last_button_left  = 0;
    uint8_t last_button_right = 0;
    uint8_t last_button_start = 0;
    uint8_t last_button_mode  = 0;
    uint8_t last_button_l1    = 0;

    double theta_               = 0.0;
    static constexpr double dt_ = 0.002;

    geometry_msgs::msg::Twist vision_twist_;
    rclcpp::Time last_vision_time_{0, 0, RCL_ROS_TIME};
    double vision_timeout_      = 0.5;
    double turn_omega_          = 0.0;
    int64_t thrower_controller_ = 0;
    bool place_controller_      = false;
    bool climb_controller_      = false;

    rclcpp::Subscription<dogbot_msg::msg::GamepadState>::SharedPtr gamepad_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr theta_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr imu_pitch_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr imu_yaw_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vision_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr turn_omega_sub_;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr thrower_controller_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr place_controller_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr climb_controller_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_hip_[4];
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_knee_[4];
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_thrower_left_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_thrower_right_;
    rclcpp::TimerBase::SharedPtr timer_;

    double imu_yaw_ = 0.0;

    double pitch_filt_         = 0.0;  // IMU pitch 低通后（弧度）
    double pitch_lift_gain_    = 0.1;  // ΔH 增益（m/rad）
    double max_extra_lift_     = 0.02; // 抬脚加高上限 (m)
    double pitch_deadband_     = 0.0;  // 死区（弧度）
    double pitch_ema_alpha_    = 0.2;  // EMA 系数
    double pitch_sign_         = 1.0;  // pitch 符号修正（实机方向反时置 -1）
    double stride_reduce_gain_ = 0.5;  // 高度↑ 步幅↓ 折算系数

    double yaw_ref_          = 0.0;    // 参考航向（指令角速度纯积分，弧度）
    double yaw_stab_gain_    = 2.0;    // yaw 纠偏 P 增益 (1/s)
    double yaw_stab_max_     = 0.8;    // 纠正角速度上限 (rad/s)
    double yaw_stab_timeout_ = 0.2;    // 参考失同步重对齐超时 (s)
    double yaw_sign_         = 1.0;    // yaw 符号修正（实机方向反时置 -1）
    std::chrono::steady_clock::time_point last_yaw_stab_time_{}; // 上次纠偏调用时刻
};

} // namespace dogbot_core::controller

RCLCPP_COMPONENTS_REGISTER_NODE(dogbot_core::controller::LegController)
