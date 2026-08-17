#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>

namespace dogbot_core::controller::action {

// 放下动作（Place）：组合动作——右转 45° → 直行一段 → 转回初始角 → 再左转 45°，
// 最终朝向相对初始角为左转方向（与右转方向相反）的 45°。
//
// 约定与调用方式：
//  - start() 传入动作开始时的 IMU yaw（弧度制，作为初始角），随后每帧把 IMU yaw
//    喂给 update()，update 回传线速度 vx (m/s) 与角速度 omega (rad/s)，供调用方
//    喂给 Gait::set_gait_param()。
//  - 无里程计：直行距离为开环估计（距离 = vx × 时间），内部用 steady_clock 计时，
//    与 update() 调用频率无关；实机打滑会影响精度，distance_m 需实机标定。
//  - 转弯为闭环 P 控制（仅比例项，不积分），误差按 [-π, π] 归一化；到位判定 =
//    |误差| < 死区 且 相内耗时 ≥ 最短转弯时间（防初始误差恰好小于死区时阶段瞬间跳过）。
//  - turn_sign：右转方向符号（±1），由 IMU 安装方向决定，实机反向时置反。
class Place {
public:
    struct Params {
        double turn_angle_deg = 45.0; // 右/左转角度（度）
        double distance_m     = 0.5;  // 直行距离 (m)
        double vx             = 0.1;  // 直行线速度 (m/s)
        double omega_max      = 0.8;  // 转弯角速度上限 (rad/s)
        double turn_gain      = 3.0;  // 转角 P 控制增益 (1/s)
        double angle_tol_deg  = 2.0;  // 转角到位死区（度）
        double min_turn_time  = 0.5;  // 转弯阶段最短持续时间 (s)
        double turn_sign      = -1.0; // 右转方向符号（±1，实机反向时置反）
    };

    // 开始动作：记录初始角，从右转阶段开始执行。仅首个 start 生效，
    // 重复调用直接忽略（不覆盖初始角度），abort() 复位后才可再次 start。
    void start(double initial_yaw) { start(initial_yaw, Params{}); }
    void start(double initial_yaw, const Params& params) {
        if (started_) {
            return;
        }
        started_     = true;
        params_      = params;
        initial_yaw_ = initial_yaw;
        begin_phase(Phase::TurnRight);
    }

    // 中止：复位 start 标志并回到 Done，后续 update() 输出零速度并返回完成。
    void abort() {
        started_ = false;
        begin_phase(Phase::Done);
    }

    // 每帧调用：输入当前 IMU yaw（弧度制），回传线速度/角速度；
    // 返回 true 表示整个动作已完成（Done 阶段）。
    bool update(double yaw, double& vx, double& omega) {
        vx    = 0.0;
        omega = 0.0;
        switch (phase_) {
        case Phase::TurnRight:
            if (turn_control(yaw, initial_yaw_ + params_.turn_sign * turn_angle(), omega)) {
                begin_phase(Phase::MoveForward);
            }
            break;
        case Phase::MoveForward: {
            // 直行：按时间开环计距，distance/vx 秒后进入转回阶段。
            const double travel_time = params_.distance_m / params_.vx;
            if (params_.distance_m <= 0.0 || params_.vx <= 0.0 || phase_elapsed() >= travel_time) {
                begin_phase(Phase::TurnBack);
            } else {
                vx = params_.vx;
            }
            break;
        }
        case Phase::TurnBack:
            if (turn_control(yaw, initial_yaw_, omega)) {
                begin_phase(Phase::TurnLeft);
            }
            break;
        case Phase::TurnLeft:
            if (turn_control(yaw, initial_yaw_ - params_.turn_sign * turn_angle(), omega)) {
                begin_phase(Phase::Done);
            }
            break;
        case Phase::Done:
        default: return true;
        }
        return phase_ == Phase::Done;
    }

private:
    enum class Phase { TurnRight, MoveForward, TurnBack, TurnLeft, Done };

    double turn_angle() const { return params_.turn_angle_deg * std::numbers::pi / 180.0; }
    double angle_tol() const { return params_.angle_tol_deg * std::numbers::pi / 180.0; }

    double phase_elapsed() const {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - phase_start_)
            .count();
    }

    void begin_phase(Phase phase) {
        phase_       = phase;
        phase_start_ = std::chrono::steady_clock::now();
    }

    // 转弯 P 控制：vx 恒为 0，omega 按误差比例输出并限幅；
    // 返回 true 表示已到位（|误差| < 死区 且 相内耗时 ≥ 最短转弯时间）。
    bool turn_control(double yaw, double target, double& omega) {
        const double err = wrap_angle(target - yaw);
        if (std::abs(err) < angle_tol() && phase_elapsed() >= params_.min_turn_time) {
            omega = 0.0;
            return true;
        }
        // 已到位但最短时间未到：保持静止等待，避免相位抖动。
        omega = std::abs(err) < angle_tol()
                  ? 0.0
                  : std::clamp(err * params_.turn_gain, -params_.omega_max, params_.omega_max);
        return false;
    }

    // 角度误差归一化到 [-π, π]（IMU yaw 为无界陀螺积分，越过 ±π 需回绕）。
    static double wrap_angle(double a) {
        constexpr double kPi = std::numbers::pi;
        a                    = std::fmod(a, 2.0 * kPi);
        if (a > kPi) {
            a -= 2.0 * kPi;
        } else if (a < -kPi) {
            a += 2.0 * kPi;
        }
        return a;
    }

    Phase phase_ = Phase::Done;
    Params params_{};
    double initial_yaw_ = 0.0;
    bool started_       = false;
    std::chrono::steady_clock::time_point phase_start_{};
};

// 预留动作空壳：与 Place 接口一致（start/abort/update），行为为空——start 不做事，
// update 直接返回完成，待后续填充具体动作。
class StubAction {
public:
    // 参数类型与 Place 保持一致，当前未使用。
    using Params = Place::Params;

    void start(double) {}
    void start(double, const Params&) {}
    void abort() {}

    bool update(double, double& vx, double& omega) {
        vx    = 0.0;
        omega = 0.0;
        return true;
    }
};

// 动作管理器：封装 Place 与预留的 StubAction，通过 start_xxx 选择激活动作，
// update() 统一分发，abort() 全部中止。同一时刻仅一个动作处于激活状态。
class Action {
public:
    enum class Type { None, Place, Other };

    void start_place(double initial_yaw) { start_place(initial_yaw, Place::Params{}); }
    void start_place(double initial_yaw, const Place::Params& params) {
        place_.start(initial_yaw, params);
        active_ = Type::Place;
    }

    void start_other(double initial_yaw) { start_other(initial_yaw, StubAction::Params{}); }
    void start_other(double initial_yaw, const StubAction::Params& params) {
        other_.start(initial_yaw, params);
        active_ = Type::Other;
    }

    void abort() {
        place_.abort();
        other_.abort();
        active_ = Type::None;
    }

    // 按当前激活动作分发；无激活动作时输出零速度并返回完成。
    bool update(double yaw, double& vx, double& omega) {
        switch (active_) {
        case Type::Place: return place_.update(yaw, vx, omega);
        case Type::Other: return other_.update(yaw, vx, omega);
        case Type::None:
        default:
            vx    = 0.0;
            omega = 0.0;
            return true;
        }
    }

private:
    Place place_;
    StubAction other_;
    Type active_ = Type::None;
};

} // namespace dogbot_core::controller::action
