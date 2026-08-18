#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>

namespace dogbot_core::controller::action {

// 角度误差归一化到 [-π, π]（IMU yaw 为无界陀螺积分，越过 ±π 需回绕）。
inline double wrap_angle(double a) {
    constexpr double kPi = std::numbers::pi;
    a                    = std::fmod(a, 2.0 * kPi);
    if (a > kPi) {
        a -= 2.0 * kPi;
    } else if (a < -kPi) {
        a += 2.0 * kPi;
    }
    return a;
}

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
        double distance_m     = 14.0; // 直行距离 (m)
        double vx             = 3.5;  // 直行线速度 (m/s)
        double omega_max      = 0.8;  // 转弯角速度上限 (rad/s)
        double turn_gain      = 3.0;  // 转角 P 控制增益 (1/s)
        double angle_tol_deg  = 2.0;  // 转角到位死区（度）
        double min_turn_time  = 0.5;  // 转弯阶段最短持续时间 (s)
        double turn_sign      = 1.0;  // 右转方向符号（±1，实机反向时置反）
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

    // 每帧调用：输入当前 IMU yaw（弧度制）与 pitch（弧度制，Climb 判完成用，
    // Place 忽略），回传线速度/角速度；返回 true 表示整个动作已完成（Done 阶段）。
    bool update(double yaw, double, double& vx, double& omega) {
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

    Phase phase_ = Phase::Done;
    Params params_{};
    double initial_yaw_ = 0.0;
    bool started_       = false;
    std::chrono::steady_clock::time_point phase_start_{};
};

// 爬坡动作（Climb）：恒定线速度前进，同时按 start() 时记录的 yaw 闭环保持；
// 完成判定基于 pitch 事件——pitch 超过上坡阈值后回落到完成阈值以下，视为爬坡完成。
// 若实机坡度不足触发 pitch_rise_deg，将持续前进直到 abort()。
//
// 约定与调用方式：
//  - start() 传入动作开始时的 IMU yaw（弧度制，作为保持目标），随后每帧把
//    IMU yaw 与 pitch 喂给 update()，update 回传线速度 vx (m/s) 与角速度
//    omega (rad/s)，返回 true 即完成标志（完成后输出零速度）。
//  - yaw 保持为闭环 P 控制（与 Place 转弯同一思路），误差按 [-π, π] 归一化。
//  - pitch 完成判定两阶段：pitch·sign > pitch_rise 进入 armed，随后
//    pitch·sign < pitch_fall 且已运行 ≥ min_run_time 视为完成（min_run_time
//    防起始时 pitch 已超阈值、噪声抖动导致瞬间误判）。
//  - pitch_sign：上坡方向符号（±1），由 IMU 安装方向决定，实机反向时置反。
class Climb {
public:
    struct Params {
        double vx             = 0.3; // 恒定线速度 (m/s)
        double omega_max      = 0.5; // yaw 保持角速度上限 (rad/s)
        double yaw_gain       = 2.0; // yaw 保持 P 控制增益 (1/s)
        double angle_tol_deg  = 2.0; // yaw 死区（度）
        double pitch_rise_deg = 8.0; // pitch 超过该值判定上坡 (度)
        double pitch_fall_deg = 0.0; // 上坡后 pitch 回落低于该值判定完成 (度)
        double pitch_sign     = 1.0; // 上坡方向符号（±1，实机反向时置反）
        double min_run_time   = 1.0; // 最短运行时间 (s)，防抖动误判完成
    };

    // 开始动作：记录 yaw 保持目标。仅首个 start 生效，abort() 复位后才可再次 start。
    void start(double initial_yaw) { start(initial_yaw, Params{}); }
    void start(double initial_yaw, const Params& params) {
        if (started_) {
            return;
        }
        started_    = true;
        params_     = params;
        yaw_target_ = initial_yaw;
        armed_      = false;
        finished_   = false;
        start_time_ = std::chrono::steady_clock::now();
    }

    // 中止：复位 start 标志与完成状态，后续 update() 输出零速度并返回完成。
    void abort() {
        started_  = false;
        finished_ = false;
        armed_    = false;
    }

    // 每帧调用：输入当前 IMU yaw 与 pitch（弧度制），回传线速度/角速度；
    // 返回 true 表示爬坡已完成（完成后输出零速度）。
    bool update(double yaw, double pitch, double& vx, double& omega) {
        vx    = 0.0;
        omega = 0.0;
        if (finished_) {
            return true;
        }

        // 恒定线速度 + yaw 闭环保持（比例控制，限幅）。
        vx               = params_.vx;
        const double err = wrap_angle(yaw_target_ - yaw);
        omega            = std::abs(err) < angle_tol()
                             ? 0.0
                             : std::clamp(err * params_.yaw_gain, -params_.omega_max, params_.omega_max);

        // pitch 完成判定：先超过上坡阈值（armed），再回落到完成阈值以下且
        // 运行时间 ≥ min_run_time 视为爬坡完成。
        const double pitched = params_.pitch_sign * pitch;
        if (pitched > pitch_rise()) {
            armed_ = true;
        }
        if (armed_ && pitched < pitch_fall() && elapsed() >= params_.min_run_time) {
            finished_ = true;
            vx        = 0.0;
            omega     = 0.0;
            return true;
        }
        return false;
    }

private:
    double pitch_rise() const { return params_.pitch_rise_deg * std::numbers::pi / 180.0; }
    double pitch_fall() const { return params_.pitch_fall_deg * std::numbers::pi / 180.0; }
    double angle_tol() const { return params_.angle_tol_deg * std::numbers::pi / 180.0; }

    double elapsed() const {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_)
            .count();
    }

    Params params_{};
    double yaw_target_ = 0.0;
    bool started_      = false;
    bool armed_        = false;
    bool finished_     = false;
    std::chrono::steady_clock::time_point start_time_{};
};

// 动作管理器：封装 Place 与 Climb，通过 start_xxx 选择激活动作，
// update() 统一分发，abort() 全部中止。同一时刻仅一个动作处于激活状态。
class Action {
public:
    enum class Type { None, Place, Climb };

    void start_place(double initial_yaw) { start_place(initial_yaw, Place::Params{}); }
    void start_place(double initial_yaw, const Place::Params& params) {
        place_.start(initial_yaw, params);
        active_ = Type::Place;
    }

    void start_climb(double initial_yaw) { start_climb(initial_yaw, Climb::Params{}); }
    void start_climb(double initial_yaw, const Climb::Params& params) {
        climb_.start(initial_yaw, params);
        active_ = Type::Climb;
    }

    void abort() {
        place_.abort();
        climb_.abort();
        active_ = Type::None;
    }

    // 按当前激活动作分发；无激活动作时输出零速度并返回完成。
    bool update(double yaw, double pitch, double& vx, double& omega) {
        switch (active_) {
        case Type::Place: return place_.update(yaw, pitch, vx, omega);
        case Type::Climb: return climb_.update(yaw, pitch, vx, omega);
        case Type::None:
        default:
            vx    = 0.0;
            omega = 0.0;
            return true;
        }
    }

private:
    Place place_;
    Climb climb_;
    Type active_ = Type::None;
};

} // namespace dogbot_core::controller::action
