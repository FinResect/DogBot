#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <optional>

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

// 手动编排方向。转向为相对锁存起点 yaw 的角度偏移，直行为速度符号：
//  - Right：偏移 = +turn_sign·angle（默认 −45°，yaw 减小 → omega < 0 = 右转）
//  - Left：偏移 = −turn_sign·angle（默认 +45°，yaw 增大 → omega > 0 = 左转）
//  - Forward：输出 vx = −|speed|（vx < 0 = 前进）；Backward：输出 vx = +|speed|
enum class Direction { Left, Right, Forward, Backward };

// 放下动作（Place）：组合动作——右转 45° → 直行一段 → 转回初始角 → 再左转 45°，
// 最终朝向相对初始角为左转方向（与右转方向相反）的 45°。
//
// 除固定序列外还暴露 turn_step / straight_step 手动编排原语（逐帧调用）：
// 参数每次调用直接生效、可任意更改（不参与状态比较），仅当步骤类型或方向切换时
// 才锁存起点 yaw / 重置计时；同一步骤内重复调用幂等。update() 返回 true 表示
// 当前步骤完成（手动模式）或整组序列完成（序列模式）。
//
// 约定与调用方式：
//  - start() 传入动作开始时的 IMU yaw（弧度制，作为初始角），随后每帧把 IMU yaw
//    喂给 update()，update 回传线速度 vx (m/s) 与角速度 omega (rad/s)，供调用方
//    喂给 Gait::set_gait_param()。
//  - 方向约定：vx < 0 前进、omega < 0 右转（依赖 IMU 安装方向，实机反向时置反
//    turn_sign）。
//  - 直行为开环计距（距离 = |vx| × 时长），内部用 steady_clock 计时，与 update()
//    调用频率无关；实机打滑影响精度，straight_time_s 需实机标定。
//  - 转弯为闭环 P 控制（仅比例项，不积分），误差按 [-π, π] 归一化；到位判定 =
//    |误差| < 死区 且 相内耗时 ≥ 最短转弯时间（防初始误差恰好小于死区时阶段瞬间跳过）。
class Place {
public:
    struct Params {
        double turn_angle_deg = 45.0; // 右/左转角度（度）
        double straight_time_s = 5.833; // 直行时长 (s)，由原 distance_m=14/(|vx|=3·0.8) 折算
        double vx            = -3.0;    // 直行线速度 (m/s)，vx<0 = 前进
        double omega_max     = 1000.0; // 转弯角速度上限 (rad/s)
        double turn_gain     = 3.0;    // 转角 P 控制增益 (1/s)
        double angle_tol_deg = 8.0;    // 转角到位死区（度）
        double min_turn_time = 0.5;    // 转弯阶段最短持续时间 (s)
        double turn_sign     = 1.0;    // 右转方向符号（±1，实机反向时置反）
    };

    // 开始动作：记录初始角，从右转阶段开始执行。仅首个 start 生效，
    // 重复调用直接忽略（不覆盖初始角度），abort() 复位后才可再次 start。
    void start(double initial_yaw) { start(initial_yaw, Params{}); }
    void start(double initial_yaw, const Params& params) {
        if (started_) {
            return;
        }
        started_      = true;
        params_       = params;
        initial_yaw_  = initial_yaw;
        phase_        = Phase::MoveForward_delay;
        step_         = Step::None;
        turn_latched_ = false;
    }

    // 中止：复位 start 标志并回到空闲，后续 update() 输出零速度并返回完成
    // （此后进入手动原语模式）。
    void abort() {
        started_      = false;
        phase_        = Phase::Done;
        step_         = Step::None;
        turn_latched_ = false;
        turn_abs_target_.reset();
    }

    // 手动编排原语：闭环转到指定角度（相对锁存起点 yaw），omega 为角速度上限。
    // 每次调用直接应用最新参数；仅当步骤类型或方向切换时锁存起点并重置计时。
    void turn_step(Direction dir, double angle_deg, double omega_lim) {
        if (step_ != Step::Turn || turn_dir_ != dir) {
            step_         = Step::Turn;
            turn_dir_     = dir;
            turn_latched_ = false;
            step_start_   = std::chrono::steady_clock::now();
        }
        turn_angle_rad_ = angle_deg * std::numbers::pi / 180.0;
        turn_omega_lim_ = omega_lim;
        turn_abs_target_.reset();
    }

    // 手动编排原语：以指定速度直行 duration 秒（Forward 输出 −|vx|，Backward 输出 +|vx|）。
    void straight_step(Direction dir, double vx, double duration) {
        if (step_ != Step::Straight || straight_dir_ != dir) {
            step_         = Step::Straight;
            straight_dir_ = dir;
            step_start_   = std::chrono::steady_clock::now();
        }
        straight_vx_       = vx;
        straight_duration_ = duration;
    }

    // 每帧调用：输入当前 IMU yaw（弧度制）与 pitch（弧度制，Climb 判完成用，
    // Place 忽略），回传线速度/角速度；返回 true 表示动作已完成（Done 阶段）。
    bool update(double yaw, double, double& vx, double& omega) {
        vx    = 0.0;
        omega = 0.0;
        if (!started_) {
            // 手动原语模式：执行当前步骤，完成时回到 None 并返回 true。
            switch (step_) {
            case Step::Turn: return turn_control(yaw, omega);
            case Step::Straight: return straight_control(vx);
            case Step::None:
            default: return true;
            }
        }
        // 固定序列模式：TurnRight → MoveForward → TurnBack → TurnLeft。
        switch (phase_) {
        case Phase::MoveForward_delay:
            if (params_.vx >= 0.0 || params_.straight_time_s <= 0.0) {
                phase_ = Phase::TurnRight;
                break;
            }
            straight_step(Direction::Forward, 0.4, 2.0);
            if (straight_control(vx)) {
                phase_ = Phase::TurnRight;
            }
            break;
        case Phase::TurnRight:
            turn_step(Direction::Right, 45.0, params_.omega_max);
            if (turn_control(yaw, omega)) {
                phase_ = Phase::MoveForward_first;
            }
            break;
        case Phase::MoveForward_first:
            // 参数非法（无前进速度/无时长）时跳过直行。
            if (params_.vx >= 0.0 || params_.straight_time_s <= 0.0) {
                phase_ = Phase::TurnBack;
                break;
            }
            straight_step(Direction::Forward, 0.4, 4.0);
            if (straight_control(vx)) {
                phase_ = Phase::TurnBack;
            }
            break;
        case Phase::TurnBack:
            // begin_turn_abs(initial_yaw_, params_.omega_max);
            turn_step(Direction::Left, 45.0, params_.omega_max);
            if (turn_control(yaw, omega)) {
                phase_ = Phase::MoveForward_second;
            }
            break;
        case Phase::MoveForward_second:
            if (params_.vx >= 0.0 || params_.straight_time_s <= 0.0) {
                phase_ = Phase::TurnLeft;
                break;
            }
            straight_step(Direction::Forward, 0.4, 3.0);
            if (straight_control(vx)) {
                phase_ = Phase::TurnLeft;
            }
            break;
        case Phase::TurnLeft:
            turn_step(Direction::Left, 45.0, params_.omega_max);
            if (turn_control(yaw, omega)) {
                phase_ = Phase::Done;
            }
            break;
        case Phase::Done:
        default: return true;
        }
        return phase_ == Phase::Done;
    }

private:
    enum class Step { None, Turn, Straight };
    enum class Phase {
        MoveForward_delay,
        TurnRight,
        MoveForward_first,
        TurnBack,
        MoveForward_second,
        TurnLeft,
        Done
    };

    // 绝对目标转弯（序列 TurnBack 用）：目标角与起始 yaw 无关，修正直行期间的 yaw 漂移。
    void begin_turn_abs(double target_rad, double omega_lim) {
        if (step_ == Step::Turn && turn_abs_target_.has_value()) {
            return;
        }
        step_            = Step::Turn;
        turn_latched_    = false;
        step_start_      = std::chrono::steady_clock::now();
        turn_abs_target_ = target_rad;
        turn_omega_lim_  = omega_lim;
    }

    double angle_tol() const { return params_.angle_tol_deg * std::numbers::pi / 180.0; }

    double step_elapsed() const {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - step_start_)
            .count();
    }

    // 相对模式的目标偏移：Right → +turn_sign·angle；Left → −turn_sign·angle。
    double turn_offset_rad() const {
        const double sign = turn_dir_ == Direction::Right ? 1.0 : -1.0;
        return sign * params_.turn_sign * turn_angle_rad_;
    }

    // 转弯 P 控制：vx 恒为 0，omega 按误差比例输出并限幅；
    // 首个调用帧锁存起点 yaw（此后角度更改不重置锚点，目标每帧重算）；
    // 返回 true 表示已到位（|误差| < 死区 且 相内耗时 ≥ 最短转弯时间）。
    bool turn_control(double yaw, double& omega) {
        if (!turn_latched_) {
            turn_start_yaw_ = yaw;
            turn_latched_   = true;
        }
        const double target =
            turn_abs_target_.has_value() ? *turn_abs_target_ : turn_start_yaw_ + turn_offset_rad();
        const double err = wrap_angle(target - yaw);
        if (std::abs(err) < angle_tol() && step_elapsed() >= params_.min_turn_time) {
            omega = 0.0;
            step_ = Step::None;
            return true;
        }
        // 已到位但最短时间未到：保持静止等待，避免相位抖动。
        omega = std::abs(err) < angle_tol()
                  ? 0.0
                  : std::clamp(err * params_.turn_gain, -turn_omega_lim_, turn_omega_lim_);
        return false;
    }

    // 直行：按时间输出 ±|vx|，时长到后清零并回到 None。
    bool straight_control(double& vx) {
        if (step_elapsed() >= straight_duration_) {
            vx    = 0.0;
            step_ = Step::None;
            return true;
        }
        vx = straight_dir_ == Direction::Forward ? -std::abs(straight_vx_) : std::abs(straight_vx_);
        return false;
    }

    Step step_             = Step::None;
    Direction turn_dir_    = Direction::Left;
    double turn_angle_rad_ = 0.0;           // 当前生效转角（弧度）
    double turn_omega_lim_ = 0.0;           // 当前生效角速度上限 (rad/s)
    double turn_start_yaw_ = 0.0;           // 锁存起点 yaw（弧度）
    bool turn_latched_     = false;
    std::optional<double> turn_abs_target_; // 绝对目标（弧度），无则相对偏移
    Direction straight_dir_   = Direction::Forward;
    double straight_vx_       = 0.0;        // 当前生效直行速度 (m/s)
    double straight_duration_ = 0.0;        // 当前生效直行时长 (s)
    std::chrono::steady_clock::time_point step_start_{};

    Phase phase_ = Phase::Done;
    Params params_{};
    double initial_yaw_ = 0.0;
    bool started_       = false;
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
        double vx             = -0.3;   // 恒定线速度 (m/s)
        double omega_max      = 1000.0; // yaw 保持角速度上限 (rad/s)
        double yaw_gain       = 6.0;    // yaw 保持 P 控制增益 (1/s)
        double angle_tol_deg  = 0.0;    // yaw 死区（度）
        double pitch_rise_deg = 15.0;   // pitch 超过该值判定上坡 (度)
        double pitch_fall_deg = 0.0;    // 上坡后 pitch 回落低于该值判定完成 (度)
        double pitch_sign     = 1.0;    // 上坡方向符号（±1，实机反向时置反）
        double min_run_time   = 3.0;    // 最短运行时间 (s)，防抖动误判完成
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
        vx = 0.0;
        if (finished_) {
            return true;
        }

        // 恒定线速度 + yaw 闭环保持（比例控制，限幅）。
        vx               = params_.vx;
        const double err = wrap_angle(yaw_target_ - yaw);
        omega += std::abs(err) < angle_tol()
                   ? 0.0
                   : std::clamp(err * params_.yaw_gain, -params_.omega_max, params_.omega_max);
        omega = std::clamp(omega, -params_.omega_max, params_.omega_max);

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
