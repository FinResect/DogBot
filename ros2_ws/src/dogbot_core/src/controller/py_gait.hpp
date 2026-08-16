#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>

namespace dogbot_core::controller {

// GaitType：独立定义（不与 gait.hpp 共用），值序与其一致。gait.hpp 与 py_gait.hpp
// 不会同时使用，若同时 include 二者会因同名枚举冲突。
enum class GaitType { Stand, Trot, Walk, Spin, Climb, TrotSpinMix };

// PyGait：开源项目 Py-Apple-Dynamics V7.3（菠萝狗）PA_GAIT.py 的忠实 C++ 移植。
//
// 源码参考（本仓库 py-apple-dynamics/Py Apple Dynamics V7.3 SRC/PA-Dynamics V7.3/）：
//  - PA_GAIT.py：trot / walk 相位窗口与重心（CG）门控逻辑
//  - padog.py：swing_curve_generate / support_curve_generate 摆线与支撑曲线
//  - PA_STABLIZE.py + padog.py mainloop：X_goal / X_S 重心模型与机体前移
//
// 坐标系与腿序（与 gait.hpp 保持一致）：
//  - 足端坐标 (x, y, z)：x 恒为 0；y 为腿平面水平轴（摆动/支撑的行程方向）；z 为从髋
//    向下的竖直距离，抬脚时 z 减小（kStanceZ=0.14 为站立时足端离髋的竖直距离）。
//  - 腿序 {left_front, left_back, right_back, right_front}（索引 0..3）。
//  - padog 腿序 {1,2,3,4} = {LF, RF, LB, RB}，映射到本类索引：1→0、2→3、3→1、4→2。
//  - padog 的 x（前进方向）→ 本类 y；padog 的 y（抬脚，-110mm 基座）→ 本类 z。
//
// 与原版的行为差异（移植取舍，勿当 bug）：
//  - padog 的 "trot" 是左右侧对抬步态（左对 LF+LB 同时摆、右对 RF+RB 同时摆），
//    不是对角步态，这里忠实保留。
//  - 仅实现 Stand / Trot / Walk；Spin / Climb / TrotSpinMix 回落站立。
//  - omega 被忽略：padog 的差动转向来自左右摇杆 L/R 对摆幅的逐腿加权，
//    与 set_gait_param(vx, omega) 的速度语义不匹配，未移植。
//  - 时间基准：原版 t 由主循环按 speed 推进（trot 约 10x 实时），本类按 update(dt)
//    实时推进，dt 限幅 0.05s。
//  - 跳过 p_origin / gyro_cal_sta 校准：/imu/pitch 已是绝对角度，pitch 零位由调用方
//    负责（set_cg_feedback 收到的是相对原点的角度即可）。
//  - 指令速度为零时回到站立位并清零重心偏移（原版 L==0 && R==0 时 t=0，但 X_S 仍
//    滞留在旧目标上，此处收口更安全）。
class PyGait {
public:
    // 切换步态并软复位（时钟、重心状态清零），避免切换瞬间相位/重心跳变。
    void set_gait_type(GaitType type) {
        type_ = type;
        soft_reset();
    }

    // 设置步态指令参数：仅 vx 生效，omega 被忽略（同 gait.hpp 的 trot / walk）。
    void set_gait_param(double vx, double omega) {
        vx_    = vx;
        omega_ = omega;
    }

    // 注入 IMU 反馈：pitch 角（弧度，抬头为正，相对校准原点）与 pitch 角速度 (rad/s)。
    // enable 对应原版 key_stab（自稳开关）：walk 原版恒置 True，trot 随 key_stab。
    // 仅当 enable 且 |vx|>0 时 pitch / pitch_rate 参与 X_goal 计算（原版
    // foward_cg_stab 的 (|r1|+|r4|+|r2|+|r3|)!=0 条件）。
    void set_cg_feedback(double pitch_rad, double pitch_rate_rad_s, bool enable) {
        pitch_      = pitch_rad;
        pitch_rate_ = pitch_rate_rad_s;
        cg_enable_  = enable;
    }

    // 复位全部状态，下次 update() 起回到站立姿态（步态类型保持不变）。
    void reset() { soft_reset(); }

    // 统一对外接口：按当前步态类型推进 dt 秒，返回 4 条腿的足端坐标（腿序见类注释）。
    // dt 限幅 0.05s：防止调用方传入异常大 dt 导致相位快进（同 gait.hpp）。
    std::array<Eigen::Vector3d, 4> update(double dt) {
        if (dt <= 0.0) {
            return feet_;
        }
        dt = std::min(dt, 0.05);

        // 仅 Stand / Trot / Walk 有意义，其余枚举回落站立。
        if (type_ != GaitType::Trot && type_ != GaitType::Walk) {
            stand_update();
            return apply_cg_shift();
        }

        // 重心模型：X_goal（目标机体位置）→ X_S（一阶滞后实际值），须先于门控更新。
        cg_update(dt);

        const double stride = stride_cmd();
        if (std::abs(stride) < 1e-6) {
            // 指令速度为零：站立保持（原版 L==0 && R==0 时 t=0），并清零重心偏移。
            t_      = 0.0;
            X_goal_ = 0.0;
            X_S_    = 0.0;
            for (auto& f : gait_feet_) {
                f = Eigen::Vector3d(0.0, 0.0, kStanceZ);
            }
            return apply_cg_shift();
        }

        switch (type_) {
        case GaitType::Trot: trot_update(dt, stride); break;
        case GaitType::Walk: walk_update(dt, stride); break;
        default: break;
        }

        return apply_cg_shift();
    }

    // 摆动相基准抬脚高度 (m)，与 gait.hpp 同名常量保持一致。
    static constexpr double kDefaultStepHeight = 0.03;

    // 注入摆动轨迹参数：逐腿抬脚高度（m，作为摆动相抬脚峰值 zh）与步幅缩放系数
    // （0..1，乘在步幅指令上）。
    void set_swing_profile(const std::array<double, 4>& lift_heights, double stride_scale) {
        step_height_  = lift_heights;
        stride_scale_ = std::clamp(stride_scale, 0.0, 1.0);
    }

private:
    // 机体前移：原版 mainloop 中 cal_ges(x=X_S) 把所有足端沿前进方向平移 -X_S，
    // 实际重心随速度/坡度/角速度动态前移，是侧向稳定（trot）与爬行稳定（walk）的关键。
    // 注意必须对未平移的 gait_feet_ 做绝对平移（而非在 feet_ 上累加），否则每次
    // update 都重复叠加 X_S 的变化量。
    std::array<Eigen::Vector3d, 4> apply_cg_shift() {
        for (int i = 0; i < 4; ++i) {
            feet_[i] = gait_feet_[i];
            feet_[i].y() -= X_S_;
        }
        return feet_;
    }
    // ---- 步态时序常量（原版 Tf=0.5s）----
    static constexpr double kStanceZ       = 0.14;  // 站立时足端离髋竖直距离 (m)
    static constexpr double kTf            = 0.5;   // 摆动相时长 = 支撑相时长 (s)
    static constexpr double kTrotPeriod    = 1.0;   // trot 周期 (s)
    static constexpr double kWalkPhaseTime = 0.5;   // walk 每相时长 (s)
    static constexpr double kWalkCycle     = 2.5;   // walk 周期 = 5 相 (s)
    static constexpr double kMaxStride     = 0.032; // 单腿行程上限 (m)，原版最大 30mm

    // ---- 重心模型常量（原版单位 mm → m）----
    static constexpr double kCgInY     = 0.017; // in_y：静态前倾偏置 (m)
    static constexpr double kCgHeight  = 0.11;  // pit_cause_cg_adjust 的 h：重心高度 (m)
    static constexpr double kCgPitchKp = 1.1;   // tan 内的增益
    static constexpr double kCgGyroGain = 0.005; // gyro_x_fitted*5：pitch 角速度前馈 (m/(rad/s))
    static constexpr double kCgGate    = 0.001; // walk 相位门控阈值 (m)，原版 <1mm
    static constexpr double kCgTauTrot = 0.05;  // X_S 一阶滞后时间常数 (s)，trot 用
    static constexpr double kCgTauWalk = 0.1;   // X_S 一阶滞后时间常数 (s)，walk 用
    // walk 各相的重心偏置 gait_need：抬前腿时重心后移 -30mm，抬后腿时前移 +40mm。
    static constexpr std::array<double, 5> kWalkGaitNeed{-0.03, -0.03, 0.04, 0.04, -0.03};
    // walk 各相（P1~P4）的摆动腿（DogBot 索引）：padog 顺序 LF→RF→LB→RB。
    static constexpr std::array<int, 4> kWalkSwingLeg{0, 3, 1, 2};

    // 步幅指令：vx * Tf（半周期行程），受步幅缩放与限幅约束。
    double stride_cmd() const {
        return std::clamp(vx_ * kTf * stride_scale_, -kMaxStride, kMaxStride);
    }

    // 摆动相水平曲线（原版 swing_curve_generate，xv0=0, x0=0）：u=t/Tf，
    // 0~1/4 保持 0，1/4~3/4 三次曲线 0→xt，3/4~1 驻留 xt（提前落足防冲击）。
    static double swing_x(double u, double xt) {
        if (u < 0.25) {
            return 0.0;
        }
        if (u < 0.75) {
            return xt * (-16.0 * u * u * u + 24.0 * u * u - 9.0 * u + 1.0);
        }
        return xt;
    }

    // 摆动相抬脚曲线（原版 swing_curve_generate 的 Z Generator，z0=0）：
    // 前 1/2 三次立起至 1，后 1/2 二次回落至 0，两端斜率 0，峰值出现在 u=0.5。
    static double swing_lift(double u) {
        if (u < 0.5) {
            return 12.0 * u * u - 16.0 * u * u * u;
        }
        return 4.0 * u - 4.0 * u * u;
    }

    // 支撑相曲线（原版 support_curve_generate）：足端相对机体线性后退，
    // 从 xt 在 kTf 时长内回到 0（机体前进 xt），与摆动结束位置连续。
    static double support_x(double t_rel, double xt) { return xt - (xt / kTf) * t_rel; }

    // 重心模型：X_goal = in_y + gait_need + h*tan(Kp*pitch) + Kgyro*pitch_rate，
    // X_S 以时间常数 tau 一阶滞后逼近 X_goal。原版 X_goal 由 foward_cg_stab 在
    // enable 且运动时更新（否则保持旧值），X_S 在 mainloop 中无条件平滑，此处同。
    void cg_update(double dt) {
        const double tau = (type_ == GaitType::Walk) ? kCgTauWalk : kCgTauTrot;
        if (cg_enable_ && std::abs(vx_) > 1e-6) {
            X_goal_ = kCgInY + gait_need() + kCgHeight * std::tan(kCgPitchKp * pitch_)
                    + kCgGyroGain * pitch_rate_;
        }
        X_S_ += (1.0 - std::exp(-dt / tau)) * (X_goal_ - X_S_);
    }

    // 当前相的重心偏置：walk 按相位查表，trot 恒为 0（原版 foward_cg_stab 的 gait_need）。
    double gait_need() const {
        if (type_ == GaitType::Walk) {
            const int phase = std::min(static_cast<int>(t_ / kWalkPhaseTime), 4);
            return kWalkGaitNeed[phase];
        }
        return 0.0;
    }

    // trot：左右侧对抬（忠实移植 padog，非对角步态）。
    // P1（t<Tf）：左对 {LF,LB}={0,1} 摆动、右对 {RF,RB}={3,2} 支撑；
    // P2（Tf<=t<2Tf）：互换。t 到周期边界回绕（原版 t>=1 → t=0）。
    void trot_update(double dt, double stride) {
        t_ += dt;
        if (t_ >= kTrotPeriod) {
            t_ = 0.0;
        }
        const bool p1 = t_ < kTf;
        for (int i = 0; i < 4; ++i) {
            const bool swing = p1 ? (i <= 1) : (i >= 2);
            if (swing) {
                const double u = p1 ? t_ / kTf : (t_ - kTf) / kTf;
                gait_feet_[i]  = Eigen::Vector3d(
                    0.0, swing_x(u, stride), kStanceZ - step_height_[i] * swing_lift(u));
            } else {
                const double t_rel = p1 ? t_ : (t_ - kTf);
                gait_feet_[i]      = Eigen::Vector3d(0.0, support_x(t_rel, stride), kStanceZ);
            }
        }
    }

    // walk：5 相爬行（忠实移植 padog，原版 if t<0.5 / 0.5~1 / 1~1.5 / 1.5~2 / >=2 五窗口）：
    // P1~P4 依次抬 LF→RF→LB→RB（先抬的腿驻留 xt，其余支撑腿保持 0），
    // P5 全部足端同步线性扫回 0（机体前移 xt）。
    // 相位门控：|X_S - X_goal| < kCgGate 才推进时钟，否则冻结足端（原版
    // abs(padog.X_S-padog.X_goal)<1 不满足时不推进 t、足端保持上帧值）。
    void walk_update(double dt, double stride) {
        if (std::abs(X_S_ - X_goal_) >= kCgGate) {
            return;
        }
        t_ += dt;
        if (t_ >= kWalkCycle) {
            t_ -= kWalkCycle;
        }
        const int phase = static_cast<int>(t_ / kWalkPhaseTime);
        const double tp = t_ - phase * kWalkPhaseTime;

        if (phase >= 4) {
            // P5：机体前移相，全部足端线性扫回 0。
            for (int i = 0; i < 4; ++i) {
                gait_feet_[i] = Eigen::Vector3d(0.0, support_x(tp, stride), kStanceZ);
            }
            return;
        }

        // P1~P4：先按驻留布置（此前已摆完的腿停在 xt），再覆盖摆动腿。
        for (int i = 0; i < 4; ++i) {
            double y = 0.0;
            for (int j = 0; j < phase; ++j) {
                if (kWalkSwingLeg[j] == i) {
                    y = stride;
                }
            }
            gait_feet_[i] = Eigen::Vector3d(0.0, y, kStanceZ);
        }
        const int swing   = kWalkSwingLeg[phase];
        const double u    = tp / kTf;
        gait_feet_[swing] = Eigen::Vector3d(
            0.0, swing_x(u, stride), kStanceZ - step_height_[swing] * swing_lift(u));
    }

    // stand：保持站立位，重心偏移清零（原版站姿 X_S 由 gesture() 控制，此处收口）。
    void stand_update() {
        X_goal_ = 0.0;
        X_S_    = 0.0;
        for (auto& f : gait_feet_) {
            f = Eigen::Vector3d(0.0, 0.0, kStanceZ);
        }
    }

    void soft_reset() {
        t_          = 0.0;
        X_goal_     = 0.0;
        X_S_        = 0.0;
        pitch_      = 0.0;
        pitch_rate_ = 0.0;
        cg_enable_  = false;
        step_height_.fill(kDefaultStepHeight);
        stride_scale_ = 1.0;
        for (auto& f : gait_feet_) {
            f = Eigen::Vector3d(0.0, 0.0, kStanceZ);
        }
        for (auto& f : feet_) {
            f = Eigen::Vector3d(0.0, 0.0, kStanceZ);
        }
    }

    GaitType type_ = GaitType::Stand;
    double vx_     = 0.0;
    double omega_  = 0.0;     // 未使用（padog 差动未移植），保留占位

    double t_ = 0.0;

    double pitch_      = 0.0; // IMU pitch（弧度，相对校准原点）
    double pitch_rate_ = 0.0; // IMU pitch 角速度 (rad/s)
    bool cg_enable_    = false;
    double X_goal_     = 0.0; // 机体重心目标位置 (m)
    double X_S_        = 0.0; // 机体重心实际位置（一阶滞后）(m)

    // 逐腿摆动抬脚高度 (m)，默认 kDefaultStepHeight，由 set_swing_profile 注入
    std::array<double, 4> step_height_{
        kDefaultStepHeight, kDefaultStepHeight, kDefaultStepHeight, kDefaultStepHeight};
    double stride_scale_ = 1.0;                  // 步幅缩放（0..1）

    std::array<Eigen::Vector3d, 4> gait_feet_{}; // 纯步态曲线足端（未平移，供 apply_cg_shift）
    std::array<Eigen::Vector3d, 4> feet_{}; // 对外输出：gait_feet_ 平移 X_S 后
};

} // namespace dogbot_core::controller
