#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace dogbot_core::controller {

enum class GaitType { Stand, Trot, Spin, Climb, TrotSpinMix };

// 步态引擎：速度级增量积分。
//
// 坐标系与硬件约定（已在实机验证，勿改符号）：
//  - 足端坐标 (x, y, z)：x 恒为 0；y 为腿平面水平轴（摆动/支撑的行程方向）；z 为从髋
//    向下的竖直距离，抬脚时 z 减小（kStanceZ=0.13 为站立时足端离髋的竖直距离）。
//  - 腿序 {left_front, left_back, right_back, right_front}（索引 0..3），与 dogbot.cpp
//    舵机 ID 顺序一致（knee/hip ID: 0/1=LF, 2/3=LB, 4/5=RB, 6/7=RF）。
//  - kBodyY[i] 为各腿肩点相对机体中心的横向杠杆臂：左腿 (LF/LB) 为 +0.05，右腿
//    (RB/RF) 为 -0.05，符号决定 spin 的差动方向，请勿改动。
//  - 支撑相足端速度 v_i = vx_eff - omega_eff * kBodyY[i]：trot 只启用 vx，spin 只启用
//    omega，trot_spin_mix 在速度层线性叠加两者。单周期足端行程恒为指令速度的积分，
//    因此融合天然不会产生双倍行程（增量叠加）。
class Gait {
public:
    // 切换步态并软复位（时钟、足端积分器、爬楼状态清零），避免切换瞬间相位跳变。
    void setGaitType(GaitType type) {
        type_ = type;
        softReset();
    }

    // 设置步态指令参数，仅 trot / spin / trot_spin_mix 有意义：
    //  - trot：仅 vx 生效，omega 被忽略
    //  - spin：仅 omega 生效，vx 被忽略
    //  - trot_spin_mix：vx 与 omega 在支撑相速度层叠加
    // stand / climb 忽略两个参数。
    void setGaitParam(double vx, double omega) {
        vx_    = vx;
        omega_ = omega;
    }

    // 复位全部状态，下次 update() 起回到站立姿态（步态类型保持不变）。
    void reset() { softReset(); }

    // 统一对外接口：按当前步态类型推进 dt 秒，返回 4 条腿的足端坐标（腿序见类注释）。
    // dt 限幅 0.05s：防止调用方传入异常大 dt 导致相位快进（单次最多 1/12 周期），
    // 也限制 Climb 一次调用最多推进一个阶段（0.05 < 阶段时长 0.8s）。
    std::array<Eigen::Vector3d, 4> update(double dt) {
        if (dt <= 0.0) {
            return feet_;
        }
        dt = std::min(dt, 0.05);
        if (type_ == GaitType::Climb) {
            climbUpdate(dt);
        } else {
            cyclicUpdate(dt);
        }
        return feet_;
    }

private:
    static constexpr double kStanceZ    = 0.08; // 站立时足端离髋竖直距离 (m)
    static constexpr double kStepHeight = 0.04; // 摆动相抬脚高度 (m)
    static constexpr double kPeriod     = 0.6;  // 步态周期 (s)
    // 足端最大速度 (m/s)：由舵机最大转速 187.5°/s (0.32s/60°) 反推——
    // 髋到足距离取保守值 r=0.114，留 ~14% 余量。支撑相足端速度 = 指令速度，
    // 因此指令在速度层钳制；摆动相目标幅值另行由 kMaxStride 限制。
    static constexpr double kMaxFootSpeed = 0.32;
    // 单腿行程上限 (m)：摆动相足端峰值速度 = pi*stride/T_swing，由 kMaxFootSpeed
    // 推导得 stride<=0.032。摆动目标与支撑相积分共用该限幅，避免两者不对称导致
    // 摆动幅度放大、速度再次超限。
    static constexpr double kMaxStride  = 0.032;
    static constexpr double kSwingRatio = 0.5; // 摆动相占周期比例
    static constexpr double kStrideTime = kPeriod * 0.25;

    // 对角配对：LF+RB 同相、LB+RF 同相（索引见类注释）。
    static constexpr std::array<double, 4> kTrotPhase{0.0, 0.5, 0.0, 0.5};
    // 各腿肩点横向杠杆臂：左腿 +0.05，右腿 -0.05。
    static constexpr std::array<double, 4> kBodyY{0.05, 0.05, -0.05, -0.05};

    static constexpr double kClimbStepHeight = 0.03; // 每级台阶升高 (m)
    static constexpr double kClimbReach      = 0.04; // 上台阶时落足前伸距离 (m)
    static constexpr double kClimbPhaseTime  = 0.8;  // 每个爬楼阶段时长 (s)
    static constexpr double kClimbSwingRatio = 0.5;  // 摆动占阶段前半段，后半段驻留
    static constexpr int kClimbSteps         = 1;    // 爬升台阶数
    static constexpr int kClimbPhaseCount    = 6;    // RF→LF→Shift→RB→LB→Settle
    static constexpr int kShiftPhase         = 2;    // shift 阶段在 climb_phase_ 中的序号
    static constexpr int kSettlePhase        = 5;    // settle 阶段在 climb_phase_ 中的序号

    // stand/trot/spin/trot_spin_mix 共用：速度级增量积分。
    std::array<Eigen::Vector3d, 4> cyclicUpdate(double dt) {
        if (type_ == GaitType::Stand) {
            for (int i = 0; i < 4; ++i) {
                y_[i]    = 0.0;
                feet_[i] = Eigen::Vector3d(0.0, 0.0, kStanceZ);
            }
            return feet_;
        }

        t_ += dt;

        const bool use_vx    = type_ != GaitType::Spin;
        const bool use_omega = type_ != GaitType::Trot;

        for (int i = 0; i < 4; ++i) {
            // 支撑相足端速度 = trot 项 + spin 项（速度层叠加，v_i*T/4 即单行程），
            // 速度级钳制到 kMaxFootSpeed，防止支撑相滑动速度超出舵机跟踪能力。
            const double v_i = std::clamp(
                (use_vx ? vx_ : 0.0) - (use_omega ? omega_ : 0.0) * kBodyY[i], -kMaxFootSpeed,
                kMaxFootSpeed);
            const double stride = std::clamp(v_i * kStrideTime, -kMaxStride, kMaxStride);

            double s = std::fmod(t_ / kPeriod + kTrotPhase[i], 1.0);
            if (s < 0.0) {
                s += 1.0;
            }

            if (std::abs(stride) < 1e-6) {
                // 指令速度为零：足端保持站立位，不做无意义抬脚。
                y_[i]             = 0.0;
                swing_entered_[i] = false;
                feet_[i]          = Eigen::Vector3d(0.0, 0.0, kStanceZ);
            } else if (s < kSwingRatio) {
                // 摆动相：从摆动起始偏移平滑摆向目标落点 stride（目标逐帧重规划，
                // 速度变化不产生足端跳变），同时抬脚。
                if (!swing_entered_[i]) {
                    y_start_[i]       = y_[i];
                    swing_entered_[i] = true;
                }
                const double u       = s / kSwingRatio;
                const double profile = 0.5 * (1.0 - std::cos(std::numbers::pi * u));
                // 双余弦抬脚弧：两端高度与斜率均为 0，摆动/支撑切换处无跳变
                // （单余弦 arc 在离散相位结束时残留 ~0.6mm 台阶）。
                const double lift = 0.5 * (1.0 - std::cos(2.0 * std::numbers::pi * u));
                y_[i]             = y_start_[i] + (stride - y_start_[i]) * profile;
                feet_[i]          = Eigen::Vector3d(0.0, y_[i], kStanceZ + kStepHeight * lift);
            } else {
                // 支撑相：足端相对机体按指令速度积分（后退），抬脚高度回落到站立位。
                swing_entered_[i] = false;
                y_[i]             = std::clamp(y_[i] - v_i * dt, -kMaxStride, kMaxStride);
                feet_[i]          = Eigen::Vector3d(0.0, y_[i], kStanceZ);
            }
        }
        return feet_;
    }

    // 爬楼梯：6 阶段状态机（RF→LF→Shift→RB→LB→Settle），每阶段前半段抬单腿到升高
    // 后的落点，其余腿驻留在各自落点；完成全部台阶后保持最终姿态，不自动回落。
    void climbUpdate(double dt) {
        if (climb_done_) {
            return;
        }

        phase_elapsed_ += dt;
        while (phase_elapsed_ >= kClimbPhaseTime) {
            phase_elapsed_ -= kClimbPhaseTime;
            if (++climb_phase_ >= kClimbPhaseCount) {
                climb_phase_ = 0;
                if (++climb_step_ >= kClimbSteps) {
                    climb_done_ = true;
                    holdClimbPose();
                    return;
                }
            }
            if (climb_phase_ == kShiftPhase || climb_phase_ == kSettlePhase) {
                // 进入 shift/settle 阶段时记录各腿起点，阶段内以绝对位置计算而非累加，
                // 避免足端随迭代逐帧指数衰减。
                for (int i = 0; i < 4; ++i) {
                    phase_base_y_[i] = foothold_[i].y();
                    phase_base_z_[i] = foothold_[i].z();
                }
            }
        }

        const double p = phase_elapsed_ / kClimbPhaseTime;
        switch (climb_phase_) {
        case 0: climbLeg(3, p); break; // right_front
        case 1: climbLeg(0, p); break; // left_front
        case 2: shiftBody(p); break;
        case 3: climbLeg(2, p); break; // right_back
        case 4: climbLeg(1, p); break; // left_back
        case 5: settle(p); break;
        default: break;
        }
    }

    // 当前台阶的地面高度与上一级台阶（目标）高度，z 向下为正。
    double zFloor() const { return kStanceZ - kClimbStepHeight * climb_step_; }
    double zNext() const { return zFloor() - kClimbStepHeight; }

    void climbLeg(int climber, double p) {
        const double sp = std::min(p / kClimbSwingRatio, 1.0);

        for (int i = 0; i < 4; ++i) {
            if (i == climber) {
                const double y_from = foothold_[i].y();
                const double z_from = foothold_[i].z();
                // 沿弧线从当前落点摆到 (kClimbReach, zNext)，抬脚弧高 kClimbStepHeight。
                const double y = y_from + (kClimbReach - y_from) * sp;
                const double z = z_from + (zNext() - z_from) * sp
                               - kClimbStepHeight * std::sin(std::numbers::pi * sp);
                feet_[i] = Eigen::Vector3d(0.0, y, z);
                if (p >= kClimbSwingRatio) {
                    foothold_[i] = Eigen::Vector3d(0.0, kClimbReach, zNext());
                }
            } else {
                feet_[i] = Eigen::Vector3d(0.0, foothold_[i].y(), foothold_[i].z());
            }
        }
    }

    // 机体前移：全部落点相对进入 shift 时的起点统一后移 reach/2（绝对位置，不累加）。
    void shiftBody(double p) {
        for (int i = 0; i < 4; ++i) {
            const double y   = phase_base_y_[i] - kClimbReach * 0.5 * p;
            foothold_[i].y() = y;
            feet_[i]         = Eigen::Vector3d(0.0, y, foothold_[i].z());
        }
    }

    // 收尾：各腿从进入 settle 时的落点平滑回中到 (0, zNext)，即台阶完成后的保持姿态。
    void settle(double p) {
        for (int i = 0; i < 4; ++i) {
            const double y = phase_base_y_[i] + (0.0 - phase_base_y_[i]) * p;
            const double z = phase_base_z_[i] + (zNext() - phase_base_z_[i]) * p;
            foothold_[i]   = Eigen::Vector3d(0.0, y, z);
            feet_[i]       = Eigen::Vector3d(0.0, y, z);
        }
    }

    void holdClimbPose() {
        for (auto& f : feet_) {
            f = Eigen::Vector3d(0.0, 0.0, zFloor());
        }
    }

    void softReset() {
        t_ = 0.0;
        y_.fill(0.0);
        y_start_.fill(0.0);
        swing_entered_.fill(false);
        climb_step_    = 0;
        climb_phase_   = 0;
        phase_elapsed_ = 0.0;
        climb_done_    = false;
        foothold_.fill(Eigen::Vector3d(0.0, 0.0, kStanceZ));
        for (auto& f : feet_) {
            f = Eigen::Vector3d(0.0, 0.0, kStanceZ);
        }
    }

    GaitType type_ = GaitType::Stand;
    double vx_     = 0.0;
    double omega_  = 0.0;

    double t_ = 0.0;

    std::array<double, 4> y_{};                 // 足端沿 y 轴相对站立位的偏移
    std::array<double, 4> y_start_{};           // 摆动相起始偏移
    std::array<bool, 4> swing_entered_{};

    int climb_step_       = 0;                  // 已完成台阶数
    int climb_phase_      = 0;                  // 当前阶段 0..5
    double phase_elapsed_ = 0.0;
    bool climb_done_      = false;
    std::array<double, 4> phase_base_y_{};      // 进入 shift/settle 阶段时各腿落点 y
    std::array<double, 4> phase_base_z_{};      // 进入 shift/settle 阶段时各腿落点 z
    std::array<Eigen::Vector3d, 4> foothold_{}; // 各腿当前落点 (y, z)

    std::array<Eigen::Vector3d, 4> feet_{};
};

} // namespace dogbot_core::controller
