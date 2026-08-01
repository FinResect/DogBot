#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace dogbot_core::controller {

class SpinGait {
public:
    void setParams(
        double stance_y, double stance_z, double spin_step, double step_height, double period) {
        stance_y_    = stance_y;
        stance_z_    = stance_z;
        spin_step_   = spin_step;
        step_height_ = step_height;
        period_      = std::max(period, 1e-3);
    }

    void reset() {
        t_           = 0.0;
        target_angle_ = 0.0;
        accumulated_ = 0.0;
        done_        = true;
    }

    void setTargetAngle(double rad) {
        target_angle_ = rad;
        accumulated_  = 0.0;
        done_         = std::abs(rad) < 1e-6;
    }

    double rotationProgress() const { return accumulated_; }

    bool isDone() const { return done_; }

    std::array<Eigen::Vector3d, 4> step(double dt, double omega_z = 0.0) {
        t_ += dt;

        const bool has_velocity = std::abs(omega_z) > 1e-6;
        const bool has_target   = std::abs(target_angle_) > 1e-6;

        if (done_ && !has_velocity && has_target) {
            return standPose();
        }

        double d = 1.0;          // 旋转方向: +1 逆时针, -1 顺时针
        double s = spin_step_;   // 单足步幅

        if (has_velocity) {
            d = omega_z > 0.0 ? 1.0 : -1.0;
            s = std::abs(omega_z) * kLateralOffset * period_ / 4.0;
        } else if (has_target) {
            d = target_angle_ > 0.0 ? 1.0 : -1.0;
        }

        const double omega_body = 4.0 * s / (period_ * kLateralOffset);
        accumulated_ += d * omega_body * dt;

        if (has_target && std::abs(accumulated_) >= std::abs(target_angle_)) {
            done_ = true;
            return standPose();
        }

        for (int i = 0; i < 4; ++i) {
            const double stride = -d * s * std::copysign(1.0, kLegBodyY[i]);
            feet_[i]            = legTrajectory(kTrotPhase[i], stride);
        }
        return feet_;
    }

private:
    static constexpr std::array<double, 4> kTrotPhase{0.0, 0.5, 0.0, 0.5};

    static constexpr std::array<double, 4> kLegBodyY{0.05, 0.05, -0.05, -0.05};

    // 所有腿的侧向偏移量 |py| 相同（与 gait.hpp kLegBodyY 一致）
    static constexpr double kLateralOffset = 0.05;

    std::array<Eigen::Vector3d, 4> standPose() {
        for (auto& f : feet_) {
            f = Eigen::Vector3d(0.0, stance_y_, stance_z_);
        }
        return feet_;
    }

    Eigen::Vector3d legTrajectory(double phase_offset, double stride) const {
        double s = std::fmod(t_ / period_ + phase_offset, 1.0);
        if (s < 0.0) {
            s += 1.0;
        }

        if (s < 0.5) {
            const double u = 2.0 * s;
            const double y = stance_y_ + stride * (2.0 * u - 1.0);
            const double z = stance_z_ + step_height_ * std::sin(std::numbers::pi * u);
            return Eigen::Vector3d(0.0, y, z);
        }

        const double v = 2.0 * (s - 0.5);
        const double y = stance_y_ + stride * (1.0 - 2.0 * v);
        return Eigen::Vector3d(0.0, y, stance_z_);
    }

    double t_ = 0.0;

    double stance_y_    = 0.0;
    double stance_z_    = 0.13;
    double spin_step_   = 0.02;
    double step_height_ = 0.03;
    double period_      = 0.6;

    double target_angle_ = 0.0;
    double accumulated_  = 0.0;
    bool done_           = true;

    std::array<Eigen::Vector3d, 4> feet_{};
};

} // namespace dogbot_core::controller
