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
        double stance_y, double stance_z, double step_height, double period,
        double spin_gain = 1.0) {
        stance_y_    = stance_y;
        stance_z_    = stance_z;
        step_height_ = step_height;
        period_      = std::max(period, 1e-3);
        spin_gain_   = std::max(spin_gain, 0.0);
    }

    void reset() { t_ = 0.0; }

    std::array<Eigen::Vector3d, 4> step(double dt, double omega_z = 0.0) {
        t_ += dt;

        if (std::abs(omega_z) < 1e-6) {
            return standPose();
        }

        const double d = omega_z > 0.0 ? 1.0 : -1.0;   // 旋转方向: +1 逆时针, -1 顺时针
        const double s = spin_gain_ * std::abs(omega_z) * kLateralOffset * period_ / 4.0;

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
    double step_height_ = 0.03;
    double period_      = 0.6;
    double spin_gain_   = 1.0;

    std::array<Eigen::Vector3d, 4> feet_{};
};

} // namespace dogbot_core::controller
