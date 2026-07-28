#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>

#include "controller/gait_config.hpp"
#include "controller/trajectory.hpp"

namespace dogbot_core::controller {

class GaitEngine {
public:
    GaitEngine() = default;

    void configure(const GaitParams& params, const std::array<LegState, kLegCount>& legs) {
        params_      = params;
        legs_        = legs;
        T_cycle_     = params.overlap_time + params.swing_time + params.clearance_time;
        if (T_cycle_ <= 0.0) {
            T_cycle_ = 0.4;
        }
        swing_ratio_ = params.swing_time / T_cycle_;
        elapsed_     = 0.0;
    }

    void setTargetVel(double vy) { vx_ = vy; omega_z_ = 0.0; }
    void setTargetTwist(double vx, double omega_z) {
        vx_     = vx;
        omega_z_ = omega_z;
    }
    void setGaitType(GaitType type) { type_ = type; }

    void step(double dt) {
        elapsed_ += dt;

        for (int i = 0; i < kLegCount; ++i) {
            double offset  = legPhaseOffset(i, type_) * T_cycle_;
            double raw     = std::fmod(elapsed_ + offset, T_cycle_);
            if (raw < 0.0) {
                raw += T_cycle_;
            }
            phases_[i]      = raw / T_cycle_;
            double stride_i = strideForLeg(i);

            feet_[i] = stepTrajectory(
                phases_[i], swing_ratio_, legs_[i].base_y, stride_i,
                legs_[i].z_stance, params_.z_clearance);

            feet_vel_[i] = stepTrajectoryVelocity(
                phases_[i], swing_ratio_, stride_i,
                params_.z_clearance, T_cycle_);
        }
    }

    const std::array<Eigen::Vector3d, kLegCount>& feet() const { return feet_; }
    const std::array<Eigen::Vector3d, kLegCount>& feetVelocities() const { return feet_vel_; }
    double cycleTime() const { return T_cycle_; }
    GaitType gaitType() const { return type_; }

    static double legPhaseOffset(int leg_index, GaitType type) {
        switch (type) {
        case GaitType::Trot: {
            constexpr double offsets[] = {0.0, 0.5, 0.0, 0.5};
            return offsets[leg_index];
        }
        case GaitType::Amble: {
            constexpr double offsets[] = {0.0, 0.3, 0.15, 0.45};
            return offsets[leg_index];
        }
        case GaitType::Walk: {
            constexpr double offsets[] = {0.0, 0.5, 0.25, 0.75};
            return offsets[leg_index];
        }
        default:
            return 0.0;
        }
    }

    static constexpr const char* legName(int index) { return kLegNames[index]; }

private:
    double strideForLeg(int i) const {
        return (vx_ - omega_z_ * kLegBodyPositions[i].py) * T_cycle_;
    }

    GaitParams params_{};
    GaitType type_ = GaitType::Stand;
    std::array<LegState, kLegCount> legs_{};

    double T_cycle_     = 0.4;
    double swing_ratio_ = 0.375;
    double elapsed_     = 0.0;
    double vx_          = 0.0;
    double omega_z_     = 0.0;

    std::array<double, kLegCount> phases_{};
    std::array<Eigen::Vector3d, kLegCount> feet_{};
    std::array<Eigen::Vector3d, kLegCount> feet_vel_{};
};

} // namespace dogbot_core::controller
