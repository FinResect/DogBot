#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <numbers>

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

    void configureClimb(const ClimbConfig& cfg) {
        climb_cfg_ = cfg;
        resetClimb();
    }

    void setTargetVel(double vy) { vx_ = vy; omega_z_ = 0.0; }
    void setTargetTwist(double vx, double omega_z) {
        vx_     = vx;
        omega_z_ = omega_z;
    }
    void setGaitType(GaitType type) {
        if (type != type_) {
            elapsed_ = 0.0;
            if (type == GaitType::Climb) {
                resetClimb();
            }
        }
        type_ = type;
    }

    void step(double dt) {
        if (type_ == GaitType::Climb) {
            climbStep(dt);
            return;
        }

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
    bool climbDone() const { return type_ == GaitType::Climb && climb_step_ >= climb_cfg_.steps; }

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
    void resetClimb() {
        climb_step_   = 0;
        climb_phase_  = 0;
        phase_elapsed_ = 0.0;
        climb_completed_.fill(false);
    }

    void climbStep(double dt) {
        if (climb_step_ >= climb_cfg_.steps) {
            setClimbHold();
            return;
        }

        phase_elapsed_ += dt;

        while (phase_elapsed_ >= climb_cfg_.phase_time) {
            phase_elapsed_ -= climb_cfg_.phase_time;
            advanceClimbPhase();
            if (climb_step_ >= climb_cfg_.steps) {
                setClimbHold();
                return;
            }
        }

        double p = phase_elapsed_ / climb_cfg_.phase_time;

        constexpr int kRF = 0, kLF = 1, kShift = 2, kRB = 3, kLB = 4, kSettle = 5;

        switch (climb_phase_) {
        case kRF:    climbLeg(3, p, false); break;  // right_front
        case kLF:    climbLeg(0, p, false); break;  // left_front
        case kShift: shiftBody(p); break;
        case kRB:    climbLeg(2, p, true);  break;  // right_back
        case kLB:    climbLeg(1, p, true);  break;  // left_back
        case kSettle: setSettlePose(); break;
        default: break;
        }
    }

    void advanceClimbPhase() {
        constexpr int kPhaseCount = 6;
        climb_phase_++;
        if (climb_phase_ >= kPhaseCount) {
            climb_phase_ = 0;
            climb_step_++;
            climb_completed_.fill(false);
        }
    }

    void climbLeg(int climber, double p, bool rear) {
        (void)rear;
        const double swingR = 0.5;
        double z_floor = legs_[0].z_stance - climb_step_ * climb_cfg_.step_height;
        double z_target = z_floor - climb_cfg_.step_height;
        double sp = p / swingR;
        if (sp > 1.0) sp = 1.0;

        for (int i = 0; i < kLegCount; ++i) {
            if (i == climber) {
                feet_[i].y() = legs_[i].base_y + climb_cfg_.reach * sp;
                double z_line = z_floor + (z_target - z_floor) * sp;
                feet_[i].z()  = z_line - climb_cfg_.step_height * std::sin(std::numbers::pi * sp);
                feet_[i].x()  = 0.0;
                if (p >= swingR) {
                    climb_completed_[i] = true;
                }
            } else if (climb_completed_[i]) {
                feet_[i].y() = legs_[i].base_y + climb_cfg_.reach;
                feet_[i].z() = z_target;
                feet_[i].x() = 0.0;
            } else {
                feet_[i].y() = legs_[i].base_y;
                feet_[i].z() = z_floor;
                feet_[i].x() = 0.0;
            }
        }

        feet_vel_.fill(Eigen::Vector3d::Zero());
    }

    void shiftBody(double p) {
        double z_floor  = legs_[0].z_stance - climb_step_ * climb_cfg_.step_height;
        double z_target = z_floor - climb_cfg_.step_height;
        double shift    = climb_cfg_.reach * 0.5 * p;

        for (int i = 0; i < kLegCount; ++i) {
            if (climb_completed_[i]) {
                feet_[i].y() = legs_[i].base_y + climb_cfg_.reach - shift;
                feet_[i].z() = z_target;
            } else {
                feet_[i].y() = legs_[i].base_y - shift;
                feet_[i].z() = z_floor;
            }
            feet_[i].x() = 0.0;
        }

        feet_vel_.fill(Eigen::Vector3d::Zero());
    }

    void setSettlePose() {
        double z_target = legs_[0].z_stance - climb_step_ * climb_cfg_.step_height;
        double shift    = climb_cfg_.reach * 0.5;

        for (int i = 0; i < kLegCount; ++i) {
            if (climb_completed_[i]) {
                feet_[i].y() = legs_[i].base_y + climb_cfg_.reach - shift;
                feet_[i].z() = z_target;
            } else {
                feet_[i].y() = legs_[i].base_y - shift;
                feet_[i].z() = z_target - climb_cfg_.step_height;
            }
            feet_[i].x() = 0.0;

            climb_completed_[i] = true;
        }

        feet_vel_.fill(Eigen::Vector3d::Zero());
    }

    void setClimbHold() {
        double z_done = legs_[0].z_stance - climb_cfg_.steps * climb_cfg_.step_height;
        for (auto& f : feet_) {
            f = Eigen::Vector3d(0.0, legs_[0].base_y, z_done);
        }
        feet_vel_.fill(Eigen::Vector3d::Zero());
    }

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

    ClimbConfig climb_cfg_{};
    int climb_step_ = 0;
    int climb_phase_ = 0;
    double phase_elapsed_ = 0.0;
    std::array<bool, kLegCount> climb_completed_{};
};

} // namespace dogbot_core::controller
