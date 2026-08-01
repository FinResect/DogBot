#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace dogbot_core::controller {

enum class GaitType { Stand, Trot, Amble, Walk, Climb, Spin };

class Gait {
public:
    void setParams(
        double stance_y, double stance_z, double step_length, double step_height, double period) {
        stance_y_    = stance_y;
        stance_z_    = stance_z;
        step_length_ = step_length;
        step_height_ = step_height;
        period_      = std::max(period, 1e-3);
    }
    
    void setClimbParams(double step_height, double reach, double phase_time, int steps) {
        climb_step_height_ = step_height;
        climb_reach_       = reach;
        climb_phase_time_  = std::max(phase_time, 1e-3);
        climb_steps_       = std::max(steps, 1);
    }

    void reset() {
        t_             = 0.0;
        climb_step_    = 0;
        climb_phase_   = 0;
        phase_elapsed_ = 0.0;
        climb_completed_.fill(false);
    }

    bool climbDone() const { return climb_step_ >= climb_steps_; }

    std::array<Eigen::Vector3d, 4> step(
        GaitType mode, double dt, double vx = 0.0, double omega_z = 0.0) {
        t_ += dt;
        switch (mode) {
        case GaitType::Stand: return standStep();
        case GaitType::Trot: return cyclicStep(kTrotPhase, vx, omega_z);
        case GaitType::Amble: return cyclicStep(kAmblePhase, vx, omega_z);
        case GaitType::Walk: return cyclicStep(kWalkPhase, vx, omega_z);
        case GaitType::Climb:
            climbStep(dt);
            return feet_;
        case GaitType::Spin:
            return standStep();
        }
        return standStep();
    }

private:
    static constexpr std::array<double, 4> kTrotPhase{0.0, 0.5, 0.0, 0.5};
    static constexpr std::array<double, 4> kAmblePhase{0.0, 0.3, 0.15, 0.45};
    static constexpr std::array<double, 4> kWalkPhase{0.0, 0.5, 0.25, 0.75};

    static constexpr std::array<double, 4> kLegBodyY{0.05, 0.05, -0.05, -0.05};

    std::array<Eigen::Vector3d, 4> standStep() {
        for (auto& f : feet_) {
            f = Eigen::Vector3d(0.0, stance_y_, stance_z_);
        }
        return feet_;
    }

    std::array<Eigen::Vector3d, 4> cyclicStep(
        const std::array<double, 4>& phase_offsets, double vx, double omega_z) {
        for (int i = 0; i < 4; ++i) {
            feet_[i] = legTrajectory(phase_offsets[i], effectiveStride(i, vx, omega_z));
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

    double effectiveStride(int leg, double vx, double omega_z) const {
        if (std::abs(vx) > 1e-6 || std::abs(omega_z) > 1e-6) {
            return (vx - omega_z * kLegBodyY[leg]) * period_ / 4.0;
        }
        return step_length_;
    }

    void climbStep(double dt) {
        if (climb_step_ >= climb_steps_) {
            setClimbHold();
            return;
        }

        phase_elapsed_ += dt;

        while (phase_elapsed_ >= climb_phase_time_) {
            phase_elapsed_ -= climb_phase_time_;
            climb_phase_++;
            if (climb_phase_ >= kClimbPhaseCount) {
                climb_phase_ = 0;
                climb_step_++;
                climb_completed_.fill(false);
            }
            if (climb_step_ >= climb_steps_) {
                setClimbHold();
                return;
            }
        }

        const double p = phase_elapsed_ / climb_phase_time_;

        switch (climb_phase_) {
        case 0: climbLeg(3, p); break; // right_front
        case 1: climbLeg(0, p); break; // left_front
        case 2: shiftBody(p); break;
        case 3: climbLeg(2, p); break; // right_back
        case 4: climbLeg(1, p); break; // left_back
        case 5: setSettlePose(); break;
        default: break;
        }
    }

    void climbLeg(int climber, double p) {
        static constexpr double kSwingRatio = 0.5;
        const double z_floor  = stance_z_ - climb_step_ * climb_step_height_;
        const double z_target = z_floor - climb_step_height_;
        const double sp       = std::min(p / kSwingRatio, 1.0);

        for (int i = 0; i < 4; ++i) {
            if (i == climber) {
                const double y      = stance_y_ + climb_reach_ * sp;
                const double z_line = z_floor + (z_target - z_floor) * sp;
                const double z      = z_line - climb_step_height_ * std::sin(std::numbers::pi * sp);
                feet_[i]            = Eigen::Vector3d(0.0, y, z);
                if (p >= kSwingRatio) {
                    climb_completed_[i] = true;
                }
            } else if (climb_completed_[i]) {
                feet_[i] = Eigen::Vector3d(0.0, stance_y_ + climb_reach_, z_target);
            } else {
                feet_[i] = Eigen::Vector3d(0.0, stance_y_, z_floor);
            }
        }
    }

    void shiftBody(double p) {
        const double z_floor  = stance_z_ - climb_step_ * climb_step_height_;
        const double z_target = z_floor - climb_step_height_;
        const double shift    = climb_reach_ * 0.5 * p;

        for (int i = 0; i < 4; ++i) {
            if (climb_completed_[i]) {
                feet_[i] = Eigen::Vector3d(0.0, stance_y_ + climb_reach_ - shift, z_target);
            } else {
                feet_[i] = Eigen::Vector3d(0.0, stance_y_ - shift, z_floor);
            }
        }
    }

    void setSettlePose() {
        const double z_target = stance_z_ - climb_step_ * climb_step_height_;
        const double shift    = climb_reach_ * 0.5;

        for (int i = 0; i < 4; ++i) {
            if (climb_completed_[i]) {
                feet_[i] = Eigen::Vector3d(0.0, stance_y_ + climb_reach_ - shift, z_target);
            } else {
                feet_[i] = Eigen::Vector3d(0.0, stance_y_ - shift, z_target - climb_step_height_);
            }
            climb_completed_[i] = true;
        }
    }

    void setClimbHold() {
        const double z_done = stance_z_ - climb_steps_ * climb_step_height_;
        for (auto& f : feet_) {
            f = Eigen::Vector3d(0.0, stance_y_, z_done);
        }
    }

    static constexpr int kClimbPhaseCount = 6;

    double t_ = 0.0;

    double stance_y_    = 0.0;
    double stance_z_    = 0.13;
    double step_length_ = 0.04;
    double step_height_ = 0.03;
    double period_      = 0.6;

    double climb_step_height_ = 0.03;
    double climb_reach_       = 0.04;
    double climb_phase_time_  = 0.8;
    int climb_steps_          = 1;

    int climb_step_    = 0;
    int climb_phase_   = 0;
    double phase_elapsed_ = 0.0;
    std::array<bool, 4> climb_completed_{};

    std::array<Eigen::Vector3d, 4> feet_{};
};

} // namespace dogbot_core::controller
