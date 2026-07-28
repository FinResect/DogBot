#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>

namespace dogbot_core::controller {

struct JointAngles {
    double hip;
    double knee;
};

struct JointVelocities {
    double hip;
    double knee;
};

class LegSolver {
public:
    LegSolver(
        double thigh, double calf, double hip_offs, double L3, double L4, double L5, double r,
        double delta, bool fork_branch = true)
        : L1_(thigh)
        , L2_(calf)
        , hip_offset_(hip_offs)
        , a_(r)
        , b_(L5)
        , c_(L4)
        , d_(L3)
        , delta_(delta)
        , fork_branch_(fork_branch) {}

    JointAngles solve(const Eigen::Vector3d& foot) const {
        double r = std::sqrt(foot.y() * foot.y() + foot.z() * foot.z());

        double r_min = std::abs(L1_ - L2_);
        double r_max = L1_ + L2_;
        r            = std::clamp(r, r_min, r_max);

        double cos_knee        = (r * r - L1_ * L1_ - L2_ * L2_) / (2.0 * L1_ * L2_);
        cos_knee               = std::clamp(cos_knee, -1.0, 1.0);
        double theta_knee_geom = std::acos(cos_knee);

        double alpha = std::atan2(foot.z(), foot.y());
        double beta =
            std::atan2(L2_ * std::sin(theta_knee_geom), L1_ + L2_ * std::cos(theta_knee_geom));
        double theta_hip = alpha - beta + hip_offset_;

        double theta_knee_servo = servoAngleFromKnee(theta_knee_geom);

        return {theta_hip, theta_knee_servo};
    }

    JointVelocities solveVelocity(const Eigen::Vector3d& foot, const Eigen::Vector3d& foot_vel) const {
        double r = std::sqrt(foot.y() * foot.y() + foot.z() * foot.z());
        double r_min = std::abs(L1_ - L2_);
        double r_max = L1_ + L2_;
        r            = std::clamp(r, r_min, r_max);

        double cos_knee = (r * r - L1_ * L1_ - L2_ * L2_) / (2.0 * L1_ * L2_);
        cos_knee        = std::clamp(cos_knee, -1.0, 1.0);
        double th_k     = std::acos(cos_knee);

        double alpha = std::atan2(foot.z(), foot.y());
        double beta =
            std::atan2(L2_ * std::sin(th_k), L1_ + L2_ * std::cos(th_k));
        double th_h = alpha - beta;

        double vy = foot_vel.y();
        double vz = foot_vel.z();

        double s1  = std::sin(th_h);
        double c1  = std::cos(th_h);
        double s12 = std::sin(th_h + th_k);
        double c12 = std::cos(th_h + th_k);

        double detJ = L1_ * L2_ * std::sin(th_k);
        constexpr double kEp = 1e-8;
        if (std::abs(detJ) < kEp) {
            return {0.0, 0.0};
        }

        double wh = (c12 * vy + s12 * vz) * L2_ / detJ;
        double wk = (-(L1_ * c1 + L2_ * c12) * vy - (L1_ * s1 + L2_ * s12) * vz) / detJ;

        double ratio = dServo_dKnee(th_k);
        return {wh, wk * ratio};
    }

private:
    double servoAngleFromKnee(double knee_geom) const {
        double psi = knee_geom + delta_;

        double A = K2() + std::cos(psi);
        double B = std::sin(psi);
        double C = K1() * std::cos(psi) + K3();

        double norm  = std::sqrt(A * A + B * B);
        double ratio = std::clamp(C / norm, -1.0, 1.0);

        double phi;
        if (fork_branch_) {
            phi = std::atan2(B, A) + std::acos(ratio);
        } else {
            phi = std::atan2(B, A) - std::acos(ratio);
        }
        return phi;
    }

    double K1() const { return d_ / c_; }
    double K2() const { return d_ / a_; }
    double K3() const { return (a_ * a_ - b_ * b_ + c_ * c_ + d_ * d_) / (2.0 * a_ * c_); }

    double dServo_dKnee(double knee_geom) const {
        double psi = knee_geom + delta_;
        double phi = servoAngleFromKnee(knee_geom);

        double num_y = b_ * std::sin(psi) - a_ * std::sin(phi);
        double num_x = d_ + b_ * std::cos(psi) - a_ * std::cos(phi);
        double th3   = std::atan2(num_y, num_x);

        double sin_3_phi = std::sin(th3 - phi);
        if (std::abs(sin_3_phi) < 1e-12) {
            return 0.0;
        }

        return (b_ / a_) * std::sin(th3 - psi) / sin_3_phi;
    }

    double L1_, L2_, hip_offset_;
    double a_, b_, c_, d_, delta_;
    bool fork_branch_;
};

} // namespace dogbot_core::controller
