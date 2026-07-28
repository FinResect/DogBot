#pragma once

#include <Eigen/Dense>
#include <cmath>

namespace dogbot_core::controller {

inline Eigen::Vector3d stepTrajectory(
    double phase,
    double swing_ratio,
    double base_y,
    double stride,
    double z_stance,
    double z_clearance)
{
    double AEP = base_y + stride / 2.0;
    double PEP = base_y - stride / 2.0;

    if (phase < swing_ratio) {
        double t = phase / swing_ratio;
        double y = PEP + (AEP - PEP) * t;
        double z = z_stance - z_clearance * std::sin(M_PI * t);
        return {0.0, y, z};
    }

    double t = (phase - swing_ratio) / (1.0 - swing_ratio);
    double y = AEP + (PEP - AEP) * t;
    return {0.0, y, z_stance};
}

inline Eigen::Vector3d stepTrajectoryVelocity(
    double phase,
    double swing_ratio,
    double stride,
    double z_clearance,
    double cycle_time)
{
    if (phase < swing_ratio) {
        double t = phase / swing_ratio;
        double vy = stride / (swing_ratio * cycle_time);
        double vz = -z_clearance * M_PI * std::cos(M_PI * t) / (swing_ratio * cycle_time);
        return {0.0, vy, vz};
    }

    double vy = -stride / ((1.0 - swing_ratio) * cycle_time);
    return {0.0, vy, 0.0};
}

} // namespace dogbot_core::controller
