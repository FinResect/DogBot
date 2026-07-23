#pragma once

#include <array>
#include <string>

namespace dogbot_core::controller {

enum class GaitType { Stand, Trot, Amble, Walk };

struct GaitParams {
    double overlap_time;
    double swing_time;
    double clearance_time;
    double z_clearance;
};

struct LegState {
    double base_y;
    double z_stance;
};

inline GaitParams makeTrotParams(double period = 0.8) {
    return {period / 4.0, period / 4.0, 0.0, 0.05};
}

inline GaitParams makeAmbleParams(double period = 1.0) {
    return {period / 5.0, period / 5.0, period / 10.0, 0.04};
}

inline GaitParams makeWalkParams(double period = 1.2) {
    return {period / 6.0, period / 6.0, period / 6.0, 0.04};
}

inline GaitParams makeFastConfig() {
    return {0.10, 0.15, 0.0, 0.05};
}

inline GaitParams makeSlowConfig() {
    return {0.40, 0.30, 0.26, 0.04};
}

inline const char* gaitName(GaitType type) {
    switch (type) {
    case GaitType::Stand: return "Stand";
    case GaitType::Trot:  return "Trot";
    case GaitType::Amble: return "Amble";
    case GaitType::Walk:  return "Walk";
    }
    return "Unknown";
}

inline constexpr int kLegCount     = 4;
inline constexpr int kServosPerLeg = 2;

inline const std::array<const char*, kLegCount> kLegNames{
    "left_front", "left_back", "right_back", "right_front"};
inline const std::array<const char*, kLegCount> kJointSuffixes{
    "hip", "knee"};

} // namespace dogbot_core::controller
