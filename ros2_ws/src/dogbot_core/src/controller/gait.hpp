#pragma once

#include <Eigen/Dense>

#include <array>
#include <cmath>
#include <numbers>

namespace dogbot_core::controller {

enum class GaitMode { Stand, Trot, Amble, Walk };

struct GaitConfig {
    double stance_y    = 0.0;
    double stance_z    = -0.08;
    double step_length = 0.02;
    double step_height = 0.012;
    double period      = 1.0;
};

class Gait {
public:
    explicit Gait(const GaitConfig& cfg = {}) : cfg_(cfg) {}

    void setConfig(const GaitConfig& cfg) { cfg_ = cfg; }

    std::array<Eigen::Vector3d, 4> feet(GaitMode mode, double t) const {
        if (mode == GaitMode::Stand) {
            std::array<Eigen::Vector3d, 4> f;
            f.fill(Eigen::Vector3d(0.0, cfg_.stance_y, cfg_.stance_z));
            return f;
        }

        const double* offsets = nullptr;
        switch (mode) {
        case GaitMode::Trot: {
            static constexpr double o[] = {0.0, 0.5, 0.0, 0.5};
            offsets = o;
            break;
        }
        case GaitMode::Amble: {
            static constexpr double o[] = {0.0, 0.3, 0.15, 0.45};
            offsets = o;
            break;
        }
        case GaitMode::Walk: {
            static constexpr double o[] = {0.0, 0.5, 0.25, 0.75};
            offsets = o;
            break;
        }
        default: {
            static constexpr double o[] = {0.0, 0.5, 0.0, 0.5};
            offsets = o;
        }
        }

        const double period = std::max(cfg_.period, 1e-3);
        std::array<Eigen::Vector3d, 4> foot;
        for (int i = 0; i < 4; ++i) {
            double s = std::fmod(t / period + offsets[i], 1.0);
            if (s < 0.0) {
                s += 1.0;
            }

            double y = cfg_.stance_y;
            double z = cfg_.stance_z;
            if (s < 0.5) {
                const double u = 2.0 * s;
                y              = cfg_.stance_y + cfg_.step_length * (2.0 * u - 1.0);
                z              = cfg_.stance_z + cfg_.step_height * std::sin(std::numbers::pi * u);
            } else {
                const double v = 2.0 * (s - 0.5);
                y              = cfg_.stance_y + cfg_.step_length * (1.0 - 2.0 * v);
                z              = cfg_.stance_z;
            }
            foot[i] = Eigen::Vector3d(0.0, y, z);
        }
        return foot;
    }

private:
    GaitConfig cfg_;
};

} // namespace dogbot_core::controller
