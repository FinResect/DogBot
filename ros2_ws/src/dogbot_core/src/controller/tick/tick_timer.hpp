#pragma once

#include <chrono>

namespace dogbot_core::controller::tick {

class TickTimer {
public:
    void reset(unsigned int cooldown) { // cooldown 单位：毫秒
        deadline_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(cooldown);
        fired_    = false;
    }

    bool tick() {
        if (std::chrono::steady_clock::now() >= deadline_) {
            if (!fired_) {
                fired_ = true;
                return true;
            }
        }
        return false;
    }

private:
    std::chrono::steady_clock::time_point deadline_{};
    bool fired_ = true;
};

} // namespace dogbot_core::controller::tick
