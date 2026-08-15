#pragma once

#include <cstdint>

namespace dogbot_msg {

enum class ControllerMode : uint8_t { Auto, Manual, Vision, None };

} // namespace dogbot_msg