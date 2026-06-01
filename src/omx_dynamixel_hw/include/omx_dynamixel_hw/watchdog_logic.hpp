// include/omx_dynamixel_hw/watchdog_logic.hpp
#pragma once
#include <cstdint>
#include <string>

namespace omx_dynamixel_hw {

struct WatchdogThresholds {
  int    max_temperature_c;
  double min_input_voltage_v;
  int    max_comm_fail_streak;
};

struct JointHealth {
  int      temperature_c;
  double   input_voltage_v;
  uint8_t  hardware_error_status;
  int      comm_fail_streak;
};

struct SafetyDecision { bool triggered; std::string reason; };

inline SafetyDecision should_safety_stop(const JointHealth & h,
                                         const WatchdogThresholds & th) {
  if (h.comm_fail_streak > th.max_comm_fail_streak)
    return {true, "comm_fail_streak exceeded"};
  if (h.hardware_error_status != 0)
    return {true, "hardware_error_status bit set"};
  if (h.temperature_c > th.max_temperature_c)
    return {true, "temperature over limit"};
  if (h.input_voltage_v < th.min_input_voltage_v)
    return {true, "input voltage under limit"};
  return {false, ""};
}

}  // namespace omx_dynamixel_hw
