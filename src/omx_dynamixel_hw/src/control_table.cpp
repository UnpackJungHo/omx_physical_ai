// src/control_table.cpp
#include "omx_dynamixel_hw/control_table.hpp"
#include <cmath>

namespace omx_dynamixel_hw {

double tick_to_rad(int tick) {
  return (static_cast<double>(tick) - CENTER_TICK) * (2.0 * M_PI / TICKS_PER_REV);
}

int rad_to_tick(double rad) {
  return static_cast<int>(std::lround(rad * (TICKS_PER_REV / (2.0 * M_PI)))) + CENTER_TICK;
}

double current_unit_to_ma(int unit) { return unit * CURRENT_MA_PER_UNIT; }

double current_unit_to_ma(int unit, double ma_per_unit) { return unit * ma_per_unit; }

double current_ma_per_unit_for_model(uint16_t model) {
  switch (model) {
    case model::XL330_M288: return XL330_CURRENT_MA_PER_UNIT;
    case model::XL430_W250: return XL430_CURRENT_MA_PER_UNIT;
    default:                return CURRENT_MA_PER_UNIT;  // 미지 모델 - 기본값 (호출부에서 경고)
  }
}

int decode_present_current(uint16_t raw) { return static_cast<int16_t>(raw); }

int decode_present_velocity(uint32_t raw) { return static_cast<int32_t>(raw); }

double velocity_unit_to_rad_per_s(int unit) {
  return unit * VELOCITY_RPM_PER_UNIT * (2.0 * M_PI / 60.0);
}

double voltage_unit_to_v(int unit) { return unit * VOLTAGE_V_PER_UNIT; }

}  // namespace omx_dynamixel_hw
