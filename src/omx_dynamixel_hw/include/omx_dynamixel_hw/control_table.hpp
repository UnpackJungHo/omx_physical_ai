// include/omx_dynamixel_hw/control_table.hpp
#pragma once
#include <cstdint>

namespace omx_dynamixel_hw {

struct Reg { uint16_t address; uint16_t length; };

// --- X-series 2.0 control table (프로토콜 상수, 튜닝값 아님) ---
namespace reg {
inline constexpr Reg TORQUE_ENABLE          {64, 1};
inline constexpr Reg HARDWARE_ERROR_STATUS  {70, 1};
inline constexpr Reg OPERATING_MODE         {11, 1};
inline constexpr Reg POSITION_P_GAIN        {84, 2};
inline constexpr Reg POSITION_I_GAIN        {82, 2};
inline constexpr Reg POSITION_D_GAIN        {80, 2};
inline constexpr Reg PROFILE_ACCELERATION   {108, 4};
inline constexpr Reg PROFILE_VELOCITY       {112, 4};
inline constexpr Reg GOAL_POSITION          {116, 4};
inline constexpr Reg GOAL_CURRENT           {102, 2};
inline constexpr Reg CURRENT_LIMIT          {38, 2};
inline constexpr Reg PRESENT_CURRENT        {126, 2};
inline constexpr Reg PRESENT_VELOCITY       {128, 4};
inline constexpr Reg PRESENT_POSITION       {132, 4};
inline constexpr Reg PRESENT_INPUT_VOLTAGE  {144, 2};
inline constexpr Reg PRESENT_TEMPERATURE    {146, 1};
}  // namespace reg

inline constexpr int    TICKS_PER_REV = 4096;
inline constexpr int    CENTER_TICK   = 2048;
inline constexpr double CURRENT_MA_PER_UNIT = 2.69;  // XM430/XL430

double tick_to_rad(int tick);
int    rad_to_tick(double rad);
double current_unit_to_ma(int unit);
int    decode_present_current(uint16_t raw);  // int16 디코드

}  // namespace omx_dynamixel_hw
