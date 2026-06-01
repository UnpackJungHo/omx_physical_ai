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
inline constexpr double VELOCITY_RPM_PER_UNIT = 0.229; // X-series Present Velocity unit (모델 공통)
inline constexpr double VOLTAGE_V_PER_UNIT = 0.1;      // X-series Present Input Voltage unit (모델 공통)

// Present Current 단위는 모델마다 다르다 (datasheet 상수).
//  - XL430-W250 (model 1060) / XM430: 2.69 mA/unit
//  - XL330-M288 (model 1200):         1.00 mA/unit
// OMX-F: ID11~13 = 1060, ID14~16 = 1200 (gripper=16 은 1200).
inline constexpr double CURRENT_MA_PER_UNIT = 2.69;       // 기본/후방호환 (XL430)
inline constexpr double XL430_CURRENT_MA_PER_UNIT = 2.69;
inline constexpr double XL330_CURRENT_MA_PER_UNIT = 1.00;
namespace model {
inline constexpr uint16_t XL430_W250 = 1060;
inline constexpr uint16_t XL330_M288 = 1200;
}  // namespace model

double tick_to_rad(int tick);
int    rad_to_tick(double rad);
double current_unit_to_ma(int unit);                   // 기본 단위(2.69) - 후방호환
double current_unit_to_ma(int unit, double ma_per_unit);
double current_ma_per_unit_for_model(uint16_t model);  // ping 으로 받은 model -> mA/unit
int    decode_present_current(uint16_t raw);   // int16 디코드
int    decode_present_velocity(uint32_t raw);  // int32 디코드
double velocity_unit_to_rad_per_s(int unit);
double voltage_unit_to_v(int unit);

}  // namespace omx_dynamixel_hw
