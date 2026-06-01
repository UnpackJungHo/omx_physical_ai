// test/test_control_table.cpp
#include <gtest/gtest.h>
#include <cmath>
#include "omx_dynamixel_hw/control_table.hpp"

using namespace omx_dynamixel_hw;

TEST(ControlTable, RegisterAddressesAreXSeriesConstants) {
  EXPECT_EQ(reg::TORQUE_ENABLE.address, 64u);
  EXPECT_EQ(reg::GOAL_POSITION.address, 116u);
  EXPECT_EQ(reg::GOAL_POSITION.length, 4u);
  EXPECT_EQ(reg::PRESENT_POSITION.address, 132u);
  EXPECT_EQ(reg::PRESENT_VELOCITY.address, 128u);
  EXPECT_EQ(reg::PRESENT_CURRENT.address, 126u);
  EXPECT_EQ(reg::PRESENT_CURRENT.length, 2u);
  EXPECT_EQ(reg::PRESENT_TEMPERATURE.address, 146u);
  EXPECT_EQ(reg::PRESENT_INPUT_VOLTAGE.address, 144u);
  EXPECT_EQ(reg::HARDWARE_ERROR_STATUS.address, 70u);
}

TEST(ControlTable, TickRadRoundTrip) {
  // X-series: 4096 tick / rev, center 2048 = 0 rad
  EXPECT_NEAR(tick_to_rad(2048), 0.0, 1e-9);
  EXPECT_NEAR(tick_to_rad(2048 + 1024), M_PI / 2.0, 1e-3);
  EXPECT_EQ(rad_to_tick(0.0), 2048);
  // round-trip
  for (int t : {0, 1024, 2048, 3072, 4095}) {
    EXPECT_NEAR(tick_to_rad(rad_to_tick(tick_to_rad(t))), tick_to_rad(t), 1e-6);
  }
}

TEST(ControlTable, CurrentUnitToMilliAmp) {
  // XM430/XL430: 2.69 mA per unit
  EXPECT_NEAR(current_unit_to_ma(100), 269.0, 1.0);
  EXPECT_NEAR(current_unit_to_ma(0), 0.0, 1e-9);
}

TEST(ControlTable, CurrentUnitPerModel) {
  // model 1060 = XL430 (2.69 mA/unit), 1200 = XL330 (1.0 mA/unit)
  EXPECT_NEAR(current_ma_per_unit_for_model(1060), 2.69, 1e-9);
  EXPECT_NEAR(current_ma_per_unit_for_model(1200), 1.00, 1e-9);
  // 미지 모델은 기본값(2.69)으로 폴백
  EXPECT_NEAR(current_ma_per_unit_for_model(9999), 2.69, 1e-9);
  // model 인지 변환: XL330 그리퍼 100 unit -> 100 mA
  EXPECT_NEAR(current_unit_to_ma(100, current_ma_per_unit_for_model(1200)), 100.0, 1e-9);
  EXPECT_NEAR(current_unit_to_ma(100, current_ma_per_unit_for_model(1060)), 269.0, 1e-9);
}

TEST(ControlTable, SignedCurrentDecode) {
  // Present Current는 int16. 0xFFFF -> -1 unit
  EXPECT_EQ(decode_present_current(0xFFFF), -1);
  EXPECT_EQ(decode_present_current(0x0064), 100);
}

TEST(ControlTable, VelocityDecodeAndConvert) {
  // Present Velocity는 int32. 0xFFFFFFFF -> -1 unit
  EXPECT_EQ(decode_present_velocity(0xFFFFFFFFu), -1);
  EXPECT_EQ(decode_present_velocity(0x00000064u), 100);
  // X-series: 0.229 rev/min per unit
  EXPECT_NEAR(velocity_unit_to_rad_per_s(0), 0.0, 1e-9);
  EXPECT_NEAR(velocity_unit_to_rad_per_s(100), 100 * 0.229 * 2.0 * M_PI / 60.0, 1e-6);
}
