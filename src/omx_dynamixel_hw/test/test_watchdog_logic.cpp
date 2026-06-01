// test/test_watchdog_logic.cpp
#include <gtest/gtest.h>
#include "omx_dynamixel_hw/watchdog_logic.hpp"

using namespace omx_dynamixel_hw;

TEST(Watchdog, NominalIsSafe) {
  WatchdogThresholds th{.max_temperature_c = 70, .min_input_voltage_v = 9.5,
                        .max_comm_fail_streak = 5};
  JointHealth h{.temperature_c = 40, .input_voltage_v = 12.0,
                .hardware_error_status = 0, .comm_fail_streak = 0};
  EXPECT_FALSE(should_safety_stop(h, th).triggered);
}

TEST(Watchdog, OverTemperatureTrips) {
  WatchdogThresholds th{70, 9.5, 5};
  JointHealth h{.temperature_c = 75, .input_voltage_v = 12.0,
                .hardware_error_status = 0, .comm_fail_streak = 0};
  auto r = should_safety_stop(h, th);
  EXPECT_TRUE(r.triggered);
  EXPECT_NE(r.reason.find("temperature"), std::string::npos);
}

TEST(Watchdog, HardwareErrorBitTrips) {
  WatchdogThresholds th{70, 9.5, 5};
  JointHealth h{.temperature_c = 40, .input_voltage_v = 12.0,
                .hardware_error_status = 0x01, .comm_fail_streak = 0};
  EXPECT_TRUE(should_safety_stop(h, th).triggered);
}

TEST(Watchdog, CommFailStreakTrips) {
  WatchdogThresholds th{70, 9.5, 5};
  JointHealth h{.temperature_c = 40, .input_voltage_v = 12.0,
                .hardware_error_status = 0, .comm_fail_streak = 6};
  EXPECT_TRUE(should_safety_stop(h, th).triggered);
}

TEST(Watchdog, UnderVoltageTrips) {
  WatchdogThresholds th{70, 9.5, 5};
  JointHealth h{.temperature_c = 40, .input_voltage_v = 9.0,
                .hardware_error_status = 0, .comm_fail_streak = 0};
  EXPECT_TRUE(should_safety_stop(h, th).triggered);
}
