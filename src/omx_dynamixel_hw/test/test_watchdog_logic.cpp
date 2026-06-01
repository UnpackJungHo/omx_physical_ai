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
                .hardware_error_status = 0, .comm_fail_streak = 0, .diag_valid = true};
  auto r = should_safety_stop(h, th);
  EXPECT_TRUE(r.triggered);
  EXPECT_NE(r.reason.find("temperature"), std::string::npos);
}

TEST(Watchdog, HardwareErrorBitTrips) {
  WatchdogThresholds th{70, 9.5, 5};
  JointHealth h{.temperature_c = 40, .input_voltage_v = 12.0,
                .hardware_error_status = 0x01, .comm_fail_streak = 0, .diag_valid = true};
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
                .hardware_error_status = 0, .comm_fail_streak = 0, .diag_valid = true};
  EXPECT_TRUE(should_safety_stop(h, th).triggered);
}

TEST(Watchdog, UnsampledMotorDoesNotTrip) {
  // 회귀: round-robin 이 아직 안 읽은 모터는 temp/voltage/hwerr 가 0 (초기값).
  // diag_valid=false 이면 0V/0C 라도 트립하면 안 된다(실HW id=16 V=0.0 오탐 케이스).
  WatchdogThresholds th{70, 3.7, 5};
  JointHealth h{.temperature_c = 0, .input_voltage_v = 0.0,
                .hardware_error_status = 0, .comm_fail_streak = 0, .diag_valid = false};
  EXPECT_FALSE(should_safety_stop(h, th).triggered);
  // 단, comm 실패는 valid 와 무관하게 평가된다.
  h.comm_fail_streak = 6;
  EXPECT_TRUE(should_safety_stop(h, th).triggered);
}

TEST(Watchdog, MinVoltageResolvedPerModel) {
  // 혼합 모델 버스: XL430(1060)=9.5V, XL330(1200)=3.7V, 미등록=폴백
  const std::map<uint16_t, double> by_model{{1060, 9.5}, {1200, 3.7}};
  EXPECT_NEAR(resolve_min_voltage(1060, by_model, 9.5), 9.5, 1e-9);
  EXPECT_NEAR(resolve_min_voltage(1200, by_model, 9.5), 3.7, 1e-9);
  EXPECT_NEAR(resolve_min_voltage(9999, by_model, 9.5), 9.5, 1e-9);  // 미등록 -> 폴백
}

TEST(Watchdog, Xl330NominalVoltageIsSafeWithModelThreshold) {
  // XL330 정상 5.0V 는 12V 폴백(9.5)이면 오탐, 모델 임계(3.7)면 안전 - 이번 버그의 회귀 테스트
  const std::map<uint16_t, double> by_model{{1060, 9.5}, {1200, 3.7}};
  JointHealth h{.temperature_c = 40, .input_voltage_v = 5.0,
                .hardware_error_status = 0, .comm_fail_streak = 0, .diag_valid = true};
  WatchdogThresholds th_fallback{70, 9.5, 5};
  EXPECT_TRUE(should_safety_stop(h, th_fallback).triggered);  // 폴백이면 오탐(트립)

  WatchdogThresholds th_model = th_fallback;
  th_model.min_input_voltage_v = resolve_min_voltage(1200, by_model, 9.5);
  EXPECT_FALSE(should_safety_stop(h, th_model).triggered);  // 모델 임계면 안전
}
