// include/omx_dynamixel_hw/watchdog_logic.hpp
#pragma once
#include <cstdint>
#include <map>
#include <string>

namespace omx_dynamixel_hw {

struct WatchdogThresholds {
  int    max_temperature_c;
  double min_input_voltage_v;   // 기본/폴백 (모델 미등록 시)
  int    max_comm_fail_streak;
};

// 혼합 모델 버스(예: XL430 12V + XL330 5V)에서 전압 컷오프는 모델마다 다르다.
// model -> min voltage 맵에 있으면 그 값, 없으면 default 폴백. (순수 함수, 단위 테스트)
inline double resolve_min_voltage(uint16_t model,
                                  const std::map<uint16_t, double> & min_v_by_model,
                                  double default_v) {
  auto it = min_v_by_model.find(model);
  return it != min_v_by_model.end() ? it->second : default_v;
}

struct JointHealth {
  int      temperature_c;
  double   input_voltage_v;
  uint8_t  hardware_error_status;
  int      comm_fail_streak;
  bool     diag_valid{false};  // temp/voltage/hwerr 가 1회 이상 샘플됐는지 (미샘플 0 오판 방지)
};

struct SafetyDecision { bool triggered; std::string reason; };

inline SafetyDecision should_safety_stop(const JointHealth & h,
                                         const WatchdogThresholds & th) {
  // 통신 실패는 매 사이클 메인 read 에서 갱신되므로 항상 평가한다.
  if (h.comm_fail_streak > th.max_comm_fail_streak)
    return {true, "comm_fail_streak exceeded"};
  // 느린 진단 레지스터(temp/voltage/hwerr)는 round-robin 으로 채워지므로, 아직 한 번도
  // 샘플되지 않은 모터의 초기값(0)을 실제 측정으로 오판하지 않도록 valid 일 때만 평가한다.
  if (h.diag_valid) {
    if (h.hardware_error_status != 0)
      return {true, "hardware_error_status bit set"};
    if (h.temperature_c > th.max_temperature_c)
      return {true, "temperature over limit"};
    if (h.input_voltage_v < th.min_input_voltage_v)
      return {true, "input voltage under limit"};
  }
  return {false, ""};
}

}  // namespace omx_dynamixel_hw
