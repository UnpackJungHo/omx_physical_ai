// include/omx_dynamixel_hw/dxl_bus.hpp
#pragma once
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "omx_dynamixel_hw/control_table.hpp"

namespace omx_dynamixel_hw {

// 한 번의 GroupSyncRead 로 가져오는 연속 블록(126 PresentCurrent .. 135 PresentPosition).
struct PresentState {
  uint16_t current;   // raw int16, decode_present_current 로 부호 복원
  uint32_t velocity;  // raw int32 tick/s
  uint32_t position;  // raw tick
};

// dynamixel_sdk 호출(PortHandler/PacketHandler/GroupSync*)을 한 곳에 격리하는 얇은 래퍼.
class DxlBus {
public:
  DxlBus(const std::string & port, int baud);
  ~DxlBus();

  bool open();   // port open + baud 설정
  void close();
  bool ping(uint8_t id, uint16_t & model_number, std::string & err);
  bool reboot(uint8_t id);   // DYNAMIXEL reboot (과부하/HW error latch 해제)

  bool write1(uint8_t id, const Reg & r, uint8_t v);
  bool write2(uint8_t id, const Reg & r, uint16_t v);
  bool write4(uint8_t id, const Reg & r, uint32_t v);
  bool read1(uint8_t id, const Reg & r, uint8_t & v);
  bool read2(uint8_t id, const Reg & r, uint16_t & v);
  bool read4(uint8_t id, const Reg & r, uint32_t & v);

  // on_activate 에서 1회 호출: GroupSyncRead/Write 파라미터를 미리 등록(주기적 alloc 회피).
  bool setup_groups(const std::vector<uint8_t> & ids);

  // GroupSyncWrite Goal Position: id->tick
  bool sync_write_goal_position(const std::map<uint8_t, uint32_t> & goals);

  // GroupSyncRead 연속 블록(current/velocity/position) 한 번에 읽기.
  bool sync_read_present_states(const std::vector<uint8_t> & ids,
                                std::map<uint8_t, PresentState> & out);

private:
  std::string port_;
  int baud_;
  dynamixel::PortHandler * port_h_{nullptr};
  dynamixel::PacketHandler * packet_h_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncWrite> sw_goal_;
  std::unique_ptr<dynamixel::GroupSyncRead> sr_state_;
};

}  // namespace omx_dynamixel_hw
