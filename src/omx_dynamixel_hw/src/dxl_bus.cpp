// src/dxl_bus.cpp
#include "omx_dynamixel_hw/dxl_bus.hpp"

namespace omx_dynamixel_hw {

namespace {
// 연속 상태 블록: PRESENT_CURRENT(126,2) .. PRESENT_POSITION(132,4) = 126..135 (10 bytes)
constexpr uint16_t STATE_BLOCK_START = reg::PRESENT_CURRENT.address;
constexpr uint16_t STATE_BLOCK_LEN =
  reg::PRESENT_POSITION.address + reg::PRESENT_POSITION.length - reg::PRESENT_CURRENT.address;
}  // namespace

DxlBus::DxlBus(const std::string & port, int baud) : port_(port), baud_(baud) {}
DxlBus::~DxlBus() { close(); }

bool DxlBus::open() {
    port_h_ = dynamixel::PortHandler::getPortHandler(port_.c_str());     // 팩토리에서 핸들 획득
    packet_h_ = dynamixel::PacketHandler::getPacketHandler(2.0);         // 프로토콜 2.0
    if (port_h_ == nullptr || packet_h_ == nullptr) return false;        // 방어 1
    if (!port_h_->openPort()) return false;                              // 방어 2: 포트 열기
    if (!port_h_->setBaudRate(baud_)) return false;                      // 방어 3: 보레이트
    return true;
  }


void DxlBus::close() {
  sw_goal_.reset(); // unique_ptr 먼저 파괴 (포트보다 먼저!)
  sr_state_.reset();
  if (port_h_ != nullptr) {
    port_h_->closePort();
    port_h_ = nullptr; // 재호출 안전 (idempotent)
  }
} //  port_h_ 와 packet_h_ 는 delete 하지않음. SDK 가 관리하고 우리는 포트만 닫아줌

// 통신선 문제와 모터 문제 판단 로직
bool DxlBus::ping(uint8_t id, uint16_t & model_number, std::string & err) {
  uint8_t dxl_err = 0;
  int rc = packet_h_->ping(port_h_, id, &model_number, &dxl_err); // 모터에러가 나면 dxl_err이 0에서 변경되나 봄
  if (rc != COMM_SUCCESS) { err = packet_h_->getTxRxResult(rc); return false; } // 통신 에러
  if (dxl_err != 0) { err = packet_h_->getRxPacketError(dxl_err); return false; } // 모터 에러
  return true;
}

bool DxlBus::reboot(uint8_t id) {
  uint8_t dxl_err = 0;
  int rc = packet_h_->reboot(port_h_, id, &dxl_err);
  return rc == COMM_SUCCESS && dxl_err == 0;
}

// TxRx 의 의미: Tx(전송) + Rx(수신)
bool DxlBus::write1(uint8_t id, const Reg & r, uint8_t v) {
  uint8_t e = 0;
  return packet_h_->write1ByteTxRx(port_h_, id, r.address, v, &e) == COMM_SUCCESS && e == 0;
}
bool DxlBus::write2(uint8_t id, const Reg & r, uint16_t v) {
  uint8_t e = 0;
  return packet_h_->write2ByteTxRx(port_h_, id, r.address, v, &e) == COMM_SUCCESS && e == 0;
}
bool DxlBus::write4(uint8_t id, const Reg & r, uint32_t v) {
  uint8_t e = 0;
  return packet_h_->write4ByteTxRx(port_h_, id, r.address, v, &e) == COMM_SUCCESS && e == 0;
}
bool DxlBus::read1(uint8_t id, const Reg & r, uint8_t & v) {
  uint8_t e = 0;
  return packet_h_->read1ByteTxRx(port_h_, id, r.address, &v, &e) == COMM_SUCCESS && e == 0;
}
bool DxlBus::read2(uint8_t id, const Reg & r, uint16_t & v) {
  uint8_t e = 0;
  return packet_h_->read2ByteTxRx(port_h_, id, r.address, &v, &e) == COMM_SUCCESS && e == 0;
}
bool DxlBus::read4(uint8_t id, const Reg & r, uint32_t & v) {
  uint8_t e = 0;
  return packet_h_->read4ByteTxRx(port_h_, id, r.address, &v, &e) == COMM_SUCCESS && e == 0;
}

bool DxlBus::setup_groups(const std::vector<uint8_t> &ids) {
  //"116번지에 4바이트씩 묶어 쓰는 그룹" 생성
  sw_goal_ = std::make_unique<dynamixel::GroupSyncWrite>(
    port_h_, packet_h_, reg::GOAL_POSITION.address, reg::GOAL_POSITION.length); // 116, 4
  // "126번지부터 10바이트 묶어 읽는 그룹"
  sr_state_ = std::make_unique<dynamixel::GroupSyncRead>(
      port_h_, packet_h_, STATE_BLOCK_START, STATE_BLOCK_LEN); // 126, 10
  // read 그룹에만 addParam을 호출하여 "어느 ID들을 읽을지" 매번 고정이기 때문에 한 번 미리 등록
  for (uint8_t id : ids) {
    if (!sr_state_->addParam(id)) return false;
  }
  return true;
}

bool DxlBus::sync_write_goal_position(const std::map<uint8_t, uint32_t> & goals) {
  if (!sw_goal_) return false; // setup_groups 안되어 있으면 return
  sw_goal_->clearParam();      // 이전 사이클 param 비우기
  for (const auto &[id, tick] : goals) {
    // DYNAMIXEL 은 little-endian(LSB 먼저) 이라 낮은 바이트부터 보냄
    // DXL_LOWORD(tick) = 하위 16비트, DXL_HIWORD(tick) = 상위 16비트
    // DXL_LOBYTE(word) = 그 16비트의 하위 8비트, DXL_HIBYTE = 상위 8비트
    uint8_t buf[4] = {DXL_LOBYTE(DXL_LOWORD(tick)), DXL_HIBYTE(DXL_LOWORD(tick)),
                      DXL_LOBYTE(DXL_HIWORD(tick)), DXL_HIBYTE(DXL_HIWORD(tick))};
    if (!sw_goal_->addParam(id, buf)) return false;
  }
  return sw_goal_->txPacket() == COMM_SUCCESS; // Tx만! 응답을 모두 기다리면 느려짐. read 사이클에서 간접 확인
}

bool DxlBus::sync_read_present_states(const std::vector<uint8_t> & ids,
                                      std::map<uint8_t, PresentState> & out) {
  if (!sr_state_) return false;
  if (sr_state_->txRxPacket() != COMM_SUCCESS) return false; // 전 모터에 한번에 요청 + 수신
  bool all_ok = true;
  for (uint8_t id : ids) {
    // isAvailable = 이 id에 데이터가 응답에 들어왔나 확인. 죽어도 멈추지 않고 continue
    if (!sr_state_->isAvailable(id, STATE_BLOCK_START, STATE_BLOCK_LEN)) {
      all_ok = false; // 이 모터 데이터 안 옴 (전체 실패는 아님)
      continue;
    }
    // 주소 크기만큼 잘라서 out에 담기
    PresentState s;
    s.current = static_cast<uint16_t>(
      sr_state_->getData(id, reg::PRESENT_CURRENT.address, reg::PRESENT_CURRENT.length));
    s.velocity = sr_state_->getData(id, reg::PRESENT_VELOCITY.address, reg::PRESENT_VELOCITY.length);
    s.position = sr_state_->getData(id, reg::PRESENT_POSITION.address, reg::PRESENT_POSITION.length);
    out[id] = s;
  }
  return all_ok;
}

}  // namespace omx_dynamixel_hw
