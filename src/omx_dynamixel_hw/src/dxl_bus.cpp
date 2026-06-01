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
  port_h_ = dynamixel::PortHandler::getPortHandler(port_.c_str());
  packet_h_ = dynamixel::PacketHandler::getPacketHandler(2.0);
  if (port_h_ == nullptr || packet_h_ == nullptr) return false;
  if (!port_h_->openPort()) return false;
  if (!port_h_->setBaudRate(baud_)) return false;
  return true;
}

void DxlBus::close() {
  sw_goal_.reset();
  sr_state_.reset();
  if (port_h_ != nullptr) {
    port_h_->closePort();
    port_h_ = nullptr;
  }
}

bool DxlBus::ping(uint8_t id, uint16_t & model_number, std::string & err) {
  uint8_t dxl_err = 0;
  int rc = packet_h_->ping(port_h_, id, &model_number, &dxl_err);
  if (rc != COMM_SUCCESS) { err = packet_h_->getTxRxResult(rc); return false; }
  if (dxl_err != 0) { err = packet_h_->getRxPacketError(dxl_err); return false; }
  return true;
}

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

bool DxlBus::setup_groups(const std::vector<uint8_t> & ids) {
  sw_goal_ = std::make_unique<dynamixel::GroupSyncWrite>(
    port_h_, packet_h_, reg::GOAL_POSITION.address, reg::GOAL_POSITION.length);
  sr_state_ = std::make_unique<dynamixel::GroupSyncRead>(
    port_h_, packet_h_, STATE_BLOCK_START, STATE_BLOCK_LEN);
  for (uint8_t id : ids) {
    if (!sr_state_->addParam(id)) return false;
  }
  return true;
}

bool DxlBus::sync_write_goal_position(const std::map<uint8_t, uint32_t> & goals) {
  if (!sw_goal_) return false;
  sw_goal_->clearParam();
  for (const auto & [id, tick] : goals) {
    uint8_t buf[4] = {DXL_LOBYTE(DXL_LOWORD(tick)), DXL_HIBYTE(DXL_LOWORD(tick)),
                      DXL_LOBYTE(DXL_HIWORD(tick)), DXL_HIBYTE(DXL_HIWORD(tick))};
    if (!sw_goal_->addParam(id, buf)) return false;
  }
  return sw_goal_->txPacket() == COMM_SUCCESS;
}

bool DxlBus::sync_read_present_states(const std::vector<uint8_t> & ids,
                                      std::map<uint8_t, PresentState> & out) {
  if (!sr_state_) return false;
  if (sr_state_->txRxPacket() != COMM_SUCCESS) return false;
  bool all_ok = true;
  for (uint8_t id : ids) {
    if (!sr_state_->isAvailable(id, STATE_BLOCK_START, STATE_BLOCK_LEN)) {
      all_ok = false;
      continue;
    }
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
