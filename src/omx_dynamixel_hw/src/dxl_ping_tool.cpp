// src/dxl_ping_tool.cpp - 실HW ping/scan 스모크 도구 (standalone)
#include <cstdio>
#include <cstdlib>
#include "omx_dynamixel_hw/dxl_bus.hpp"

using namespace omx_dynamixel_hw;

int main(int argc, char ** argv) {
  std::string port = (argc > 1) ? argv[1] : "/dev/ttyACM1";
  int baud = (argc > 2) ? std::atoi(argv[2]) : 1000000;
  DxlBus bus(port, baud);
  if (!bus.open()) { std::fprintf(stderr, "open failed: %s\n", port.c_str()); return 1; }
  int fail = 0;
  for (uint8_t id = 11; id <= 16; ++id) {
    uint16_t model = 0;
    std::string err;
    if (bus.ping(id, model, err)) {
      std::printf("ID %u OK model=%u\n", id, model);
    } else {
      std::printf("ID %u FAIL: %s\n", id, err.c_str());
      ++fail;
    }
  }
  bus.close();
  return fail == 0 ? 0 : 2;
}
