# DynamixelDiagnostics.msg
# 커스텀 DYNAMIXEL SystemInterface(omx_dynamixel_hw)가 발행하는 모터별 진단.
# 배열은 ids 와 동일 길이로 정렬된다.
std_msgs/Header header
uint8[]   ids
float64[] present_current_ma
int32[]   temperature_c
float64[] input_voltage_v
uint8[]   hardware_error_status
bool[]    comm_ok
