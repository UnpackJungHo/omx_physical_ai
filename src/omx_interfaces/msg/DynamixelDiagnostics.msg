# DynamixelDiagnostics.msg
# 커스텀 DYNAMIXEL SystemInterface(omx_dynamixel_hw)가 발행하는 모터별 진단.
# 배열은 ids 와 동일 길이로 정렬된다.
std_msgs/Header header
uint8[]   ids
uint16[]  model              # DYNAMIXEL model number (ping). 모델별 임계(전압/전류 단위) 판정용
float64[] present_current_ma
int32[]   temperature_c
float64[] input_voltage_v
uint8[]   hardware_error_status
bool[]    comm_ok
bool[]    diag_valid           # temp/voltage/hwerr 가 1회 이상 round-robin 샘플됐는가 (미샘플 0 오판 방지)
