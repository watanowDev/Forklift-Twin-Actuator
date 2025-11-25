# 액추에이터 타입
string actuator_type  # "led_buzzer", "speaker", "battery", "power_mode"

# 현재 상태
string status  # "idle", "active", "error", "disconnected"

# 상세 메시지
string message

# 에러 코드 (0: 정상, >0: 에러)
int32 error_code

# 타임스탬프
builtin_interfaces/Time timestamp
