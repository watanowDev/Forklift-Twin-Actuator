# SignalCommand.msg
# 액추에이터 신호 제어 명령 메시지
# 제조사 독립적인 신호 장치(LED, Buzzer 등) 제어를 위한 추상화된 명령

# 액션 타입 (시나리오 식별자)
# 예: "CONTAINER_OK", "SIZE_CHECK_START", "DEVICE_ERROR" 등
string action_type

# 우선순위 (선택적, 향후 확장용)
# 0 = INFO, 1 = WARNING, 2 = ERROR
uint8 priority 0
