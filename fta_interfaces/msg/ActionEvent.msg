# 액추에이터 타입
string actuator_type  # "led_buzzer", "speaker", "battery", "power_mode"

# 액션 명령
string action  # "set_light", "set_buzzer", "play_sound", etc.

# 파라미터 (JSON 형식)
# 예: {"color": "red", "pattern": "flash", "buzzer_pattern": "pattern1", "count": 3}
string parameters

# 타임스탬프
builtin_interfaces/Time timestamp
