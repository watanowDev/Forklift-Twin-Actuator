# PatliteLedBuzzer LED/Buzzer 제어 메시지
# C# Pattlite_LED_Buzzer_Model 포팅

# LED 색상 (0=OFF, 1=RED, 2=GREEN, 3=AMBER, 4=BLUE, 5=PURPLE, 6=CYAN, 7=CLEAR)
uint8 led_color

# LED 패턴 (0=OFF, 1=CONTINUOUS, 2~7=PATTERN1~6)
uint8 led_pattern

# Buzzer 패턴 (0=OFF, 1=CONTINUOUS, 2~7=PATTERN1~6)
uint8 buzzer_pattern

# Buzzer 반복 횟수
int32 buzzer_count

# 타임스탬프
builtin_interfaces/Time timestamp
