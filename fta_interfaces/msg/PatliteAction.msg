# PatliteLedBuzzer 사전 정의 액션 메시지
# C# ePlayBuzzerLed enum 포팅

# 액션 타입 (0~25 범위)
uint8 action

# 액션 타입 상수 정의
uint8 CONTAINER_OK = 0
uint8 SIZE_CHECK_START = 1
uint8 SIZE_MEASURE_OK = 2
uint8 NO_QR_SIZE_MEASURE_OK = 3
uint8 CHECK_COMPLETE = 4
uint8 QR_PICKUP = 5
uint8 QR_MEASURE_OK = 6
uint8 NO_QR_PICKUP = 7
uint8 NO_QR_MEASURE_OK = 8
uint8 NO_QR_CHECK_COMPLETE = 9
uint8 SET_ITEM = 10
uint8 SET_ITEM_NORMAL = 11
uint8 SET_ITEM_PICKUP = 12
uint8 SET_ITEM_SIZE_CHECK_START = 13
uint8 SET_ITEM_MEASURE_OK = 14
uint8 SET_ITEM_CHECK_COMPLETE = 15
uint8 CLEAR_ITEM = 16
uint8 DROP = 17
uint8 ACTION_START = 18
uint8 ACTION_FINISH = 19
uint8 ACTION_FAIL = 20
uint8 EMERGENCY = 21
uint8 EMERGENCY2 = 22
uint8 DEVICE_ERROR = 23
uint8 DEVICE_ERROR_CLEAR = 24
uint8 INVALID_PLACE = 25

# 타임스탬프
builtin_interfaces/Time timestamp
