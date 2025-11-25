# Actuator type
string actuator_type  # "led_buzzer", "speaker", "battery", "power_mode"

# Status
string status  # "idle", "active", "error", "disconnected"

# Status message
string message

# Error code (0: success, >0: error)
int32 error_code

# Timestamp
builtin_interfaces/Time timestamp
