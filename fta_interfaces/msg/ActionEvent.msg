# Actuator type
string actuator_type  # "led_buzzer", "speaker", "battery", "power_mode"

# Action command
string action  # "set_light", "set_buzzer", "play_sound", etc.

# Parameters (JSON format)
# Example: {"color": "red", "pattern": "flash", "buzzer_pattern": "pattern1", "count": 3}
string parameters

# Timestamp
builtin_interfaces/Time timestamp
