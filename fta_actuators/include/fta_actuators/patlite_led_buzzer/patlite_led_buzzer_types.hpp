#ifndef FTA_ACTUATORS__PATLITE_LED_BUZZER_TYPES_HPP_
#define FTA_ACTUATORS__PATLITE_LED_BUZZER_TYPES_HPP_

#include <cstdint>

namespace fta_actuators
{
namespace patlite_led_buzzer
{

/**
 * @brief LED 색상 정의 (C# eLEDColors 포팅)
 */
enum class LEDColor : uint8_t
{
  OFF = 0,
  RED = 1,
  GREEN = 2,
  AMBER = 3,
  BLUE = 4,
  PURPLE = 5,
  CYAN = 6,
  CLEAR = 7,
  OTHER = 0xF
};

/**
 * @brief LED 패턴 정의 (C# eLEDPatterns 포팅)
 */
enum class LEDPattern : uint8_t
{
  OFF = 0,
  CONTINUOUS = 1,      // 연속 점등
  PATTERN1 = 2,        // -- -- -- --
  PATTERN2 = 3,        // -  -  -  -
  PATTERN3 = 4,        // .. .. .. ..
  PATTERN4 = 5,        // .  .  .  .
  PATTERN5 = 6,        // ,  ,  ,  ,
  PATTERN6 = 7,        // ,. ,. ,. ,.
  OTHER = 0xF
};

/**
 * @brief Buzzer 패턴 정의 (C# eBuzzerPatterns 포팅)
 */
enum class BuzzerPattern : uint8_t
{
  OFF = 0,
  CONTINUOUS = 1,      // Beep-
  PATTERN1 = 2,        // Bee~ ing~
  PATTERN2 = 3,        // Be Be Be Be Beep!
  PATTERN3 = 4,        // Beep-! Beep-! Beep-! Beep-! Beep-!
  PATTERN4 = 5,        // Beep, Beep, Beep, Beep, Beep
  PATTERN5 = 6,        // Twinkle Twinkle Little Star Song
  PATTERN6 = 7,        // How's the weather Song
  OTHER = 0xF
};

/**
 * @brief 사전 정의된 LED/Buzzer 액션 시나리오 (C# ePlayBuzzerLed 포팅)
 */
enum class PatliteLedBuzzerAction : uint8_t
{
  CONTAINER_OK = 0,
  SIZE_CHECK_START = 1,
  SIZE_MEASURE_OK = 2,
  NO_QR_SIZE_MEASURE_OK = 3,
  CHECK_COMPLETE = 4,
  QR_PICKUP = 5,
  QR_MEASURE_OK = 6,
  NO_QR_PICKUP = 7,
  NO_QR_MEASURE_OK = 8,
  NO_QR_CHECK_COMPLETE = 9,
  SET_ITEM = 10,
  SET_ITEM_NORMAL = 11,
  SET_ITEM_PICKUP = 12,
  SET_ITEM_SIZE_CHECK_START = 13,
  SET_ITEM_MEASURE_OK = 14,
  SET_ITEM_CHECK_COMPLETE = 15,
  CLEAR_ITEM = 16,
  DROP = 17,
  ACTION_START = 18,
  ACTION_FINISH = 19,
  ACTION_FAIL = 20,
  EMERGENCY = 21,
  EMERGENCY2 = 22,
  DEVICE_ERROR = 23,
  DEVICE_ERROR_CLEAR = 24,
  INVALID_PLACE = 25
};

/**
 * @brief PatliteLedBuzzer LED/Buzzer 제어 명령 구조체
 */
struct PatliteLedBuzzerCommand
{
  LEDColor led_color;
  LEDPattern led_pattern;
  BuzzerPattern buzzer_pattern;
  int buzzer_count;

  PatliteLedBuzzerCommand()
  : led_color(LEDColor::OFF),
    led_pattern(LEDPattern::OFF),
    buzzer_pattern(BuzzerPattern::OFF),
    buzzer_count(0)
  {}

  PatliteLedBuzzerCommand(
    LEDColor color, LEDPattern led_pat,
    BuzzerPattern buz_pat, int buz_cnt)
  : led_color(color),
    led_pattern(led_pat),
    buzzer_pattern(buz_pat),
    buzzer_count(buz_cnt)
  {}
};

}  // namespace patlite_led_buzzer

using patlite_led_buzzer::BuzzerPattern;
using patlite_led_buzzer::LEDColor;
using patlite_led_buzzer::LEDPattern;
using patlite_led_buzzer::PatliteLedBuzzerAction;
using patlite_led_buzzer::PatliteLedBuzzerCommand;

}  // namespace fta_actuators

#endif  // FTA_ACTUATORS__PATLITE_LED_BUZZER_TYPES_HPP_

