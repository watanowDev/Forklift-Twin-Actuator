#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_controller.hpp"

namespace fta_actuators
{

PatliteLedBuzzerController::PatliteLedBuzzerController(rclcpp::Logger logger)
: logger_(logger)
{
  initialize_action_map();
  RCLCPP_INFO(logger_, "PatliteLedBuzzerController initialized with %zu action patterns", action_map_.size());
}

void PatliteLedBuzzerController::register_action(
  PatliteLedBuzzerAction action,
  LEDColor led_color,
  LEDPattern led_pattern,
  BuzzerPattern buzzer_pattern,
  int buzzer_count)
{
  action_map_[action] = PatliteLedBuzzerCommand(led_color, led_pattern, buzzer_pattern, buzzer_count);
}

void PatliteLedBuzzerController::initialize_action_map()
{
  // C# Pattlite_Buzzer_LED 함수의 각 case를 맵으로 등록
  
  // CONTAINER_OK: 녹색 연속 + 부저 연속
  register_action(
    PatliteLedBuzzerAction::CONTAINER_OK,
    LEDColor::GREEN, LEDPattern::CONTINUOUS,
    BuzzerPattern::CONTINUOUS, 1);

  // SIZE_CHECK_START: 녹색 Pattern3 + 부저 Pattern2
  register_action(
    PatliteLedBuzzerAction::SIZE_CHECK_START,
    LEDColor::GREEN, LEDPattern::PATTERN3,
    BuzzerPattern::PATTERN2, 1);

  // SIZE_MEASURE_OK: 녹색 Pattern6 + 부저 Pattern1
  register_action(
    PatliteLedBuzzerAction::SIZE_MEASURE_OK,
    LEDColor::GREEN, LEDPattern::PATTERN6,
    BuzzerPattern::PATTERN1, 1);

  // NO_QR_SIZE_MEASURE_OK: 보라색 Pattern6 + 부저 Pattern1
  register_action(
    PatliteLedBuzzerAction::NO_QR_SIZE_MEASURE_OK,
    LEDColor::PURPLE, LEDPattern::PATTERN6,
    BuzzerPattern::PATTERN1, 1);

  // QR_PICKUP: 녹색 Pattern3 + 부저 Pattern2
  register_action(
    PatliteLedBuzzerAction::QR_PICKUP,
    LEDColor::GREEN, LEDPattern::PATTERN3,
    BuzzerPattern::PATTERN2, 1);

  // QR_MEASURE_OK: 녹색 연속 + 부저 연속
  register_action(
    PatliteLedBuzzerAction::QR_MEASURE_OK,
    LEDColor::GREEN, LEDPattern::CONTINUOUS,
    BuzzerPattern::CONTINUOUS, 1);

  // NO_QR_PICKUP: 보라색 Pattern3 + 부저 Pattern2
  register_action(
    PatliteLedBuzzerAction::NO_QR_PICKUP,
    LEDColor::PURPLE, LEDPattern::PATTERN3,
    BuzzerPattern::PATTERN2, 1);

  // NO_QR_MEASURE_OK: 보라색 연속 + 부저 연속
  register_action(
    PatliteLedBuzzerAction::NO_QR_MEASURE_OK,
    LEDColor::PURPLE, LEDPattern::CONTINUOUS,
    BuzzerPattern::CONTINUOUS, 1);

  // SET_ITEM: 청록색 연속 + 부저 연속
  register_action(
    PatliteLedBuzzerAction::SET_ITEM,
    LEDColor::CYAN, LEDPattern::CONTINUOUS,
    BuzzerPattern::CONTINUOUS, 1);

  // SET_ITEM_NORMAL: 청록색 연속 + 부저 OFF
  register_action(
    PatliteLedBuzzerAction::SET_ITEM_NORMAL,
    LEDColor::CYAN, LEDPattern::CONTINUOUS,
    BuzzerPattern::OFF, 1);

  // SET_ITEM_PICKUP: 청록색 Pattern3 + 부저 Pattern2
  register_action(
    PatliteLedBuzzerAction::SET_ITEM_PICKUP,
    LEDColor::CYAN, LEDPattern::PATTERN3,
    BuzzerPattern::PATTERN2, 1);

  // SET_ITEM_SIZE_CHECK_START: 청록색 Pattern3 + 부저 Pattern2
  register_action(
    PatliteLedBuzzerAction::SET_ITEM_SIZE_CHECK_START,
    LEDColor::CYAN, LEDPattern::PATTERN3,
    BuzzerPattern::PATTERN2, 1);

  // SET_ITEM_MEASURE_OK: 청록색 연속 + 부저 연속
  register_action(
    PatliteLedBuzzerAction::SET_ITEM_MEASURE_OK,
    LEDColor::CYAN, LEDPattern::CONTINUOUS,
    BuzzerPattern::CONTINUOUS, 1);

  // CLEAR_ITEM: 청록색 연속 + 부저 연속
  register_action(
    PatliteLedBuzzerAction::CLEAR_ITEM,
    LEDColor::CYAN, LEDPattern::CONTINUOUS,
    BuzzerPattern::CONTINUOUS, 1);

  // DROP: 녹색 연속 + 부저 OFF
  register_action(
    PatliteLedBuzzerAction::DROP,
    LEDColor::GREEN, LEDPattern::CONTINUOUS,
    BuzzerPattern::OFF, 1);

  // DEVICE_ERROR: 빨강 Pattern2 + 부저 Pattern4
  register_action(
    PatliteLedBuzzerAction::DEVICE_ERROR,
    LEDColor::RED, LEDPattern::PATTERN2,
    BuzzerPattern::PATTERN4, 1);

  // DEVICE_ERROR_CLEAR: 녹색 연속 + 부저 연속
  register_action(
    PatliteLedBuzzerAction::DEVICE_ERROR_CLEAR,
    LEDColor::GREEN, LEDPattern::CONTINUOUS,
    BuzzerPattern::CONTINUOUS, 1);

  // CHECK_COMPLETE: 녹색 연속 + 부저 연속
  register_action(
    PatliteLedBuzzerAction::CHECK_COMPLETE,
    LEDColor::GREEN, LEDPattern::CONTINUOUS,
    BuzzerPattern::CONTINUOUS, 1);

  // NO_QR_CHECK_COMPLETE: 보라색 연속 + 부저 연속
  register_action(
    PatliteLedBuzzerAction::NO_QR_CHECK_COMPLETE,
    LEDColor::PURPLE, LEDPattern::CONTINUOUS,
    BuzzerPattern::CONTINUOUS, 1);

  // SET_ITEM_CHECK_COMPLETE: 청록색 연속 + 부저 연속
  register_action(
    PatliteLedBuzzerAction::SET_ITEM_CHECK_COMPLETE,
    LEDColor::CYAN, LEDPattern::CONTINUOUS,
    BuzzerPattern::CONTINUOUS, 1);

  // INVALID_PLACE: 빨강 Pattern6 + 부저 Pattern3
  register_action(
    PatliteLedBuzzerAction::INVALID_PLACE,
    LEDColor::RED, LEDPattern::PATTERN6,
    BuzzerPattern::PATTERN3, 1);
}

PatliteLedBuzzerCommand PatliteLedBuzzerController::get_command_for_action(PatliteLedBuzzerAction action)
{
  auto it = action_map_.find(action);
  if (it != action_map_.end()) {
    RCLCPP_DEBUG(
      logger_,
      "PatliteLedBuzzerAction %d -> LED(color=%d, pattern=%d), Buzzer(pattern=%d, count=%d)",
      static_cast<int>(action),
      static_cast<int>(it->second.led_color),
      static_cast<int>(it->second.led_pattern),
      static_cast<int>(it->second.buzzer_pattern),
      it->second.buzzer_count);
    return it->second;
  } else {
    RCLCPP_WARN(logger_, "Unknown PatliteLedBuzzerAction: %d, returning OFF command", static_cast<int>(action));
    return PatliteLedBuzzerCommand();  // 기본값(모두 OFF) 반환
  }
}

}  // namespace fta_actuators
