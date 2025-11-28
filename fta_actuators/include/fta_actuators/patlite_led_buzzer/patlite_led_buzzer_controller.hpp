#ifndef FTA_ACTUATORS__PATLITE_LED_BUZZER_CONTROLLER_HPP_
#define FTA_ACTUATORS__PATLITE_LED_BUZZER_CONTROLLER_HPP_

#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_types.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <map>

namespace fta_actuators
{

/**
 * @brief PatliteLedBuzzer LED/Buzzer 제어 로직 클래스 (C# StatusService_WATA 포팅)
 * 
 * C# Pattlite_Buzzer_LED 함수를 C++로 포팅한 클래스입니다.
 * PatliteLedBuzzerAction enum을 받아 사전 정의된 LED/Buzzer 패턴으로 변환합니다.
 */
class PatliteLedBuzzerController
{
public:
  explicit PatliteLedBuzzerController(rclcpp::Logger logger);
  ~PatliteLedBuzzerController() = default;

  /**
   * @brief 사전 정의된 액션을 PatliteLedBuzzer 명령으로 변환
   * @param action 실행할 액션 타입
   * @return PatliteLedBuzzerCommand 구조체
   * 
   * C# Pattlite_Buzzer_LED 함수에 대응됩니다.
   */
  PatliteLedBuzzerCommand get_command_for_action(PatliteLedBuzzerAction action);

  /**
   * @brief 모든 사전 정의된 액션 맵 초기화
   * 
   * C# StatusService_WATA.cs의 각 if-else 분기를 맵으로 변환
   */
  void initialize_action_map();

private:
  rclcpp::Logger logger_;
  std::map<PatliteLedBuzzerAction, PatliteLedBuzzerCommand> action_map_;

  /**
   * @brief 개별 액션 등록 헬퍼 함수
   */
  void register_action(
    PatliteLedBuzzerAction action,
    LEDColor led_color,
    LEDPattern led_pattern,
    BuzzerPattern buzzer_pattern,
    int buzzer_count);
};

}  // namespace fta_actuators

#endif  // FTA_ACTUATORS__PATLITE_LED_BUZZER_CONTROLLER_HPP_

