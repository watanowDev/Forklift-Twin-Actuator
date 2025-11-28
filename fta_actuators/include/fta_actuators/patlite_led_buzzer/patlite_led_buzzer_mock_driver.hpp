#ifndef FTA_ACTUATORS__PATLITE_LED_BUZZER_MOCK_DRIVER_HPP_
#define FTA_ACTUATORS__PATLITE_LED_BUZZER_MOCK_DRIVER_HPP_

#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_hardware_interface.hpp"
#include <iostream>
#include <sstream>

namespace fta_actuators
{

/**
 * @brief PatliteLedBuzzer Mock 드라이버 (하드웨어 없이 테스트용)
 * 
 * 실제 하드웨어 없이도 동작하며, 명령을 로그로 출력합니다.
 * 개발 및 CI/CD 환경에서 유용합니다.
 */
class PatliteLedBuzzerMockDriver : public PatliteLedBuzzerHardwareInterface
{
public:
  PatliteLedBuzzerMockDriver() : connected_(false), buzzer_active_(false) {}
  ~PatliteLedBuzzerMockDriver() override { close(); }

  bool open(const std::string& device_path = "") override
  {
    std::cout << "[PatliteLedBuzzerMock] Opening device: " 
              << (device_path.empty() ? "default" : device_path) << std::endl;
    connected_ = true;
    return true;
  }

  void close() override
  {
    if (connected_) {
      std::cout << "[PatliteLedBuzzerMock] Closing device" << std::endl;
      connected_ = false;
    }
  }

  bool is_connected() const override
  {
    return connected_;
  }

  bool set_led(LEDColor color, LEDPattern pattern) override
  {
    std::cout << "[PatliteLedBuzzerMock] LED: color=" << get_color_name(color)
              << " (" << static_cast<int>(color) << "), "
              << "pattern=" << get_pattern_name(pattern)
              << " (" << static_cast<int>(pattern) << ")"
              << std::endl;
    return true;
  }

  bool set_buzzer(BuzzerPattern pattern, int volume, int count) override
  {
    std::cout << "[PatliteLedBuzzerMock] Buzzer: pattern=" << get_buzzer_pattern_name(pattern)
              << " (" << static_cast<int>(pattern) << "), "
              << "volume=" << volume << ", "
              << "count=" << count
              << std::endl;
    buzzer_active_ = (pattern != BuzzerPattern::OFF);
    return true;
  }

  bool execute_command(const PatliteLedBuzzerCommand& cmd, int volume) override
  {
    std::cout << "[PatliteLedBuzzerMock] Execute Command:" << std::endl;
    set_led(cmd.led_color, cmd.led_pattern);
    set_buzzer(cmd.buzzer_pattern, volume, cmd.buzzer_count);
    return true;
  }

  bool get_device_state(
    bool& buzzer_state,
    bool& led_state,
    bool& touch_state) const override
  {
    buzzer_state = buzzer_active_;
    led_state = connected_;
    touch_state = false;
    return true;
  }

  std::string get_device_info() const override
  {
    return "PatliteLedBuzzer Mock Driver v1.0 (No Hardware)";
  }

  std::string get_driver_name() const override
  {
    return "mock";
  }

private:
  bool connected_;
  bool buzzer_active_;

  std::string get_color_name(LEDColor color) const
  {
    switch (color) {
      case LEDColor::OFF: return "OFF";
      case LEDColor::RED: return "RED";
      case LEDColor::GREEN: return "GREEN";
      case LEDColor::AMBER: return "AMBER";
      case LEDColor::BLUE: return "BLUE";
      case LEDColor::PURPLE: return "PURPLE";
      case LEDColor::CYAN: return "CYAN";
      case LEDColor::CLEAR: return "CLEAR";
      default: return "OTHER";
    }
  }

  std::string get_pattern_name(LEDPattern pattern) const
  {
    switch (pattern) {
      case LEDPattern::OFF: return "OFF";
      case LEDPattern::CONTINUOUS: return "CONTINUOUS";
      case LEDPattern::PATTERN1: return "PATTERN1";
      case LEDPattern::PATTERN2: return "PATTERN2";
      case LEDPattern::PATTERN3: return "PATTERN3";
      case LEDPattern::PATTERN4: return "PATTERN4";
      case LEDPattern::PATTERN5: return "PATTERN5";
      case LEDPattern::PATTERN6: return "PATTERN6";
      default: return "OTHER";
    }
  }

  std::string get_buzzer_pattern_name(BuzzerPattern pattern) const
  {
    switch (pattern) {
      case BuzzerPattern::OFF: return "OFF";
      case BuzzerPattern::CONTINUOUS: return "CONTINUOUS";
      case BuzzerPattern::PATTERN1: return "PATTERN1";
      case BuzzerPattern::PATTERN2: return "PATTERN2";
      case BuzzerPattern::PATTERN3: return "PATTERN3";
      case BuzzerPattern::PATTERN4: return "PATTERN4";
      case BuzzerPattern::PATTERN5: return "PATTERN5";
      case BuzzerPattern::PATTERN6: return "PATTERN6";
      default: return "OTHER";
    }
  }
};

}  // namespace fta_actuators

#endif  // FTA_ACTUATORS__PATLITE_LED_BUZZER_MOCK_DRIVER_HPP_

