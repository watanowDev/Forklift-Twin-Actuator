#ifndef FTA_ACTUATORS__PATLITE_DRIVER_HPP_
#define FTA_ACTUATORS__PATLITE_DRIVER_HPP_

#include <string>
#include <memory>

namespace fta_actuators
{

// ============================================
// Patlite LED 색상 열거형
// ============================================
enum class LEDColor {
  CLEAR = 0,
  RED = 1,
  YELLOW = 2,
  LEMON = 3,
  GREEN = 4,
  SKYBLUE = 5,
  BLUE = 6,
  PURPLE = 7,
  PEACHBLOW = 8,
  WHITE = 9
};

// ============================================
// Patlite LED 패턴 열거형
// ============================================
enum class LEDPattern {
  OFF = 0,
  CONTINUOUS = 1,
  FLASH1 = 2,
  FLASH2 = 3,
  FLASH3 = 4
};

// ============================================
// Patlite 부저 패턴 열거형
// ============================================
enum class BuzzerPattern {
  PATTERN1 = 0,
  PATTERN2 = 1,
  PATTERN3 = 2,
  PATTERN4 = 3,
  PATTERN5 = 4,
  PATTERN6 = 5,
  PATTERN7 = 6,
  PATTERN8 = 7,
  PATTERN9 = 8,
  PATTERN10 = 9,
  PATTERN11 = 10,
  STOP = 11
};

// ============================================
// Patlite 드라이버 클래스
// NeUsbController.dll 래핑
// ============================================
class PatliteDriver
{
public:
  PatliteDriver();
  ~PatliteDriver();
  
  // 디바이스 연결
  bool open_device();
  void close_device();
  bool is_connected() const { return is_connected_; }
  
  // LED 제어
  bool set_light(LEDColor color, LEDPattern pattern);
  
  // 부저 제어
  bool set_buzzer(BuzzerPattern pattern, int volume, int count);
  
  // 디바이스 상태 확인
  bool get_device_state(bool & buzzer_state, bool & led_state, bool & touch_state);
  
  // 문자열 변환 유틸리티
  static LEDColor string_to_color(const std::string & color_str);
  static LEDPattern string_to_pattern(const std::string & pattern_str);
  static BuzzerPattern string_to_buzzer_pattern(const std::string & pattern_str);
  
private:
  bool is_connected_;
  void* dll_handle_;  // DLL 핸들 (Windows) 또는 SO 핸들 (Linux)
  
  // DLL 함수 포인터들
  int (*ne_open_device_)();
  int (*ne_close_device_)();
  int (*ne_set_light_)(int color, int pattern);
  int (*ne_set_buz_)(int pattern, int volume, int count);
  int (*ne_get_device_state_)(int* buzzer, int* led, int* touch);
  
  // DLL 로드
  bool load_dll();
  void unload_dll();
};

}  // namespace fta_actuators

#endif  // FTA_ACTUATORS__PATLITE_DRIVER_HPP_
