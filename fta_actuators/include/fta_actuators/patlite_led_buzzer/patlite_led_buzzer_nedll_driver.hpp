#ifndef FTA_ACTUATORS__PATLITE_LED_BUZZER_NEDLL_DRIVER_HPP_
#define FTA_ACTUATORS__PATLITE_LED_BUZZER_NEDLL_DRIVER_HPP_

#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_hardware_interface.hpp"
#include <dlfcn.h>
#include <iostream>
#include <stdexcept>

namespace fta_actuators
{

/**
 * @brief NeUsbController.dll 래퍼 드라이버
 * 
 * C# 원본에서 사용하는 NeUsbController.dll을 C++에서 호출합니다.
 * Linux에서는 Wine을 통해 DLL을 로드할 수 있습니다.
 */
class PatliteLedBuzzerNeDllDriver : public PatliteLedBuzzerHardwareInterface
{
public:
  PatliteLedBuzzerNeDllDriver();
  ~PatliteLedBuzzerNeDllDriver() override;

  bool open(const std::string& device_path = "") override;
  void close() override;
  bool is_connected() const override;

  bool set_led(LEDColor color, LEDPattern pattern) override;
  bool set_buzzer(BuzzerPattern pattern, int volume, int count) override;
  bool execute_command(const PatliteLedBuzzerCommand& cmd, int volume) override;

  bool get_device_state(
    bool& buzzer_state,
    bool& led_state,
    bool& touch_state) const override;

  std::string get_device_info() const override;
  std::string get_driver_name() const override;

private:
  void* dll_handle_;
  bool connected_;

  // DLL 함수 포인터 타입 정의
  // C# 원본: NeUsbController.NE_OpenDevice()
  typedef int (*NE_OpenDevice_t)();
  
  // C# 원본: NeUsbController.NE_CloseDevice()
  typedef int (*NE_CloseDevice_t)();
  
  // C# 원본: NeUsbController.NE_SetLight(LEDColors color, LEDPatterns pattern)
  typedef int (*NE_SetLight_t)(int color, int pattern);
  
  // C# 원본: NeUsbController.NE_SetBuz(BuzzerPatterns pattern, int volume, int count)
  typedef int (*NE_SetBuz_t)(int pattern, int volume, int count);
  
  // C# 원본: NeUsbController.NE_GetDeviceState(out bool buzzer, out bool led, out bool touch)
  typedef int (*NE_GetDeviceState_t)(bool* buzzer, bool* led, bool* touch);

  // 함수 포인터
  NE_OpenDevice_t NE_OpenDevice;
  NE_CloseDevice_t NE_CloseDevice;
  NE_SetLight_t NE_SetLight;
  NE_SetBuz_t NE_SetBuz;
  NE_GetDeviceState_t NE_GetDeviceState;

  bool load_dll(const std::string& dll_path);
  void unload_dll();
  bool load_function_pointers();
};

}  // namespace fta_actuators

#endif  // FTA_ACTUATORS__PATLITE_LED_BUZZER_NEDLL_DRIVER_HPP_

