#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_nedll_driver.hpp"
#include <cstring>

namespace fta_actuators
{

PatliteLedBuzzerNeDllDriver::PatliteLedBuzzerNeDllDriver()
: dll_handle_(nullptr),
  connected_(false),
  NE_OpenDevice(nullptr),
  NE_CloseDevice(nullptr),
  NE_SetLight(nullptr),
  NE_SetBuz(nullptr),
  NE_GetDeviceState(nullptr)
{
}

PatliteLedBuzzerNeDllDriver::~PatliteLedBuzzerNeDllDriver()
{
  close();
  unload_dll();
}

bool PatliteLedBuzzerNeDllDriver::load_dll(const std::string& dll_path)
{
  // DLL 로드 (Linux에서는 .so로 변환하거나 Wine 사용)
  dll_handle_ = dlopen(dll_path.c_str(), RTLD_LAZY);
  if (!dll_handle_) {
    std::cerr << "[PatliteLedBuzzerNeDll] Failed to load DLL: " << dlerror() << std::endl;
    return false;
  }

  std::cout << "[PatliteLedBuzzerNeDll] DLL loaded: " << dll_path << std::endl;
  return load_function_pointers();
}

void PatliteLedBuzzerNeDllDriver::unload_dll()
{
  if (dll_handle_) {
    dlclose(dll_handle_);
    dll_handle_ = nullptr;
  }
}

bool PatliteLedBuzzerNeDllDriver::load_function_pointers()
{
  // 함수 포인터 로드
  NE_OpenDevice = (NE_OpenDevice_t)dlsym(dll_handle_, "NE_OpenDevice");
  NE_CloseDevice = (NE_CloseDevice_t)dlsym(dll_handle_, "NE_CloseDevice");
  NE_SetLight = (NE_SetLight_t)dlsym(dll_handle_, "NE_SetLight");
  NE_SetBuz = (NE_SetBuz_t)dlsym(dll_handle_, "NE_SetBuz");
  NE_GetDeviceState = (NE_GetDeviceState_t)dlsym(dll_handle_, "NE_GetDeviceState");

  if (!NE_OpenDevice || !NE_SetLight || !NE_SetBuz) {
    std::cerr << "[PatliteLedBuzzerNeDll] Failed to load function pointers" << std::endl;
    std::cerr << "  NE_OpenDevice: " << (NE_OpenDevice ? "OK" : "FAIL") << std::endl;
    std::cerr << "  NE_SetLight: " << (NE_SetLight ? "OK" : "FAIL") << std::endl;
    std::cerr << "  NE_SetBuz: " << (NE_SetBuz ? "OK" : "FAIL") << std::endl;
    return false;
  }

  std::cout << "[PatliteLedBuzzerNeDll] All function pointers loaded successfully" << std::endl;
  return true;
}

bool PatliteLedBuzzerNeDllDriver::open(const std::string& device_path)
{
  // DLL 경로 결정
  std::string dll_path = device_path.empty() 
    ? "lib/NeUsbController.dll"  // 기본 경로
    : device_path;

  // DLL 로드
  if (!load_dll(dll_path)) {
    std::cerr << "[PatliteLedBuzzerNeDll] Failed to load DLL from: " << dll_path << std::endl;
    return false;
  }

  // 디바이스 열기
  int result = NE_OpenDevice();
  if (result == 0) {
    std::cout << "[PatliteLedBuzzerNeDll] Device opened successfully" << std::endl;
    connected_ = true;
    return true;
  } else {
    std::cerr << "[PatliteLedBuzzerNeDll] NE_OpenDevice failed with code: " << result << std::endl;
    return false;
  }
}

void PatliteLedBuzzerNeDllDriver::close()
{
  if (connected_ && NE_CloseDevice) {
    NE_CloseDevice();
    std::cout << "[PatliteLedBuzzerNeDll] Device closed" << std::endl;
    connected_ = false;
  }
}

bool PatliteLedBuzzerNeDllDriver::is_connected() const
{
  return connected_;
}

bool PatliteLedBuzzerNeDllDriver::set_led(LEDColor color, LEDPattern pattern)
{
  if (!connected_ || !NE_SetLight) {
    std::cerr << "[PatliteLedBuzzerNeDll] Not connected or function not loaded" << std::endl;
    return false;
  }

  int result = NE_SetLight(static_cast<int>(color), static_cast<int>(pattern));
  
  std::cout << "[PatliteLedBuzzerNeDll] LED: color=" << static_cast<int>(color)
            << ", pattern=" << static_cast<int>(pattern)
            << ", result=" << result << std::endl;
  
  return (result == 0);
}

bool PatliteLedBuzzerNeDllDriver::set_buzzer(BuzzerPattern pattern, int volume, int count)
{
  if (!connected_ || !NE_SetBuz) {
    std::cerr << "[PatliteLedBuzzerNeDll] Not connected or function not loaded" << std::endl;
    return false;
  }

  int result = NE_SetBuz(static_cast<int>(pattern), volume, count);
  
  std::cout << "[PatliteLedBuzzerNeDll] Buzzer: pattern=" << static_cast<int>(pattern)
            << ", volume=" << volume
            << ", count=" << count
            << ", result=" << result << std::endl;
  
  return (result == 0);
}

bool PatliteLedBuzzerNeDllDriver::execute_command(const PatliteLedBuzzerCommand& cmd, int volume)
{
  bool led_ok = set_led(cmd.led_color, cmd.led_pattern);
  bool buzzer_ok = set_buzzer(cmd.buzzer_pattern, volume, cmd.buzzer_count);
  return led_ok && buzzer_ok;
}

bool PatliteLedBuzzerNeDllDriver::get_device_state(
  bool& buzzer_state,
  bool& led_state,
  bool& touch_state) const
{
  if (!connected_ || !NE_GetDeviceState) {
    return false;
  }

  int result = NE_GetDeviceState(&buzzer_state, &led_state, &touch_state);
  return (result == 0);
}

std::string PatliteLedBuzzerNeDllDriver::get_device_info() const
{
  return "PatliteLedBuzzer LED/Buzzer (NeUsbController.dll)";
}

std::string PatliteLedBuzzerNeDllDriver::get_driver_name() const
{
  return "ne_dll";
}

}  // namespace fta_actuators
