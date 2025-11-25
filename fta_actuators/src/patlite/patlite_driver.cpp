#include "fta_actuators/patlite/patlite_driver.hpp"
#include <iostream>
#include <algorithm>
#include <cctype>

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

namespace fta_actuators
{

  // ============================================
  // ������
  // ============================================
  PatliteDriver::PatliteDriver()
      : is_connected_(false),
        dll_handle_(nullptr),
        ne_open_device_(nullptr),
        ne_close_device_(nullptr),
        ne_set_light_(nullptr),
        ne_set_buz_(nullptr),
        ne_get_device_state_(nullptr)
  {
  }

  // ============================================
  // �Ҹ���
  // ============================================
  PatliteDriver::~PatliteDriver()
  {
    close_device();
    unload_dll();
  }

  // ============================================
  // DLL �ε� (Windows/Linux ȣȯ)
  // ============================================
  bool PatliteDriver::load_dll()
  {
#ifdef _WIN32
    // Windows: NeUsbController.dll �ε�
    dll_handle_ = LoadLibraryA("NeUsbController.dll");
    if (!dll_handle_)
    {
      std::cerr << "Failed to load NeUsbController.dll" << std::endl;
      return false;
    }

    // �Լ� ������ �ε�
    ne_open_device_ = (int (*)())GetProcAddress((HMODULE)dll_handle_, "NE_OpenDevice");
    ne_close_device_ = (int (*)())GetProcAddress((HMODULE)dll_handle_, "NE_CloseDevice");
    ne_set_light_ = (int (*)(int, int))GetProcAddress((HMODULE)dll_handle_, "NE_SetLight");
    ne_set_buz_ = (int (*)(int, int, int))GetProcAddress((HMODULE)dll_handle_, "NE_SetBuz");
    ne_get_device_state_ = (int (*)(int *, int *, int *))GetProcAddress((HMODULE)dll_handle_, "NE_GetDeviceState");

#else
    // Linux: libNeUsbController.so �ε� (���� Linux���� �ִٸ�)
    dll_handle_ = dlopen("libNeUsbController.so", RTLD_LAZY);
    if (!dll_handle_)
    {
      std::cerr << "Failed to load libNeUsbController.so: " << dlerror() << std::endl;
      return false;
    }

    ne_open_device_ = (int (*)())dlsym(dll_handle_, "NE_OpenDevice");
    ne_close_device_ = (int (*)())dlsym(dll_handle_, "NE_CloseDevice");
    ne_set_light_ = (int (*)(int, int))dlsym(dll_handle_, "NE_SetLight");
    ne_set_buz_ = (int (*)(int, int, int))dlsym(dll_handle_, "NE_SetBuz");
    ne_get_device_state_ = (int (*)(int *, int *, int *))dlsym(dll_handle_, "NE_GetDeviceState");
#endif

    // �Լ� ������ Ȯ��
    if (!ne_open_device_ || !ne_close_device_ || !ne_set_light_ ||
        !ne_set_buz_ || !ne_get_device_state_)
    {
      std::cerr << "Failed to load one or more DLL functions" << std::endl;
      unload_dll();
      return false;
    }

    return true;
  }

  // ============================================
  // DLL ��ε�
  // ============================================
  void PatliteDriver::unload_dll()
  {
    if (dll_handle_)
    {
#ifdef _WIN32
      FreeLibrary((HMODULE)dll_handle_);
#else
      dlclose(dll_handle_);
#endif
      dll_handle_ = nullptr;
    }
  }

  // ============================================
  // ����̽� ����
  // ============================================
  bool PatliteDriver::open_device()
  {
    if (is_connected_)
    {
      return true;
    }

    // DLL �ε�
    if (!dll_handle_ && !load_dll())
    {
      return false;
    }

    // ����̽� ����
    int result = ne_open_device_();
    if (result == 0)
    {
      is_connected_ = true;
      std::cout << "Patlite device connected successfully" << std::endl;
      return true;
    }

    std::cerr << "Failed to open Patlite device: error code " << result << std::endl;
    return false;
  }

  // ============================================
  // ����̽� ���� ����
  // ============================================
  void PatliteDriver::close_device()
  {
    if (is_connected_ && ne_close_device_)
    {
      ne_close_device_();
      is_connected_ = false;
      std::cout << "Patlite device disconnected" << std::endl;
    }
  }

  // ============================================
  // LED ����
  // ============================================
  bool PatliteDriver::set_light(LEDColor color, LEDPattern pattern)
  {
    if (!is_connected_ || !ne_set_light_)
    {
      std::cerr << "Device not connected" << std::endl;
      return false;
    }

    int result = ne_set_light_(static_cast<int>(color), static_cast<int>(pattern));
    if (result == 0)
    {
      std::cout << "LED set: color=" << static_cast<int>(color)
                << ", pattern=" << static_cast<int>(pattern) << std::endl;
      return true;
    }

    std::cerr << "Failed to set LED: error code " << result << std::endl;
    return false;
  }

  // ============================================
  // ���� ����
  // ============================================
  bool PatliteDriver::set_buzzer(BuzzerPattern pattern, int volume, int count)
  {
    if (!is_connected_ || !ne_set_buz_)
    {
      std::cerr << "Device not connected" << std::endl;
      return false;
    }

    int result = ne_set_buz_(static_cast<int>(pattern), volume, count);
    if (result == 0)
    {
      std::cout << "Buzzer set: pattern=" << static_cast<int>(pattern)
                << ", volume=" << volume << ", count=" << count << std::endl;
      return true;
    }

    std::cerr << "Failed to set buzzer: error code " << result << std::endl;
    return false;
  }

  // ============================================
  // ����̽� ���� Ȯ��
  // ============================================
  bool PatliteDriver::get_device_state(bool &buzzer_state, bool &led_state, bool &touch_state)
  {
    if (!is_connected_ || !ne_get_device_state_)
    {
      return false;
    }

    int b = 0, l = 0, t = 0;
    int result = ne_get_device_state_(&b, &l, &t);

    buzzer_state = (b != 0);
    led_state = (l != 0);
    touch_state = (t != 0);

    return (result == 0);
  }

  // ============================================
  // ���ڿ� �� LED ���� ��ȯ
  // ============================================
  LEDColor PatliteDriver::string_to_color(const std::string &color_str)
  {
    std::string lower_str = color_str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                   [](unsigned char c)
                   { return std::tolower(c); });

    if (lower_str == "red")
      return LEDColor::RED;
    if (lower_str == "yellow")
      return LEDColor::YELLOW;
    if (lower_str == "green")
      return LEDColor::GREEN;
    if (lower_str == "blue")
      return LEDColor::BLUE;
    if (lower_str == "white")
      return LEDColor::WHITE;
    if (lower_str == "skyblue")
      return LEDColor::SKYBLUE;
    if (lower_str == "purple")
      return LEDColor::PURPLE;
    if (lower_str == "clear" || lower_str == "off")
      return LEDColor::CLEAR;

    return LEDColor::CLEAR;
  }

  // ============================================
  // ���ڿ� �� LED ���� ��ȯ
  // ============================================
  LEDPattern PatliteDriver::string_to_pattern(const std::string &pattern_str)
  {
    std::string lower_str = pattern_str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                   [](unsigned char c)
                   { return std::tolower(c); });

    if (lower_str == "continuous" || lower_str == "on")
      return LEDPattern::CONTINUOUS;
    if (lower_str == "flash1" || lower_str == "flash")
      return LEDPattern::FLASH1;
    if (lower_str == "flash2")
      return LEDPattern::FLASH2;
    if (lower_str == "flash3")
      return LEDPattern::FLASH3;
    if (lower_str == "off")
      return LEDPattern::OFF;

    return LEDPattern::OFF;
  }

  // ============================================
  // ���ڿ� �� ���� ���� ��ȯ
  // ============================================
  BuzzerPattern PatliteDriver::string_to_buzzer_pattern(const std::string &pattern_str)
  {
    std::string lower_str = pattern_str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                   [](unsigned char c)
                   { return std::tolower(c); });

    if (lower_str == "pattern1")
      return BuzzerPattern::PATTERN1;
    if (lower_str == "pattern2")
      return BuzzerPattern::PATTERN2;
    if (lower_str == "pattern3")
      return BuzzerPattern::PATTERN3;
    if (lower_str == "pattern4")
      return BuzzerPattern::PATTERN4;
    if (lower_str == "pattern5")
      return BuzzerPattern::PATTERN5;
    if (lower_str == "pattern6")
      return BuzzerPattern::PATTERN6;
    if (lower_str == "pattern7")
      return BuzzerPattern::PATTERN7;
    if (lower_str == "pattern8")
      return BuzzerPattern::PATTERN8;
    if (lower_str == "pattern9")
      return BuzzerPattern::PATTERN9;
    if (lower_str == "pattern10")
      return BuzzerPattern::PATTERN10;
    if (lower_str == "pattern11")
      return BuzzerPattern::PATTERN11;
    if (lower_str == "stop" || lower_str == "off")
      return BuzzerPattern::STOP;

    return BuzzerPattern::STOP;
  }

} // namespace fta_actuators
