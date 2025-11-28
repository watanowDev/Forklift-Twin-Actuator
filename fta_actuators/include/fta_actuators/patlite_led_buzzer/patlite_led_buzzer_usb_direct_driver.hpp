#ifndef FTA_ACTUATORS__PATLITE_LED_BUZZER_USB_DIRECT_DRIVER_HPP_
#define FTA_ACTUATORS__PATLITE_LED_BUZZER_USB_DIRECT_DRIVER_HPP_

#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_hardware_interface.hpp"
#include <libusb-1.0/libusb.h>
#include <iostream>
#include <cstring>

namespace fta_actuators
{

/**
 * @brief PatliteLedBuzzer USB HID 직접 제어 드라이버
 * 
 * libusb-1.0을 사용하여 PatliteLedBuzzer LED/Buzzer를 직접 제어합니다.
 * 
 * 디바이스 정보:
 * - VID: 0x191a (PATLITE Corporation)
 * - PID: 0x6001 (USB Signal Beacon)
 * - Interface Class: HID
 * - Endpoint IN: 0x81 (64 bytes, Interrupt)
 * - Endpoint OUT: 0x01 (64 bytes, Interrupt)
 * - Report Size: Input 2 bytes, Output 8 bytes
 */
class PatliteLedBuzzerUsbDirectDriver : public PatliteLedBuzzerHardwareInterface
{
public:
  PatliteLedBuzzerUsbDirectDriver();
  ~PatliteLedBuzzerUsbDirectDriver() override;

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
  // USB 디바이스 정보
  static constexpr uint16_t PATLITE_VID = 0x191a;
  static constexpr uint16_t PATLITE_PID = 0x6001;
  static constexpr uint8_t INTERFACE_NUM = 0;
  static constexpr uint8_t ENDPOINT_IN = 0x81;
  static constexpr uint8_t ENDPOINT_OUT = 0x01;
  static constexpr int TIMEOUT_MS = 1000;

  // HID Report 크기
  static constexpr size_t REPORT_SIZE_IN = 2;
  static constexpr size_t REPORT_SIZE_OUT = 8;

  libusb_context* usb_context_;
  libusb_device_handle* device_handle_;
  bool connected_;
  bool kernel_driver_detached_;

  bool init_libusb();
  void cleanup_libusb();
  bool open_device();
  bool claim_interface();
  void release_interface();

  // USB 통신
  bool send_hid_report(const uint8_t* data, size_t length);
  bool read_hid_report(uint8_t* data, size_t length);

  // 프로토콜 변환 (리버스 엔지니어링 필요)
  void build_led_command(uint8_t* buffer, LEDColor color, LEDPattern pattern);
  void build_buzzer_command(uint8_t* buffer, BuzzerPattern pattern, int volume, int count);
};

}  // namespace fta_actuators

#endif  // FTA_ACTUATORS__PATLITE_LED_BUZZER_USB_DIRECT_DRIVER_HPP_

