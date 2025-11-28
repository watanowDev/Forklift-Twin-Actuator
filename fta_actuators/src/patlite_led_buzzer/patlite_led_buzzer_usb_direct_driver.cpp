#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_usb_direct_driver.hpp"

namespace fta_actuators
{

PatliteLedBuzzerUsbDirectDriver::PatliteLedBuzzerUsbDirectDriver()
: usb_context_(nullptr),
  device_handle_(nullptr),
  connected_(false),
  kernel_driver_detached_(false)
{
}

PatliteLedBuzzerUsbDirectDriver::~PatliteLedBuzzerUsbDirectDriver()
{
  close();
  cleanup_libusb();
}

bool PatliteLedBuzzerUsbDirectDriver::init_libusb()
{
  int ret = libusb_init(&usb_context_);
  if (ret < 0) {
    std::cerr << "[PatliteLedBuzzerUsb] Failed to initialize libusb: " 
              << libusb_error_name(ret) << std::endl;
    return false;
  }

  // 디버그 레벨 설정 (3 = INFO)
  libusb_set_option(usb_context_, LIBUSB_OPTION_LOG_LEVEL, 3);
  
  std::cout << "[PatliteLedBuzzerUsb] libusb initialized" << std::endl;
  return true;
}

void PatliteLedBuzzerUsbDirectDriver::cleanup_libusb()
{
  if (usb_context_) {
    libusb_exit(usb_context_);
    usb_context_ = nullptr;
    std::cout << "[PatliteLedBuzzerUsb] libusb cleanup complete" << std::endl;
  }
}

bool PatliteLedBuzzerUsbDirectDriver::open_device()
{
  device_handle_ = libusb_open_device_with_vid_pid(
    usb_context_, PATLITE_VID, PATLITE_PID);

  if (!device_handle_) {
    std::cerr << "[PatliteLedBuzzerUsb] Failed to open device (VID:0x" 
              << std::hex << PATLITE_VID << ", PID:0x" << PATLITE_PID 
              << std::dec << ")" << std::endl;
    return false;
  }

  std::cout << "[PatliteLedBuzzerUsb] Device opened successfully" << std::endl;
  return true;
}

bool PatliteLedBuzzerUsbDirectDriver::claim_interface()
{
  // 커널 드라이버가 인터페이스를 사용 중이면 분리
  if (libusb_kernel_driver_active(device_handle_, INTERFACE_NUM) == 1) {
    std::cout << "[PatliteLedBuzzerUsb] Kernel driver is active, detaching..." << std::endl;
    int ret = libusb_detach_kernel_driver(device_handle_, INTERFACE_NUM);
    if (ret == 0) {
      kernel_driver_detached_ = true;
      std::cout << "[PatliteLedBuzzerUsb] Kernel driver detached" << std::endl;
    } else {
      std::cerr << "[PatliteLedBuzzerUsb] Failed to detach kernel driver: " 
                << libusb_error_name(ret) << std::endl;
      return false;
    }
  }

  // 인터페이스 클레임
  int ret = libusb_claim_interface(device_handle_, INTERFACE_NUM);
  if (ret < 0) {
    std::cerr << "[PatliteLedBuzzerUsb] Failed to claim interface: " 
              << libusb_error_name(ret) << std::endl;
    return false;
  }

  std::cout << "[PatliteLedBuzzerUsb] Interface claimed" << std::endl;
  return true;
}

void PatliteLedBuzzerUsbDirectDriver::release_interface()
{
  if (device_handle_) {
    libusb_release_interface(device_handle_, INTERFACE_NUM);
    std::cout << "[PatliteLedBuzzerUsb] Interface released" << std::endl;

    // 커널 드라이버 재연결
    if (kernel_driver_detached_) {
      libusb_attach_kernel_driver(device_handle_, INTERFACE_NUM);
      kernel_driver_detached_ = false;
      std::cout << "[PatliteLedBuzzerUsb] Kernel driver reattached" << std::endl;
    }
  }
}

bool PatliteLedBuzzerUsbDirectDriver::open(const std::string& device_path)
{
  if (connected_) {
    std::cout << "[PatliteLedBuzzerUsb] Already connected" << std::endl;
    return true;
  }

  // libusb 초기화
  if (!init_libusb()) {
    return false;
  }

  // 디바이스 열기
  if (!open_device()) {
    cleanup_libusb();
    return false;
  }

  // 인터페이스 클레임
  if (!claim_interface()) {
    libusb_close(device_handle_);
    device_handle_ = nullptr;
    cleanup_libusb();
    return false;
  }

  connected_ = true;
  std::cout << "[PatliteLedBuzzerUsb] Device ready (VID:0x" << std::hex << PATLITE_VID 
            << ", PID:0x" << PATLITE_PID << std::dec << ")" << std::endl;
  return true;
}

void PatliteLedBuzzerUsbDirectDriver::close()
{
  if (!connected_) {
    return;
  }

  release_interface();

  if (device_handle_) {
    libusb_close(device_handle_);
    device_handle_ = nullptr;
  }

  connected_ = false;
  std::cout << "[PatliteLedBuzzerUsb] Device closed" << std::endl;
}

bool PatliteLedBuzzerUsbDirectDriver::is_connected() const
{
  return connected_;
}

bool PatliteLedBuzzerUsbDirectDriver::send_hid_report(const uint8_t* data, size_t length)
{
  if (!connected_ || !device_handle_) {
    std::cerr << "[PatliteLedBuzzerUsb] Device not connected" << std::endl;
    return false;
  }

  int transferred = 0;
  int ret = libusb_interrupt_transfer(
    device_handle_,
    ENDPOINT_OUT,
    const_cast<uint8_t*>(data),
    length,
    &transferred,
    TIMEOUT_MS
  );

  if (ret < 0) {
    std::cerr << "[PatliteLedBuzzerUsb] Failed to send HID report: " 
              << libusb_error_name(ret) << std::endl;
    return false;
  }

  std::cout << "[PatliteLedBuzzerUsb] Sent " << transferred << " bytes: ";
  for (size_t i = 0; i < length; i++) {
    printf("%02X ", data[i]);
  }
  std::cout << std::endl;

  return true;
}

bool PatliteLedBuzzerUsbDirectDriver::read_hid_report(uint8_t* data, size_t length)
{
  if (!connected_ || !device_handle_) {
    std::cerr << "[PatliteLedBuzzerUsb] Device not connected" << std::endl;
    return false;
  }

  int transferred = 0;
  int ret = libusb_interrupt_transfer(
    device_handle_,
    ENDPOINT_IN,
    data,
    length,
    &transferred,
    TIMEOUT_MS
  );

  if (ret < 0) {
    std::cerr << "[PatliteLedBuzzerUsb] Failed to read HID report: " 
              << libusb_error_name(ret) << std::endl;
    return false;
  }

  std::cout << "[PatliteLedBuzzerUsb] Received " << transferred << " bytes: ";
  for (int i = 0; i < transferred; i++) {
    printf("%02X ", data[i]);
  }
  std::cout << std::endl;

  return true;
}

void PatliteLedBuzzerUsbDirectDriver::build_led_command(
  uint8_t* buffer, LEDColor color, LEDPattern pattern)
{
  // PATLITE 공식 프로토콜 (NE-USB Linux C example 기반)
  // https://github.com/PATLITE-Corporation/NE-USB_linux_C_example
  std::memset(buffer, 0, REPORT_SIZE_OUT);
  
  buffer[0] = 0x00;  // Command version (고정)
  buffer[1] = 0x00;  // Command ID: 0x00 = Control
  buffer[2] = 0xFF;  // Buzzer: 유지 (count:0xF, pattern:0xF)
  buffer[3] = 0x0F;  // Buzzer volume: 유지
  buffer[4] = (static_cast<uint8_t>(color) << 4) | static_cast<uint8_t>(pattern);
  // buffer[5-7] = 0x00 (이미 memset으로 설정됨)
  
  std::cout << "[PatliteLedBuzzerUsb] LED command built: color=" 
            << static_cast<int>(color) 
            << ", pattern=" << static_cast<int>(pattern) << std::endl;
}

void PatliteLedBuzzerUsbDirectDriver::build_buzzer_command(
  uint8_t* buffer, BuzzerPattern pattern, int volume, int count)
{
  // PATLITE 공식 프로토콜 (NE-USB Linux C example 기반)
  // https://github.com/PATLITE-Corporation/NE-USB_linux_C_example
  std::memset(buffer, 0, REPORT_SIZE_OUT);
  
  buffer[0] = 0x00;  // Command version (고정)
  buffer[1] = 0x00;  // Command ID: 0x00 = Control
  buffer[2] = (static_cast<uint8_t>(count) << 4) | static_cast<uint8_t>(pattern);
  buffer[3] = static_cast<uint8_t>(volume);  // 0x00~0x0A (0=mute, 10=max)
  buffer[4] = 0xFF;  // LED: 유지 (color:0xF, pattern:0xF)
  // buffer[5-7] = 0x00 (이미 memset으로 설정됨)
  
  std::cout << "[PatliteLedBuzzerUsb] Buzzer command built: pattern=" 
            << static_cast<int>(pattern) 
            << ", volume=" << volume 
            << ", count=" << count << std::endl;
}

bool PatliteLedBuzzerUsbDirectDriver::set_led(LEDColor color, LEDPattern pattern)
{
  uint8_t buffer[REPORT_SIZE_OUT];
  build_led_command(buffer, color, pattern);
  return send_hid_report(buffer, REPORT_SIZE_OUT);
}

bool PatliteLedBuzzerUsbDirectDriver::set_buzzer(BuzzerPattern pattern, int volume, int count)
{
  uint8_t buffer[REPORT_SIZE_OUT];
  build_buzzer_command(buffer, pattern, volume, count);
  return send_hid_report(buffer, REPORT_SIZE_OUT);
}

bool PatliteLedBuzzerUsbDirectDriver::execute_command(const PatliteLedBuzzerCommand& cmd, int volume)
{
  bool led_ok = set_led(cmd.led_color, cmd.led_pattern);
  bool buzzer_ok = set_buzzer(cmd.buzzer_pattern, volume, cmd.buzzer_count);
  return led_ok && buzzer_ok;
}

bool PatliteLedBuzzerUsbDirectDriver::get_device_state(
  bool& buzzer_state,
  bool& led_state,
  bool& touch_state) const
{
  if (!connected_) {
    return false;
  }

  // TODO: 실제 상태 읽기 구현
  // 현재는 더미 값 반환
  buzzer_state = false;
  led_state = false;
  touch_state = false;

  return true;
}

std::string PatliteLedBuzzerUsbDirectDriver::get_device_info() const
{
  return "PatliteLedBuzzer USB Signal Beacon (Direct libusb control)";
}

std::string PatliteLedBuzzerUsbDirectDriver::get_driver_name() const
{
  return "usb_direct";
}

}  // namespace fta_actuators
