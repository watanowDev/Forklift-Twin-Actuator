#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_hardware_interface.hpp"
#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_mock_driver.hpp"
#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_nedll_driver.hpp"
#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_usb_direct_driver.hpp"
#include <memory>
#include <stdexcept>

namespace fta_actuators
{

std::unique_ptr<PatliteLedBuzzerHardwareInterface> PatliteLedBuzzerHardwareFactory::create_driver(
  DriverType type)
{
  switch (type) {
    case DriverType::USB_DIRECT:
      return std::make_unique<PatliteLedBuzzerUsbDirectDriver>();

    case DriverType::NE_DLL:
      return std::make_unique<PatliteLedBuzzerNeDllDriver>();

    case DriverType::MOCK:
      return std::make_unique<PatliteLedBuzzerMockDriver>();

    default:
      throw std::runtime_error("Unknown driver type");
  }
}

PatliteLedBuzzerHardwareFactory::DriverType PatliteLedBuzzerHardwareFactory::parse_driver_type(
  const std::string& type_str)
{
  if (type_str == "usb" || type_str == "usb_direct") {
    return DriverType::USB_DIRECT;
  } else if (type_str == "dll" || type_str == "ne_dll") {
    return DriverType::NE_DLL;
  } else if (type_str == "mock") {
    return DriverType::MOCK;
  } else {
    throw std::runtime_error("Unknown driver type string: " + type_str);
  }
}

}  // namespace fta_actuators
