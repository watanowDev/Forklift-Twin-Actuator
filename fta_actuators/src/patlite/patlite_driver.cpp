#include "fta_actuators/patlite/patlite_driver.hpp"
#include "fta_actuators/patlite/patlite_usb_driver.hpp"
#include <iostream>
#include <algorithm>
#include <cctype>

namespace fta_actuators
{

    // ============================================
    // Constructor
    // ============================================
    PatliteDriver::PatliteDriver()
        : is_connected_(false),
          usb_driver_(std::make_unique<patlite::PatliteUsbDriver>())
    {
    }

    // ============================================
    // Destructor
    // ============================================
    PatliteDriver::~PatliteDriver()
    {
        close_device();
    }

    // ============================================
    // Initialize and connect to device
    // ============================================
    bool PatliteDriver::initialize()
    {
        // Patlite USB Signal Beacon: Vendor ID = 0x191a, Product ID = 0x6001
        constexpr uint16_t PATLITE_VENDOR_ID = 0x191a;
        constexpr uint16_t PATLITE_PRODUCT_ID = 0x6001;

        if (!usb_driver_->open_device(PATLITE_VENDOR_ID, PATLITE_PRODUCT_ID))
        {
            std::cerr << "Failed to open Patlite USB device (VID: 0x"
                      << std::hex << PATLITE_VENDOR_ID
                      << ", PID: 0x" << PATLITE_PRODUCT_ID << ")" << std::dec << std::endl;
            return false;
        }

        is_connected_ = true;
        return true;
    }

    // ============================================
    // Close device connection
    // ============================================
    bool PatliteDriver::close_device()
    {
        if (is_connected_ && usb_driver_)
        {
            usb_driver_->close_device();
            is_connected_ = false;
        }
        return true;
    }

    // ============================================
    // Check if device is connected
    // ============================================
    bool PatliteDriver::is_connected() const
    {
        return is_connected_;
    }

    // ============================================
    // Set LED light (color + pattern)
    // ============================================
    bool PatliteDriver::set_light(LEDColor color, LEDPattern pattern)
    {
        if (!is_connected_)
        {
            std::cerr << "Device not connected" << std::endl;
            return false;
        }

        return usb_driver_->set_light(color, pattern);
    }

    // ============================================
    // Set buzzer (pattern + volume + count)
    // ============================================
    bool PatliteDriver::set_buzzer(BuzzerPattern pattern, int volume, int count)
    {
        if (!is_connected_)
        {
            std::cerr << "Device not connected" << std::endl;
            return false;
        }

        return usb_driver_->set_buzzer(pattern, volume, count);
    }

    // ============================================
    // Set both LED and buzzer simultaneously
    // ============================================
    bool PatliteDriver::set_all(LEDColor color, LEDPattern led_pattern,
                                BuzzerPattern buzzer_pattern, int volume, int count)
    {
        if (!is_connected_)
        {
            std::cerr << "Device not connected" << std::endl;
            return false;
        }

        return usb_driver_->set_all(color, led_pattern, buzzer_pattern, volume, count);
    }

    // ============================================
    // Turn off all lights and buzzer
    // ============================================
    bool PatliteDriver::turn_off_all()
    {
        if (!is_connected_)
        {
            std::cerr << "Device not connected" << std::endl;
            return false;
        }

        return usb_driver_->turn_off_all();
    }

    // ============================================
    // Get current device state
    // ============================================
    bool PatliteDriver::get_device_state(bool &buzzer, bool &led, bool &touch)
    {
        if (!is_connected_)
        {
            std::cerr << "Device not connected" << std::endl;
            return false;
        }

        return usb_driver_->get_device_state(buzzer, led, touch);
    }

    // ============================================
    // Convert string to LEDColor enum
    // ============================================
    LEDColor PatliteDriver::parse_color(const std::string &color_str)
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
        if (lower_str == "off")
            return LEDColor::OFF;

        std::cerr << "Unknown color: " << color_str << ", defaulting to OFF" << std::endl;
        return LEDColor::OFF;
    }

    // ============================================
    // Convert string to LEDPattern enum
    // ============================================
    LEDPattern PatliteDriver::parse_led_pattern(const std::string &pattern_str)
    {
        std::string lower_str = pattern_str;
        std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                       [](unsigned char c)
                       { return std::tolower(c); });

        if (lower_str == "off")
            return LEDPattern::OFF;
        if (lower_str == "on")
            return LEDPattern::ON;
        if (lower_str == "blink")
            return LEDPattern::BLINK;
        if (lower_str == "blink_fast")
            return LEDPattern::BLINK_FAST;

        std::cerr << "Unknown LED pattern: " << pattern_str << ", defaulting to OFF" << std::endl;
        return LEDPattern::OFF;
    }

    // ============================================
    // Convert string to BuzzerPattern enum
    // ============================================
    BuzzerPattern PatliteDriver::parse_buzzer_pattern(const std::string &pattern_str)
    {
        std::string lower_str = pattern_str;
        std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(),
                       [](unsigned char c)
                       { return std::tolower(c); });

        if (lower_str == "off")
            return BuzzerPattern::OFF;
        if (lower_str == "continuous")
            return BuzzerPattern::CONTINUOUS;
        if (lower_str == "pattern_1" || lower_str == "pattern1")
            return BuzzerPattern::PATTERN_1;
        if (lower_str == "pattern_2" || lower_str == "pattern2")
            return BuzzerPattern::PATTERN_2;
        if (lower_str == "pattern_3" || lower_str == "pattern3")
            return BuzzerPattern::PATTERN_3;

        std::cerr << "Unknown buzzer pattern: " << pattern_str << ", defaulting to OFF" << std::endl;
        return BuzzerPattern::OFF;
    }

} // namespace fta_actuators
