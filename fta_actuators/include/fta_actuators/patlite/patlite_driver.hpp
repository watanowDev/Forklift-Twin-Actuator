#ifndef FTA_ACTUATORS__PATLITE_DRIVER_HPP_
#define FTA_ACTUATORS__PATLITE_DRIVER_HPP_

#include <string>
#include <memory>

namespace fta_actuators
{

    // Forward declaration
    namespace patlite
    {
        class PatliteUsbDriver;
    }

    // ============================================
    // Patlite LED Color Enum
    // ============================================
    enum class LEDColor
    {
        OFF = 0,
        RED = 1,
        YELLOW = 2,
        GREEN = 3,
        BLUE = 4,
        WHITE = 5
    };

    // ============================================
    // Patlite LED Pattern Enum
    // ============================================
    enum class LEDPattern
    {
        OFF = 0,
        ON = 1,
        BLINK = 2,
        BLINK_FAST = 3
    };

    // ============================================
    // Patlite Buzzer Pattern Enum
    // ============================================
    enum class BuzzerPattern
    {
        OFF = 0,
        CONTINUOUS = 1,
        PATTERN_1 = 2,
        PATTERN_2 = 3,
        PATTERN_3 = 4
    };

    // ============================================
    // Patlite Driver Class
    // Uses libusb-1.0 based PatliteUsbDriver
    // ============================================
    class PatliteDriver
    {
    public:
        PatliteDriver();
        ~PatliteDriver();

        // Device connection
        bool initialize();
        bool close_device();
        bool is_connected() const;

        // LED control
        bool set_light(LEDColor color, LEDPattern pattern);

        // Buzzer control
        bool set_buzzer(BuzzerPattern pattern, int volume, int count);

        // Combined LED + Buzzer control
        bool set_all(LEDColor color, LEDPattern led_pattern,
                     BuzzerPattern buzzer_pattern, int volume, int count);

        // Turn off all
        bool turn_off_all();

        // Device state query
        bool get_device_state(bool &buzzer, bool &led, bool &touch);

        // String parsing utilities
        static LEDColor parse_color(const std::string &color_str);
        static LEDPattern parse_led_pattern(const std::string &pattern_str);
        static BuzzerPattern parse_buzzer_pattern(const std::string &pattern_str);

    private:
        bool is_connected_;
        std::unique_ptr<patlite::PatliteUsbDriver> usb_driver_;
    };

} // namespace fta_actuators

#endif // FTA_ACTUATORS__PATLITE_DRIVER_HPP_
