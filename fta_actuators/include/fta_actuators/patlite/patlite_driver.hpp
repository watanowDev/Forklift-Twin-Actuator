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
    // Patlite LED Color Enum (matches C# implementation)
    // ============================================
    enum class LEDColor
    {
        CLEAR = 0,     // Off
        RED = 1,       // Red
        YELLOW = 2,    // Yellow
        LEMON = 3,     // Lemon
        GREEN = 4,     // Green
        SKYBLUE = 5,   // Sky Blue (Cyan)
        BLUE = 6,      // Blue
        PURPLE = 7,    // Purple
        PEACHBLOW = 8, // Peach Blow
        WHITE = 9      // White
    };

    // ============================================
    // Patlite LED Pattern Enum (matches C# implementation)
    // ============================================
    enum class LEDPattern
    {
        OFF = 0,        // Off
        CONTINUOUS = 1, // Continuous (always on)
        PATTERN2 = 2,   // Pattern 2
        PATTERN3 = 3,   // Pattern 3
        PATTERN4 = 4,   // Pattern 4
        PATTERN5 = 5,   // Pattern 5
        PATTERN6 = 6    // Pattern 6
    };

    // ============================================
    // Patlite Buzzer Pattern Enum (matches C# implementation)
    // ============================================
    enum class BuzzerPattern
    {
        OFF = 0,        // Off
        CONTINUOUS = 1, // Continuous sound
        PATTERN1 = 2,   // Pattern 1
        PATTERN2 = 3,   // Pattern 2
        PATTERN3 = 4,   // Pattern 3
        PATTERN4 = 5,   // Pattern 4
        PATTERN5 = 6,   // Pattern 5
        PATTERN6 = 7    // Pattern 6
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
