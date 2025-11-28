#ifndef FTA_ACTUATORS__PATLITE_LED_BUZZER_USB_DRIVER_HPP_
#define FTA_ACTUATORS__PATLITE_LED_BUZZER_USB_DRIVER_HPP_

#include <libusb-1.0/libusb.h>
#include <cstdint>
#include <string>
#include <vector>
#include <memory>

namespace fta_actuators
{
    // Forward declarations
    enum class LEDColor;
    enum class LEDPattern;
    enum class BuzzerPattern;
    struct PatliteLedBuzzerCommand;

    namespace patlite_led_buzzer
    {

        // ============================================
        // USB 장치 정보
        // ============================================
        struct UsbDeviceInfo
        {
            uint16_t vendor_id;
            uint16_t product_id;
            uint8_t interface_number;
            uint8_t endpoint_in;
            uint8_t endpoint_out;
            std::string manufacturer;
            std::string product;
            std::string serial_number;
        };

        // ============================================
        // PatliteLedBuzzer USB 드라이버 클래스
        // ============================================
        class PatliteLedBuzzerUsbDriver
        {
        public:
            PatliteLedBuzzerUsbDriver();
            ~PatliteLedBuzzerUsbDriver();

            // USB 장치 초기화 및 연결
            bool open_device(uint16_t vendor_id = 0x191a, uint16_t product_id = 0x0000);
            void close_device();
            bool is_connected() const;

            // LED 제어
            bool set_light(LEDColor color, LEDPattern pattern);

            // Buzzer 제어
            bool set_buzzer(BuzzerPattern pattern, int volume, int count);

            // 장치 상태 조회
            bool get_device_state(bool &buzzer_active, bool &led_active, bool &touch_active);

            // 전체 제어 (LED + Buzzer 동시)
            bool set_all(LEDColor color, LEDPattern led_pattern,
                         BuzzerPattern buzzer_pattern, int volume, int count);

            // 모두 끄기
            bool turn_off_all();

            // 장치 정보 가져오기
            UsbDeviceInfo get_device_info() const;

            // 에러 메시지 가져오기
            std::string get_last_error() const;

        private:
            // libusb 컨텍스트 및 장치 핸들
            libusb_context *context_;
            libusb_device_handle *device_handle_;
            UsbDeviceInfo device_info_;
            std::string last_error_;

            // USB 통신 파라미터 (USB 패킷 캡처 분석 후 설정)
            struct UsbProtocol
            {
                // Control Transfer 파라미터
                uint8_t request_type;
                uint8_t request;
                uint16_t value;
                uint16_t index;
                unsigned int timeout_ms;

                UsbProtocol()
                    : request_type(0x21), // Host to Device, Class, Interface (HID 표준)
                      request(0x09),      // SET_REPORT (HID 표준)
                      value(0x0200),      // TODO: USB 캡처로 확인 필요
                      index(0x0000),      // Interface 0
                      timeout_ms(1000)
                {
                }
            };

            UsbProtocol protocol_;

            // 내부 헬퍼 함수
            bool initialize_libusb();
            bool find_and_claim_device(uint16_t vendor_id, uint16_t product_id);
            bool send_control_transfer(const uint8_t *data, int length);
            bool receive_control_transfer(uint8_t *data, int length);
            void set_error(const std::string &error_msg);

            // 프로토콜 변환 함수 (USB 패킷 캡처 분석 후 구현)
            std::vector<uint8_t> encode_led_command(LEDColor color, LEDPattern pattern);
            std::vector<uint8_t> encode_buzzer_command(BuzzerPattern pattern, int volume, int count);
            std::vector<uint8_t> encode_combined_command(LEDColor color, LEDPattern led_pattern,
                                                         BuzzerPattern buzzer_pattern, int volume, int count);

            // Non-copyable
            PatliteLedBuzzerUsbDriver(const PatliteLedBuzzerUsbDriver &) = delete;
            PatliteLedBuzzerUsbDriver &operator=(const PatliteLedBuzzerUsbDriver &) = delete;
        };

        // ============================================
        // 편의 함수: 단일 명령 실행
        // ============================================
        bool execute_patlite_command(const PatliteLedBuzzerCommand &command);

    } // namespace patlite_led_buzzer
} // namespace fta_actuators

#endif  // FTA_ACTUATORS__PATLITE_LED_BUZZER_USB_DRIVER_HPP_

