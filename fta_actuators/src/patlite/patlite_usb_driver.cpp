// ============================================================================
// patlite_usb_driver.cpp
// Patlite USB LED/Buzzer 제어를 위한 USB 통신 계층 구현
//
// libusb-1.0을 사용하여 USB 장치 직접 제어
// ============================================================================

#include "fta_actuators/patlite/patlite_usb_driver.hpp"
#include <iostream>
#include <cstring>
#include <vector>

namespace fta_actuators
{
    namespace patlite
    {

        // ============================================
        // 생성자 / 소멸자
        // ============================================

        PatliteUsbDriver::PatliteUsbDriver()
            : context_(nullptr), device_handle_(nullptr)
        {
            device_info_ = {};
        }

        PatliteUsbDriver::~PatliteUsbDriver()
        {
            close_device();
        }

        // ============================================
        // USB 장치 초기화 및 연결
        // ============================================

        bool PatliteUsbDriver::open_device(uint16_t vendor_id, uint16_t product_id)
        {
            if (device_handle_ != nullptr)
            {
                set_error("Device already opened");
                return false;
            }

            if (!initialize_libusb())
            {
                return false;
            }

            if (!find_and_claim_device(vendor_id, product_id))
            {
                libusb_exit(context_);
                context_ = nullptr;
                return false;
            }

            std::cout << "[PatliteUsbDriver] Device opened successfully" << std::endl;
            return true;
        }

        void PatliteUsbDriver::close_device()
        {
            if (device_handle_ != nullptr)
            {
                libusb_release_interface(device_handle_, device_info_.interface_number);
                libusb_close(device_handle_);
                device_handle_ = nullptr;
            }

            if (context_ != nullptr)
            {
                libusb_exit(context_);
                context_ = nullptr;
            }

            std::cout << "[PatliteUsbDriver] Device closed" << std::endl;
        }

        bool PatliteUsbDriver::is_connected() const
        {
            return device_handle_ != nullptr;
        }

        // ============================================
        // LED 제어
        // ============================================

        bool PatliteUsbDriver::set_light(LEDColor color, LEDPattern pattern)
        {
            if (!is_connected())
            {
                set_error("Device not connected");
                return false;
            }

            std::vector<uint8_t> data = encode_led_command(color, pattern);
            return send_control_transfer(data.data(), data.size());
        }

        // ============================================
        // Buzzer 제어
        // ============================================

        bool PatliteUsbDriver::set_buzzer(BuzzerPattern pattern, int volume, int count)
        {
            if (!is_connected())
            {
                set_error("Device not connected");
                return false;
            }

            std::vector<uint8_t> data = encode_buzzer_command(pattern, volume, count);
            return send_control_transfer(data.data(), data.size());
        }

        // ============================================
        // 장치 상태 조회
        // ============================================

        bool PatliteUsbDriver::get_device_state(bool &buzzer_active, bool &led_active, bool &touch_active)
        {
            if (!is_connected())
            {
                set_error("Device not connected");
                return false;
            }

            // TODO: USB 패킷 캡처로 상태 조회 프로토콜 분석 후 구현
            uint8_t response[64] = {0};
            if (!receive_control_transfer(response, sizeof(response)))
            {
                return false;
            }

            // 임시 구현 (실제 프로토콜 분석 후 수정)
            buzzer_active = (response[0] & 0x01) != 0;
            led_active = (response[0] & 0x02) != 0;
            touch_active = (response[0] & 0x04) != 0;

            return true;
        }

        // ============================================
        // LED + Buzzer 동시 제어
        // ============================================

        bool PatliteUsbDriver::set_all(LEDColor color, LEDPattern led_pattern,
                                       BuzzerPattern buzzer_pattern, int volume, int count)
        {
            if (!is_connected())
            {
                set_error("Device not connected");
                return false;
            }

            std::vector<uint8_t> data = encode_combined_command(color, led_pattern, buzzer_pattern, volume, count);
            return send_control_transfer(data.data(), data.size());
        }

        // ============================================
        // 모두 끄기
        // ============================================
        bool PatliteUsbDriver::turn_off_all()
        {
            if (!is_connected())
            {
                set_error("Device not connected");
                return false;
            }

            // LED OFF + Buzzer OFF
            return set_all(LEDColor::CLEAR, LEDPattern::OFF, BuzzerPattern::OFF, 0, 0);
        }

        // ============================================
        // 장치 정보 가져오기
        // ============================================

        UsbDeviceInfo PatliteUsbDriver::get_device_info() const
        {
            return device_info_;
        }

        std::string PatliteUsbDriver::get_last_error() const
        {
            return last_error_;
        }

        // ============================================
        // 내부 헬퍼 함수
        // ============================================

        bool PatliteUsbDriver::initialize_libusb()
        {
            int ret = libusb_init(&context_);
            if (ret < 0)
            {
                set_error("Failed to initialize libusb: " + std::string(libusb_error_name(ret)));
                return false;
            }

            libusb_set_option(context_, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
            return true;
        }

        bool PatliteUsbDriver::find_and_claim_device(uint16_t vendor_id, uint16_t product_id)
        {
            libusb_device **device_list;
            ssize_t count = libusb_get_device_list(context_, &device_list);

            if (count < 0)
            {
                set_error("Failed to get device list");
                return false;
            }

            bool found = false;
            for (ssize_t i = 0; i < count; i++)
            {
                libusb_device *device = device_list[i];
                struct libusb_device_descriptor desc;

                if (libusb_get_device_descriptor(device, &desc) < 0)
                {
                    continue;
                }

                // Vendor ID 일치 확인
                if (desc.idVendor != vendor_id)
                {
                    continue;
                }

                // Product ID가 지정되었으면 확인
                if (product_id != 0x0000 && desc.idProduct != product_id)
                {
                    continue;
                }

                // 장치 열기
                int ret = libusb_open(device, &device_handle_);
                if (ret < 0)
                {
                    set_error("Failed to open device: " + std::string(libusb_error_name(ret)));
                    continue;
                }

                // 장치 정보 저장
                device_info_.vendor_id = desc.idVendor;
                device_info_.product_id = desc.idProduct;

                // 문자열 descriptor 읽기 (제조사, 제품명, 시리얼)
                unsigned char str_buffer[256];
                if (desc.iManufacturer > 0)
                {
                    libusb_get_string_descriptor_ascii(device_handle_, desc.iManufacturer, str_buffer, sizeof(str_buffer));
                    device_info_.manufacturer = std::string(reinterpret_cast<char *>(str_buffer));
                }
                if (desc.iProduct > 0)
                {
                    libusb_get_string_descriptor_ascii(device_handle_, desc.iProduct, str_buffer, sizeof(str_buffer));
                    device_info_.product = std::string(reinterpret_cast<char *>(str_buffer));
                }
                if (desc.iSerialNumber > 0)
                {
                    libusb_get_string_descriptor_ascii(device_handle_, desc.iSerialNumber, str_buffer, sizeof(str_buffer));
                    device_info_.serial_number = std::string(reinterpret_cast<char *>(str_buffer));
                }

                // 커널 드라이버 분리 (Linux에서 필요할 수 있음)
                device_info_.interface_number = 0;
                if (libusb_kernel_driver_active(device_handle_, device_info_.interface_number) == 1)
                {
                    libusb_detach_kernel_driver(device_handle_, device_info_.interface_number);
                }

                // 인터페이스 claim
                ret = libusb_claim_interface(device_handle_, device_info_.interface_number);
                if (ret < 0)
                {
                    set_error("Failed to claim interface: " + std::string(libusb_error_name(ret)));
                    libusb_close(device_handle_);
                    device_handle_ = nullptr;
                    continue;
                }

                found = true;
                break;
            }

            libusb_free_device_list(device_list, 1);

            if (!found)
            {
                set_error("Patlite device not found (Vendor ID: 0x" +
                          std::to_string(vendor_id) + ")");
                return false;
            }

            return true;
        }

        bool PatliteUsbDriver::send_control_transfer(const uint8_t *data, int length)
        {
            int ret = libusb_control_transfer(
                device_handle_,
                protocol_.request_type,
                protocol_.request,
                protocol_.value,
                protocol_.index,
                const_cast<uint8_t *>(data),
                length,
                protocol_.timeout_ms);

            if (ret < 0)
            {
                set_error("Control transfer failed: " + std::string(libusb_error_name(ret)));
                return false;
            }

            return true;
        }

        bool PatliteUsbDriver::receive_control_transfer(uint8_t *data, int length)
        {
            // TODO: USB 패킷 캡처로 GET 프로토콜 분석 후 구현
            int ret = libusb_control_transfer(
                device_handle_,
                LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
                0x01, // GET_REPORT
                0x0100,
                protocol_.index,
                data,
                length,
                protocol_.timeout_ms);

            if (ret < 0)
            {
                set_error("Control transfer (receive) failed: " + std::string(libusb_error_name(ret)));
                return false;
            }

            return true;
        }

        void PatliteUsbDriver::set_error(const std::string &error_msg)
        {
            last_error_ = error_msg;
            std::cerr << "[PatliteUsbDriver ERROR] " << error_msg << std::endl;
        }

        // ============================================
        // 프로토콜 인코딩 함수 (USB 패킷 캡처 후 구현)
        // ============================================

        std::vector<uint8_t> PatliteUsbDriver::encode_led_command(LEDColor color, LEDPattern pattern)
        {
            // TODO: USB 패킷 캡처로 실제 프로토콜 분석 후 구현
            // 임시 구현 (예시)
            std::vector<uint8_t> data;
            data.push_back(0x57);                          // 명령 헤더 (추정)
            data.push_back(static_cast<uint8_t>(color));   // LED 색상
            data.push_back(static_cast<uint8_t>(pattern)); // LED 패턴
            data.push_back(0x00);                          // 패딩
            data.push_back(0x00);                          // 패딩

            std::cout << "[PatliteUsbDriver] LED Command: color=" << static_cast<int>(color)
                      << " pattern=" << static_cast<int>(pattern) << std::endl;

            return data;
        }

        std::vector<uint8_t> PatliteUsbDriver::encode_buzzer_command(BuzzerPattern pattern, int volume, int count)
        {
            // TODO: USB 패킷 캡처로 실제 프로토콜 분석 후 구현
            std::vector<uint8_t> data;
            data.push_back(0x58);                          // 명령 헤더 (추정)
            data.push_back(static_cast<uint8_t>(pattern)); // Buzzer 패턴
            data.push_back(static_cast<uint8_t>(volume));  // 볼륨
            data.push_back(static_cast<uint8_t>(count));   // 반복 횟수
            data.push_back(0x00);                          // 패딩

            std::cout << "[PatliteUsbDriver] Buzzer Command: pattern=" << static_cast<int>(pattern)
                      << " volume=" << volume << " count=" << count << std::endl;

            return data;
        }

        std::vector<uint8_t> PatliteUsbDriver::encode_combined_command(
            LEDColor color, LEDPattern led_pattern,
            BuzzerPattern buzzer_pattern, int volume, int count)
        {
            // TODO: USB 패킷 캡처로 실제 프로토콜 분석 후 구현
            std::vector<uint8_t> data;
            data.push_back(0x59);                                 // 명령 헤더 (추정)
            data.push_back(static_cast<uint8_t>(color));          // LED 색상
            data.push_back(static_cast<uint8_t>(led_pattern));    // LED 패턴
            data.push_back(static_cast<uint8_t>(buzzer_pattern)); // Buzzer 패턴
            data.push_back(static_cast<uint8_t>(volume));         // 볼륨
            data.push_back(static_cast<uint8_t>(count));          // 반복 횟수

            std::cout << "[PatliteUsbDriver] Combined Command: LED(color=" << static_cast<int>(color)
                      << ", pattern=" << static_cast<int>(led_pattern)
                      << ") Buzzer(pattern=" << static_cast<int>(buzzer_pattern)
                      << ", volume=" << volume << ", count=" << count << ")" << std::endl;

            return data;
        }

        // ============================================
        // 편의 함수 구현
        // ============================================

        bool execute_patlite_command(const PatliteCommand &command)
        {
            static PatliteUsbDriver driver; // Singleton 패턴 (간단한 구현)
            static bool initialized = false;

            if (!initialized)
            {
                if (!driver.open_device())
                {
                    std::cerr << "Failed to open Patlite device: " << driver.get_last_error() << std::endl;
                    return false;
                }
                initialized = true;
            }

            return driver.set_all(
                command.led_color,
                command.led_pattern,
                command.buzzer_pattern,
                50, // 기본 볼륨
                command.buzzer_count);
        }

    } // namespace patlite
} // namespace fta_actuators
