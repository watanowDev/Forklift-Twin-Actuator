#ifndef FTA_ACTUATORS__PATLITE_LED_BUZZER_HARDWARE_INTERFACE_HPP_
#define FTA_ACTUATORS__PATLITE_LED_BUZZER_HARDWARE_INTERFACE_HPP_

#include "fta_actuators/patlite_led_buzzer/patlite_led_buzzer_types.hpp"
#include <string>
#include <memory>

namespace fta_actuators
{

/**
 * @brief PatliteLedBuzzer 하드웨어 제어 인터페이스
 * 
 * 제조사별 드라이버를 플러그인 형태로 교체할 수 있도록 추상화합니다.
 * 구현체:
 * - PatliteLedBuzzerUsbDriver: libusb 기반 직접 제어 (현재 구현)
 * - PatliteLedBuzzerNeDllDriver: NeUsbController.dll 래퍼 (추후 구현)
 * - PatliteLedBuzzerMockDriver: 테스트용 Mock (추후 구현)
 */
class PatliteLedBuzzerHardwareInterface
{
public:
  virtual ~PatliteLedBuzzerHardwareInterface() = default;

  /**
   * @brief 하드웨어 연결 및 초기화
   * @param device_path 디바이스 경로 (USB: "/dev/bus/usb/..." 또는 빈 문자열)
   * @return true 성공, false 실패
   */
  virtual bool open(const std::string& device_path = "") = 0;

  /**
   * @brief 하드웨어 연결 해제
   */
  virtual void close() = 0;

  /**
   * @brief 연결 상태 확인
   * @return true 연결됨, false 연결 안 됨
   */
  virtual bool is_connected() const = 0;

  /**
   * @brief LED 제어
   * @param color LED 색상
   * @param pattern LED 패턴
   * @return true 성공, false 실패
   */
  virtual bool set_led(LEDColor color, LEDPattern pattern) = 0;

  /**
   * @brief Buzzer 제어
   * @param pattern Buzzer 패턴
   * @param volume 볼륨 (0~10)
   * @param count 반복 횟수
   * @return true 성공, false 실패
   */
  virtual bool set_buzzer(BuzzerPattern pattern, int volume, int count) = 0;

  /**
   * @brief LED와 Buzzer 동시 제어 (원자적 실행)
   * @param cmd PatliteLedBuzzer 명령
   * @param volume 볼륨 (0~10)
   * @return true 성공, false 실패
   */
  virtual bool execute_command(const PatliteLedBuzzerCommand& cmd, int volume) = 0;

  /**
   * @brief 디바이스 상태 조회
   * @param buzzer_state Buzzer 작동 상태 (출력)
   * @param led_state LED 작동 상태 (출력)
   * @param touch_state 터치 센서 상태 (출력)
   * @return true 성공, false 실패
   */
  virtual bool get_device_state(
    bool& buzzer_state,
    bool& led_state,
    bool& touch_state) const = 0;

  /**
   * @brief 디바이스 정보 반환
   * @return 제조사, 모델명, 버전 등의 정보
   */
  virtual std::string get_device_info() const = 0;

  /**
   * @brief 드라이버 이름 반환
   * @return 드라이버 식별 문자열 (예: "NeUsbController", "libusb", "mock")
   */
  virtual std::string get_driver_name() const = 0;
};

/**
 * @brief 하드웨어 인터페이스 팩토리
 * 
 * 설정에 따라 적절한 드라이버를 생성합니다.
 */
class PatliteLedBuzzerHardwareFactory
{
public:
  enum class DriverType
  {
    USB_DIRECT,    // libusb 기반 직접 제어 (기본값)
    NE_DLL,        // NeUsbController.dll 래퍼
    MOCK           // 테스트용 Mock
  };

  /**
   * @brief 드라이버 생성
   * @param type 드라이버 타입
   * @return 생성된 드라이버 인스턴스
   */
  static std::unique_ptr<PatliteLedBuzzerHardwareInterface> create_driver(
    DriverType type = DriverType::USB_DIRECT);

  /**
   * @brief 문자열로부터 드라이버 타입 변환
   * @param type_str "usb", "dll", "mock"
   * @return 드라이버 타입
   */
  static DriverType parse_driver_type(const std::string& type_str);
};

}  // namespace fta_actuators

#endif  // FTA_ACTUATORS__PATLITE_LED_BUZZER_HARDWARE_INTERFACE_HPP_

