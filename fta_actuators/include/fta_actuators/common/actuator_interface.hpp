#ifndef FTA_ACTUATORS__COMMON__ACTUATOR_INTERFACE_HPP_
#define FTA_ACTUATORS__COMMON__ACTUATOR_INTERFACE_HPP_

#include <string>
#include <memory>

namespace fta_actuators
{

/**
 * @brief 모든 액추에이터가 구현해야 하는 기본 인터페이스
 * 
 * 이 인터페이스를 통해 PatliteLedBuzzer, Speaker, Battery 등
 * 다양한 액추에이터를 일관된 방식으로 제어할 수 있습니다.
 */
class ActuatorInterface
{
public:
  virtual ~ActuatorInterface() = default;

  /**
   * @brief 액추에이터 초기화
   * @return true 성공, false 실패
   */
  virtual bool initialize() = 0;

  /**
   * @brief 액추에이터 종료 (리소스 정리)
   */
  virtual void shutdown() = 0;

  /**
   * @brief 액추에이터가 정상 작동 중인지 확인
   * @return true 정상, false 오류
   */
  virtual bool is_healthy() const = 0;

  /**
   * @brief 액추에이터 이름 반환
   * @return 액추에이터 식별 문자열
   */
  virtual std::string get_name() const = 0;

  /**
   * @brief 액추에이터 타입 반환
   * @return 액추에이터 타입 문자열 (예: "patlite", "speaker", "battery")
   */
  virtual std::string get_type() const = 0;

  /**
   * @brief 현재 상태를 문자열로 반환
   * @return JSON 형식의 상태 정보
   */
  virtual std::string get_status() const = 0;

  /**
   * @brief 액추에이터 리셋
   * @return true 성공, false 실패
   */
  virtual bool reset() = 0;
};

}  // namespace fta_actuators

#endif  // FTA_ACTUATORS__COMMON__ACTUATOR_INTERFACE_HPP_
