from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    FTE 없이 LED/Buzzer를 테스트하기 위한 Launch 파일
    - led_buzzer_node: 실제 디바이스 제어
    - test_action_publisher: 5초마다 테스트 액션 발행
    """
    
    # LED/Buzzer 노드
    led_buzzer_node = Node(
        package='fta_actuators',
        executable='led_buzzer_node',
        name='led_buzzer_node',
        output='screen',
        parameters=[{
            'enabled': True,
            'volume': 50,
        }]
    )
    
    # 테스트 액션 발행 노드
    test_publisher = Node(
        package='fta_actuators',
        executable='test_action_publisher',
        name='test_action_publisher',
        output='screen'
    )
    
    return LaunchDescription([
        led_buzzer_node,
        test_publisher,
    ])
