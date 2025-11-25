from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    FTE ���� LED/Buzzer�� �׽�Ʈ�ϱ� ���� Launch ����
    - led_buzzer_node: ���� ����̽� ����
    - test_action_publisher: 5�ʸ��� �׽�Ʈ �׼� ����
    """
    
    # LED/Buzzer ���
    led_buzzer_node = Node(
        package='fta_actuators',
        executable='led_buzzer_node',
        name='led_buzzer_node',
        output='screen',
        parameters=[{
            'enabled': True,
            'volume': 5,  # Range: 0-10
        }],
    )
    
    # �׽�Ʈ �׼� ���� ���
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
