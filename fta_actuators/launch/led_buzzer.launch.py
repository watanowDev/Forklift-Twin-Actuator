from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 패키지 경로
    pkg_dir = get_package_share_directory('fta_actuators')
    
    # Config 파일 경로
    config_file = os.path.join(pkg_dir, 'config', 'led_buzzer.yaml')
    
    # Launch 인자
    enabled_arg = DeclareLaunchArgument(
        'enabled',
        default_value='true',
        description='Enable LED/Buzzer node'
    )
    
    volume_arg = DeclareLaunchArgument(
        'volume',
        default_value='50',
        description='Buzzer volume (0-100)'
    )
    
    # LED/Buzzer 노드
    led_buzzer_node = Node(
        package='fta_actuators',
        executable='led_buzzer_node',
        name='led_buzzer_node',
        output='screen',
        parameters=[{
            'enabled': LaunchConfiguration('enabled'),
            'volume': LaunchConfiguration('volume'),
        }]
    )
    
    return LaunchDescription([
        enabled_arg,
        volume_arg,
        led_buzzer_node,
    ])
