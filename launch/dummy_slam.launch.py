from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('echo_slam')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    return LaunchDescription([
        # Node(
        #     package='echo_slam',        # ← dummy_odom_node.py가 속한 패키지명으로 수정
        #     executable='dummy_odom_node',       # ← setup.py에서 entry point 이름
        #     name='dummy_odom_node',
        #     output='screen'
        # ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file],
        )
    ])