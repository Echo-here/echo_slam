from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('echo_slam')  # 패키지 이름 확인
    slam_nav2_params_file = os.path.join(pkg_share, 'config', 'slam_nav2_params.yaml')

    return LaunchDescription([
        # 1️⃣ SLAM launch 바로 실행
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("slam_toolbox"),
                    "launch",
                    "online_sync_launch.py"
                ])
            ]),
            launch_arguments={
                "use_sim_time": "False",
                "params_file": slam_nav2_params_file
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "navigation_launch.py"
                ])
            ]),
            launch_arguments={
                "use_sim_time": "False"
            }.items()
        ),
        # 3️⃣ map_utils_node 바로 실행
        Node(
            package='echo_slam',
            executable='mqtt_node',
            name='mqtt',
            output='screen'
        )
    ])
