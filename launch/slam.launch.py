from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('echo_slam')  # 사용자 패키지
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    return LaunchDescription([
        # 1️⃣ SLAM Toolbox 실행
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file],
        ),

        # 2️⃣ Nav2 bringup 실행 (SLAM과 연동)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "bringup_launch.py"
                ])
            ]),
            launch_arguments={
                "use_sim_time": "False",
                "slam": "False",  # SLAM Toolbox를 직접 실행하므로 False
                "params_file": nav2_params_file,
                "map": "",  # SLAM 사용 시 빈 문자열
            }.items()
        ),

        # 3️⃣ 추가 사용자 노드 (선택)
        Node(
            package='echo_slam',
            executable='map_utils_node.py',
            name='map_utils',
            output='screen'
        ),
    ])
