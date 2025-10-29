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
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "slam_launch.py"
                ])
            ]),
            launch_arguments={
                "use_sim_time": "False",
                "autostart": "True",
                "params_file": slam_nav2_params_file,
                "map": ""
            }.items()
        ),

        # 2️⃣ Nav2 bringup은 10초 delay 후 실행
        TimerAction(
            period=10.0,  # 10초 delay
            actions=[
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
                        "map": "/"
                    }.items()
                )
            ]
        ),

        # 3️⃣ map_utils_node 바로 실행
        Node(
            package='echo_slam',
            executable='map_utils_node',
            name='map_utils',
            output='screen'
        )
    ])
