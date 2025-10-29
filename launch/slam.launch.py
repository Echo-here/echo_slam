from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('echo_slam')  # 패키지 이름 확인
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_params.yaml')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    slam_nav2_params_file = os.path.join(pkg_share, 'config', 'slam_nav2_params.yaml')
    nav2_map = os.path.join(pkg_share, 'map', 'map.yaml')

    return LaunchDescription([
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
        ),
        Node(
            package='echo_slam',
            executable='map_utils_node',
            name='map_utils',
            output='screen'
        )
        # 1️⃣ SLAM Toolbox
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='slam_toolbox',
        #     output='screen',
        #     parameters=[slam_params_file],
        # ),

        # # 2️⃣ Nav2 Controller Server
        # Node(
        #     package='nav2_controller',
        #     executable='controller_server',
        #     name='controller_server',
        #     output='screen',
        #     parameters=[nav2_params_file]
        # ),

        # # 3️⃣ Nav2 Planner Server
        # Node(
        #     package='nav2_planner',
        #     executable='planner_server',
        #     name='planner_server',
        #     output='screen',
        #     parameters=[nav2_params_file]
        # ),

        # # 4️⃣ Nav2 BT Navigator
        # Node(
        #     package='nav2_bt_navigator',
        #     executable='bt_navigator',
        #     name='bt_navigator',
        #     output='screen',
        #     parameters=[nav2_params_file]
        # ),

        # # 5️⃣ Nav2 Lifecycle Manager
        # Node(
        #     package='nav2_lifecycle_manager',
        #     executable='lifecycle_manager',
        #     name='lifecycle_manager',
        #     output='screen',
        #     parameters=[{
        #         'use_sim_time': False,
        #         'autostart': True,
        #         'node_names': ['controller_server', 'planner_server', 'bt_navigator']
        #     }]
        # )
    ])
