from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import subprocess

# 🧹 Kill any running slam_toolbox instance
subprocess.run(["pkill", "-f", "slam_toolbox"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def generate_launch_description():
    # === Package paths ===
    bringup_dir = get_package_share_directory('nav2_bringup')
    my_bot_dir = get_package_share_directory('my_bot')  # 👈 Replace with your robot package

    # === Launch files ===
    localization_launch = os.path.join(bringup_dir, 'launch', 'localization_launch.py')
    navigation_launch = os.path.join(bringup_dir, 'launch', 'navigation_launch.py')

    # === Map file (absolute path) ===
    map_file = os.path.abspath(os.path.join(os.getcwd(), 'my_map_save.yaml'))

    # === RViz configuration ===
    rviz_config = os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz')

    # === URDF path ===
    # Replace 'my_bot.urdf' with your actual robot file name
    urdf_file = os.path.join(my_bot_dir, 'urdf', 'my_bot.urdf')

    # === Robot State Publisher ===
    # Publishes /robot_description → allows RViz to show RobotModel when 2D Pose Estimate is set
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[urdf_file]
    )

    # === RViz Node ===
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}]
    )

    # === Localization ===
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'true'
        }.items()
    )

    # === Navigation ===
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch),
        launch_arguments={
            'use_sim_time': 'true',
            'map_subscribe_transient_local': 'true'
        }.items()
    )

    # === Launch all ===
    return LaunchDescription([
        robot_state_publisher,  # 👈 makes robot model appear when 2D Pose Estimate used
        localization,
        navigation,
        rviz_node
    ])
