import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'ethiopia_travel'
    pkg_share = get_package_share_directory(pkg_name)

    # 1. World File Path
    world_file = os.path.join(pkg_share, 'worlds', 'ethiopia.world')

    # 2. URDF File Path
    urdf_file = os.path.join(pkg_share, 'urdf', 'travel_bot.urdf')

    # 3. Start Gazebo Server (Headless)
    start_gazebo_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_file, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # 4. Start Gazebo Client (GUI)
    start_gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    # 5. Spawn Robot Node
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        # x=5.0, y=5.0 puts it in empty space (hopefully)
        # z=1.0 drops it from the air so it lands safely
        arguments=['-entity', 'travel_bot', '-file', urdf_file, '-x', '5.0', '-y', '5.0', '-z', '1.0'],
        output='screen'
    )

    # 6. Robot State Publisher (Required for TF/Sensors)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}]
    )

    return LaunchDescription([
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher,
        spawn_entity,
    ])