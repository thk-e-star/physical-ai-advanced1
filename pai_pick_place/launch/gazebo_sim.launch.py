import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('pai_pick_place')
    world_path = os.path.join(pkg_share, 'worlds', 'simple_scene.world')
    urdf_path = '/workspace/ros2_ws/src/pai_pick_place/urdf/simple_arm.urdf'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'simple_arm', '-file', urdf_path, '-x', '0.0', '-y', '0.0', '-z', '0.05'],
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        )
    ])
