from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': '/home/pes2ug23cs505/ros2_ws/src/fire_sim/worlds/fire_world.world'
        }.items()
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'drone',
            '-file', '/home/pes2ug23cs505/ros2_ws/src/fire_sim/urdf/drone_camera.urdf'
        ],
        output='screen'
    )

    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn]
    )

    return LaunchDescription([
        gazebo,
        delayed_spawn
    ])
