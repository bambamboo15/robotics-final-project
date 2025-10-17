from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package paths
    pkg_sim = get_package_share_directory('line_follower_sim')
    pkg_robot = get_package_share_directory('line_follower_robot')

    # World and models
    world_file = os.path.join(pkg_sim, 'worlds', 'scene.sdf')
    models_path = os.path.join(pkg_sim, 'models')

    # Include the robot launch file
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_robot, 'launch', 'robot.launch.py')
        )
    )

    return LaunchDescription([
        # Set Ignition resource path for models
        SetEnvironmentVariable(
            'IGN_GAZEBO_RESOURCE_PATH',
            models_path
        ),

        # Launch Ignition Gazebo with the world
        ExecuteProcess(
            cmd=['ign', 'gazebo', '-v4', world_file],
            output='screen'
        ),

        # Launch robot_state_publisher from the robot package
        robot_launch,

        # Spawn the robot after a short delay to ensure /robot_description is published
        TimerAction(
            period=2.0,
            actions=[Node(
                package='ros_gz_sim',
                executable='create',
                output='screen',
                arguments=[
                    '-name', 'lfrobot',
                    '-topic', 'robot_description',
                    '-x', '-2.375',
                    '-y', '0.0',
                    '-z', '0.35'
                ]
            )]
        ),
    ])
