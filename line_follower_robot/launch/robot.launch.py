from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    pkg_share = get_package_share_directory('line_follower_robot')

    # Process XACRO into URDF string
    xacro_file = os.path.join(pkg_share, 'urdf', 'simple_robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = robot_description_config.toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}]
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            arguments=[
                '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'
            ]
        ),
        Node(
            package='line_follower_robot',
            executable='simple_move',
            name='simple_move_node',
            remappings=[('cmd_vel', '/cmd_vel')]
        ),
        
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='left_camera_bridge',
            arguments=[
                '/left_camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image'
            ]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='middle_camera_bridge',
            arguments=[
                '/middle_camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image'
            ]
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='right_camera_bridge',
            arguments=[
                '/right_camera/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image'
            ]
        ),
        Node(
            package='line_follower_robot',
            executable='tape_detector',
            name='tape_detector_node'
        )
    ])
