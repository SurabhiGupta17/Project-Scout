from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory('scout_desc'),
        'urdf',
        'diff_drive.urdf.xacro'
    )

    robot_description = Command(['xacro ', urdf_file])

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        # ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(
                get_package_share_directory('scout_desc'),
                'rviz',
                'diff_drive.config'
            )]
        )
    ])
