from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['gz', 'sim', '-v', '4', PathJoinSubstitution([
                get_package_share_directory('scout_gz_sim'),
                'worlds',
                'empty_world.sdf'
            ])],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': PathJoinSubstitution([
                    get_package_share_directory('scout_desc'),
                    'urdf',
                    'diff_drive.urdf.xacro'
                ])
            }]
        ),

        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gz_ros2_control', 'spawn_entity.py',
                '-entity', 'mybot',
                '-topic', '/robot_description'
            ],
            output='screen'
        )
    ])
