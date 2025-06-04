import os
import xacro
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_name = 'mybot_description'  # your package name
    pkg_dir = get_package_share_directory(pkg_name)

    # Paths
    xacro_file = os.path.join(pkg_dir, 'urdf', 'mybot.urdf.xacro')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'mybot.generated.urdf')
    world_file = os.path.join(pkg_dir, 'worlds', 'empty.sdf')  # make this if it doesn't exist

    # Generate URDF from Xacro
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    os.makedirs(os.path.dirname(urdf_file), exist_ok=True)
    with open(urdf_file, 'w') as f:
        f.write(robot_description_raw)

    # Env vars for GZ Harmonic
    env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
            os.path.join(get_package_share_directory('ros_gz_sim'), 'lib')
        ]),
        'GZ_SIM_RESOURCE_PATH': ':'.join([
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
            pkg_dir
        ])
    }

    return LaunchDescription([
        # Start Gazebo
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_file],
            output='screen',
            additional_env=env,
        ),

        # Start robot_state_publisher
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='rsp',
                    output='screen',
                    parameters=[{'robot_description': robot_description_raw}]
                )
            ]
        ),

        # Spawn robot into Gazebo
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'gz', 'service', '-s', '/world/empty/create',
                        '--reqtype', 'gz.msgs.EntityFactory',
                        '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '2000',
                        '--req', f'sdf_filename: "{urdf_file}", name: "mybot", pose: {{position: {{x: 0.0, y: 0.0, z: 0.1}}}}'
                    ],
                    output='screen'
                )
            ]
        ),

        # Spawner for joint_state_broadcaster
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                    output='screen'
                )
            ]
        ),

        # Spawner for diff drive controller
        TimerAction(
            period=9.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
                    output='screen'
                )
            ]
        )
    ])
