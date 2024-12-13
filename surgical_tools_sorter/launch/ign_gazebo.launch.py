import os
import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    # Set the environment variable for Ignition Gazebo resources
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = os.path.join(
        FindPackageShare('surgicalBot').find("surgical_tools_sorter"), 'models')


    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            get_package_share_directory('surgical_tools_sorter'),
            'worlds',
            'just_arm.sdf'
        ])}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(get_package_share_directory('surgical_tools_sorter'), 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
    sdf_file = os.path.join( get_package_share_directory('surgical_tools_sorter'),
                            'models','robot_arm',
                            'eb100.sdf')

    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Return the launch description with all actions
    return launch.LaunchDescription([
        gz_sim,
        bridge,
        robot_state_publisher
    ])
