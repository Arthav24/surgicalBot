import os
import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    # Set the environment variable for Ignition Gazebo resources
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = os.path.join(
        FindPackageShare('surgicalBot').find("surgical_tools_sorter"), 'models')

    # Launch Ignition Gazebo with the specified world
    ign_gazebo_world = ExecuteProcess(
        cmd=[
            'ign', 'gazebo',  # Change this line
            PathJoinSubstitution([
                FindPackageShare('surgical_tools_sorter'),
                'worlds',
                'tool_sorter_world.sdf'
            ]),
            '--render-engine', 'ogre2',  # Set the rendering engine to Ogre
            '-v', '4'  # Set verbosity level to 4 for more detailed logs
        ],
        shell=True
    )

    # Launch RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
    )

    # Launch the ROS 2 to Ignition bridge for the camera topic
    cam_topic_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@ignition.msgs.Image'],
    )

    # Example of a robot controller node (uncomment and configure as needed)
    # controller_node = Node(
    #     package='controller_manager',
    #     executable='controller_spawner',
    #     arguments=['your_controller_configurations_here'],
    # )

    # Return the launch description with all actions
    return launch.LaunchDescription([
        ign_gazebo_world,
        cam_topic_bridge,
        # rviz_node,
        # controller_node  # Uncomment and configure if you want to launch a controller node
    ])