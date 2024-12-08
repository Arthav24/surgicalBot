import os
import launch
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    os.environ["IGN_GAZEBO_RESOURCE_PATH"] = os.path.join(
        FindPackageShare('surgicalBot').find("surgical_tools_sorter"), 'models')

    ign_gazebo_world = ExecuteProcess(
        cmd=[[
            'ign gazebo ',
            PathJoinSubstitution([
                FindPackageShare('surgical_tools_sorter'),
                'worlds',
                'tool_sorter_world.sdf '
            ]),
            '--render-engine ogre ',
            '-v 4 '
        ]],
        shell=True
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
    )
    
    cam_topic_bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@ignition.msgs.Image'],
    )
    
    # controller_node = Node( 
    # )
    
    return launch.LaunchDescription([
        ign_gazebo_world,
        cam_topic_bridge,
        rviz_node,
        # controller_node
    ])