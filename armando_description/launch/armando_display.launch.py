from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    declared_arguments=[]
    # Percorso al file URDF
    pkg_share = FindPackageShare('armando_description')
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'arm.urdf.xacro'])

    # Parametro: file URDF da usare
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file", 
            default_value=PathJoinSubstitution(
                [FindPackageShare("armando_description"), "config", "prova1.rviz"]
            ),
            description="RViz config file (absolute path) to use when launching rviz.",
        )
    )
    
    ### Xacro routine ###
    #xacro_armando = Command(['xacro ', urdf_file])
    #robot_description = {"robot_description": xacro_armando}
  

    # Nodo robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
        #parameters=[robot_description, {"use_sim_time": True }]
    )

    # Nodo joint_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
        #parameters=[robot_description, {"use_sim_time": True }] to control both in gazebo and rviz
    )


    # Nodo RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
