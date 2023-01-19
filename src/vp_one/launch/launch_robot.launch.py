import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='vp_one' #<--- CHANGE ME

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("odrive_demo_bringup"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("diffbot_description"),
            "config",
            "diffbot.rviz"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diffbot_base_controller", "-c", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    
    return LaunchDescription([
        #rsp,
        # control_node,
        # robot_state_pub_node,
        # joint_state_broadcaster_spawner,
        # robot_controller_spawner,
        # rviz_node
    ])