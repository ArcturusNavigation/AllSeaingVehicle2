from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
import launch_ros
import os

# sample launch file to run the sydney regatta sim with an xdrive boat and controller

def generate_launch_description():
    vrx_gz_prefix = get_package_share_directory("vrx_gz") 
    controller_suite_prefix = get_package_share_directory("controller_suite") 
    robot_localization_params = os.path.join(get_package_share_directory("controller_suite"), "params", "dual_ekf_navsat.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("in_sim", default_value=TextSubstitution(text="False")),
        DeclareLaunchArgument("with_control", default_value=TextSubstitution(text="True")),
        launch_ros.actions.Node(
	        package="allseaing_main", 
            executable="state_reporter",
            remappings=[
                ("/imu/data", "/wamv/sensors/imu/imu/data"),
                ("/gps/fix", "/wamv/sensors/gps/gps/fix")
            ]
        ),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            parameters=[robot_localization_params]),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            remappings=[("/gps/fix", "/wamv/sensors/gps/gps/fix")],
            parameters=[robot_localization_params]),
        launch_ros.actions.Node(
            package="controller_suite",
            executable="xdrive_controller.py",
            name="controller",
            parameters=[{"in_sim": LaunchConfiguration("in_sim")}],
            condition=IfCondition(LaunchConfiguration("with_control"))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
            launch_arguments = {"world": "sydney_regatta", "urdf": f"{controller_suite_prefix}/resource/wamv_target.urdf"}.items()),
    ])

