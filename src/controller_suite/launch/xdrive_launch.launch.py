from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os

# sample launch file to run the sydney regatta sim with an xdrive boat and controller

def generate_launch_description():
    vrx_gz_prefix = get_package_share_directory("vrx_gz") 
    vrx_2023_prefix = get_package_share_directory("vrx_2023") 
    robot_localization_params = os.path.join(get_package_share_directory("vrx_2023"), "params", "dual_ekf_navsat.yaml")

    return LaunchDescription([
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
            output="screen",
            parameters=[robot_localization_params]),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="navsat_transform_node",
            name="navsat_transform_node",
            output="screen",
            remappings=[("/gps/fix", "/wamv/sensors/gps/gps/fix")],
            parameters=[robot_localization_params]),
        launch_ros.actions.Node(
            package="vrx_2023",
            executable="xdrive_controller.py"
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
            launch_arguments = {"world": "sydney_regatta", "urdf": f"{vrx_2023_prefix}/resource/wamv_target.urdf"}.items()),
    ])

