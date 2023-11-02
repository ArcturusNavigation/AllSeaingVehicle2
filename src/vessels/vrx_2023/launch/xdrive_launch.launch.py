from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros

def generate_launch_description():
    vrx_gz_prefix = get_package_share_directory("vrx_gz") 
    vrx_2023_prefix = get_package_share_directory("vrx_2023") 

    return LaunchDescription([
        launch_ros.actions.Node(
	        package="allseaing_main", 
            executable="state_reporter",
            remappings=[
                ("/imu/data", "/wamv/sensors/imu/imu/data"),
            ]
        ),
        launch_ros.actions.Node(
            package="vrx_2023",
            executable="xdrive_controller.py",
            output="screen"
        ),
        launch_ros.actions.Node(
            package="vrx_2023",
            executable="pwm_subscriber.py",
            output="screen",
        ),

        # wayfinding
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([vrx_gz_prefix, "/launch/competition.launch.py"]),
            launch_arguments = {"world": "sydney_regatta", "urdf": f"{vrx_2023_prefix}/resource/wamv_target.urdf"}.items()),
    ])

