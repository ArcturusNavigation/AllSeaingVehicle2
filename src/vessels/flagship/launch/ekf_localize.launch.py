from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros

def generate_launch_description():

    robot_localization_params = os.path.join(get_package_share_directory("flagship"), "params", "dual_ekf_navsat.yaml")

    return LaunchDescription([
        # robot localization
        launch_ros.actions.Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[robot_localization_params]),
        launch_ros.actions.Node(
            package="robot_localization",
            executable="navsat_transfrom_node",
            name="navsat_transform_node",
            output="screen",
            remappings=[("/gps/fix", "/mavros/gpsstatus/gps1/raw")],
            parameters=[robot_localization_params]),
    ])

