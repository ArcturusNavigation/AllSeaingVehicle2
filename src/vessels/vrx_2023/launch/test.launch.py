from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros

def generate_launch_description():

    robot_localization_params = os.path.join(get_package_share_directory("vrx_2023"), "params", "dual_ekf_navsat.yaml")

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
            executable="navsat_transform_node",
            name="navsat_transform_node",
            output="screen",
            remappings=[("/gps/fix", "/wamv/sensors/gps/gps/fix")],
            parameters=[robot_localization_params]),

        # controller
        launch_ros.actions.Node(
            package="controller_suite",
            executable="simple_controller.py",
            output="screen",
            remappings=[
                ("/left_thrust", "/wamv/thrusters/left/thrust"),
                ("/right_thrust", "/wamv/thrusters/right/thrust")],
            parameters=[
                {"linear_scaling": 25.0},
                {"angular_scaling": 15.0},
                {"lower_thrust_limit": -1000.0},
                {"upper_thrust_limit": 1000.0}]),

        # state reporter
        launch_ros.actions.Node(
	        package="allseaing_main", 
            executable="state_reporter", 
            output="screen",
            remappings=[
                ("/imu/data", "/wamv/sensors/imu/imu/data"),
                ("/gps/fix", "/wamv/sensors/gps/gps/fix")]),

        # MOOS-ROS bridge
        launch_ros.actions.Node(
	        package="protobuf_client", executable="protobuf_client_node", output="screen"),
        launch_ros.actions.Node(
	        package="protobuf_client", executable="message_parser", output="screen"),
        launch_ros.actions.Node(
	        package="protobuf_client", executable="message_sender", output="screen"),
    ])

