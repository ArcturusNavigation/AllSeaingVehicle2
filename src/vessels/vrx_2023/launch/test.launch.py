from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
import launch_ros

def generate_launch_description():

    robot_localization_params = os.path.join(get_package_share_directory("vrx_2023"), 'params', 'dual_ekf_navsat.yaml')

    return LaunchDescription([
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[robot_localization_params]),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            remappings=[
                ('/gps/fix', '/wamv/sensors/gps/gps/fix'),
            ],
            parameters=[robot_localization_params]),

        launch_ros.actions.Node(
	        package='allseaing_main', 
            executable='state_reporter', 
            output='screen',
            remappings=[
                ('/imu/data', '/wamv/sensors/imu/imu/data'),
                ('/gps/fix', '/wamv/sensors/gps/gps/fix'),
            ]),
        launch_ros.actions.Node(
	        package='protobuf_client', executable='protobuf_client_node', output='screen'),
        launch_ros.actions.Node(
	        package='protobuf_client', executable='message_parser', output='screen'),
        launch_ros.actions.Node(
	        package='protobuf_client', executable='message_sender', output='screen'),
    ])

