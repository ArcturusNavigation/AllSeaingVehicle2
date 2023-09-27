from launch import LaunchDescription
import launch_ros

def generate_launch_description():
   
    return LaunchDescription([
        launch_ros.actions.SetParameter(name="lat_origin", value=-33.7228),
        launch_ros.actions.SetParameter(name="lon_origin", value=150.6740),

        launch_ros.actions.Node(
	        package='allseaing_main', executable='state_reporter', output='screen'),
        launch_ros.actions.Node(
	        package='protobuf_client', executable='protobuf_client_node', output='screen'),
        launch_ros.actions.Node(
	        package='protobuf_client', executable='message_parser', output='screen'),
        launch_ros.actions.Node(
	        package='protobuf_client', executable='message_sender', output='screen'),
    ])

