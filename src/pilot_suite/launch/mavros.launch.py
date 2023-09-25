from launch import LaunchDescription
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('pilot_suite'),
        'config',
        'params.yaml'
    )
    
    return LaunchDescription([
        launch_ros.actions.SetParameter(name="fcu_url", value="/dev/ttyACM0"),
        launch_ros.actions.Node(
	        package='mavros', executable='mavros_node', output='screen'),
    ])