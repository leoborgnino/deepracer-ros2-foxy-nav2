from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
	    package='waypoint_follower_pkg',
	    namespace='waypoint_follower_pkg',
	    executable='waypoint_follower_node',
	    output='screen',
	    name='waypoint_follower_node'
	)
    ])
