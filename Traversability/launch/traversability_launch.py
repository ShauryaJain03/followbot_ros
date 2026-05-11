from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='traversability_gridmap',
            executable='traversability_node',
            name='traversability_node',
            output ='screen',
            parameters=[
                {"pc_topic": "/lio_sam/mapping/cloud_registered"},
            	{"resolution": 0.2},
                {"half_size": 75.},
                {"security_distance": 0.15},
                {"max_slope": 0.6}, 
                {"ground_clearance": 0.20}, 
                {"robot_height": 1.15}, 
                {"robot_width": 1.04},
                {"robot_length": 1.25},
                {"draw_isodistance_each": 1.},
                {"frame_id":"map"},
                {"global_mapping":True},

            ],
        )
    ])
