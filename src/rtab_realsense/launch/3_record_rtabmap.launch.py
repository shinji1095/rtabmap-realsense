from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

###################
output_filename = "sample1.bag"
###################

def generate_launch_description():
    record_topics = [
        '/rtabmap/mapData',          
        '/rtabmap/mapGraph',         
        '/rtabmap/odom',             
        '/rtabmap/odom_info',        
        '/rtabmap/local_grid_obstacles',  
        '/rtabmap/local_grid_empty',      
        '/rtabmap/local_grid_ground',    
        '/tf',                          
        '/tf_static'                    
    ]

    # RTAB-Map ノード
    rtabmap_node = Node(
        package='rtabmap_ros',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[
            {'subscribe_depth': True},
            {'subscribe_rgbd': True},
            {'subscribe_imu': True}
        ],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('depth/image', '/camera/depth/image_rect_aligned'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('imu', '/camera/imu')
        ]
    )

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', "../bag/rtabmap/"+output_filename] + record_topics,
        output='screen'
    )

    return LaunchDescription([
        rtabmap_node,
        rosbag_record
    ])