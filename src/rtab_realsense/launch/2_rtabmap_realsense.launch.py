from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        output='screen',
        parameters=[
            {'align_depth.enable': True},           
            {'enable_gyro': True},                 
            {'enable_accel': True},                 
            {'enable_color': True},                 
            {'enable_depth': True}                 
        ]
    )

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

    return LaunchDescription([
        realsense_node,
        rtabmap_node
    ])
