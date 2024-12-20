from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

###############

output_filename = "sample1.bag"

###############

def generate_launch_description():
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense_camera',
        output='screen',
        parameters=[
            {'align_depth.enable': True}
        ]
    )

    record_topics = [
        '/camera/color/image_raw',
        '/camera/depth/image_rect_aligned',
        '/camera/infra1/image_rect_raw',
        '/camera/infra2/image_rect_raw',
        '/camera/color/camera_info',
        '/camera/depth/camera_info',
        '/camera/gyro/imu_info',
        '/camera/gyro/sample', 
        '/camera/accel/imu_info', 
        '/camera/accel/sample', 
        '/tf',
        '/tf_static'
    ]

    record_process = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-o', '../bag/record/' + output_filename] + record_topics,
        output='screen'
    )

    return LaunchDescription([
        realsense_node,
        record_process
    ])
