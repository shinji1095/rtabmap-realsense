from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

###############
bag_file_path = 'sample1.bag'
###############

def generate_launch_description():

    rosbag_play = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', bag_file_path],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '../config/rviz.rviz']
    )

    return LaunchDescription([
        rosbag_play,
        rviz_node
    ])
