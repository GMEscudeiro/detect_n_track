from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    detect_n_track = Node(
        package='track_n_follow',
        executable='detection_track.py',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    infer_pos = Node(
        package='track_n_follow',
        executable='inference_position.py',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                description='Flag to enable use_sim_time'),
        detect_n_track,
        infer_pos
    ])