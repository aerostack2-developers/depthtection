from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='quadrotor_1'),
        DeclareLaunchArgument('camera_topic', default_value='slot0'),
        DeclareLaunchArgument('detection_topic', default_value='detector_node/detections'),
        DeclareLaunchArgument('ground_truth_topic', default_value='""'),
        DeclareLaunchArgument('base_frame', default_value='quadrotor_1'),
        DeclareLaunchArgument('show_detection', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='depthtection',
            executable='depthtection_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{'namespace ': LaunchConfiguration('namespace')},
                        {'camera_topic': LaunchConfiguration('camera_topic')},
                        {'detection_topic': LaunchConfiguration('detection_topic')},
                        {'ground_truth_topic': LaunchConfiguration('ground_truth_topic')},
                        {'base_frame': LaunchConfiguration('base_frame')},
                        {'show_detection': LaunchConfiguration('show_detection')},
                        {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen',
            emulate_tty=True
        )
    ])
