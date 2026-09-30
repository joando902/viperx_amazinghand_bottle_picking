"""Run continuous AprilTag detection for the VX300s RealSense camera."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('apriltag_ros'),
        'config',
        'vx300s_tags.param.yaml',
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare('apriltag_ros'),
        'config',
        'vx300s_apriltag.rviz',
    ])

    config_file = LaunchConfiguration('config_file')
    image_topic = LaunchConfiguration('image_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    tag_id = LaunchConfiguration('tag_id')
    tag_size = LaunchConfiguration('tag_size')
    use_rviz = LaunchConfiguration('use_rviz')

    detector = Node(
        package='apriltag_ros',
        executable='apriltag_ros_continuous_detector_node',
        namespace='apriltag',
        name='detector',
        output='screen',
        parameters=[
            config_file,
            {
                'standalone_tags.tag_0.id': ParameterValue(tag_id, value_type=int),
                'standalone_tags.tag_0.size': ParameterValue(tag_size, value_type=float),
            },
        ],
        remappings=[
            ('~/image_rect', image_topic),
            ('~/camera_info', camera_info_topic),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='apriltag_rviz',
        output='screen',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='AprilTag detector parameter file.',
        ),
        DeclareLaunchArgument(
            'image_topic',
            default_value='/camera/camera/color/image_raw',
            description='Rectified or distortion-free camera image topic.',
        ),
        DeclareLaunchArgument(
            'camera_info_topic',
            default_value='/camera/camera/color/camera_info',
            description='CameraInfo topic paired with image_topic.',
        ),
        DeclareLaunchArgument(
            'tag_id',
            default_value='0',
            description='ID encoded by the printed tag.',
        ),
        DeclareLaunchArgument(
            'tag_size',
            default_value='0.040',
            description='Black/white boundary side length in metres.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 with TF and detection-image displays.',
        ),
        detector,
        rviz,
    ])

