import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('paxini_tactile')
    urdf_file = os.path.join(pkg_share, 'urdf', 'amazing_hand.urdf')
    use_tactile = LaunchConfiguration('use_tactile')
    grasp_angle_deg = LaunchConfiguration('grasp_angle_deg')
    use_rviz = LaunchConfiguration('use_rviz')

    # 讀取 URDF 模型內容
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_tactile',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to launch and use the Paxini tactile sensor.',
        ),
        DeclareLaunchArgument(
            'grasp_angle_deg',
            default_value='90.0',
            description='Fixed finger grasp angle used without tactile sensing.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            choices=['true', 'false'],
            description='Whether to launch RViz.',
        ),


        # 1. 機器人狀態發布器 (解析 URDF + 發布 TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        # 2. Amazing Hand 馬達驅動節點
        Node(
            package='paxini_tactile',
            executable='amazing_hand_node',
            name='amazing_hand_node',
            output='screen',
            parameters=[{
                'port': '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5AE6056274-if00',
                'baudrate': 1000000,
                'servo_ids': [11, 12, 13, 14, 15, 16, 17, 18],
            }]
        ),

        # 3. Paxini 觸覺感測器節點
        Node(
            package='paxini_tactile',
            executable='paxini_node',
            name='paxini_node',
            output='screen',
            parameters=[{'port': '/dev/serial/by-id/usb-PaxiniAdapte3_Paxini__Adapter_83E567757710-if00', 'baudrate': 921600}],
            condition=IfCondition(use_tactile),
        ),

        # 4. 抓握控制器節點
        Node(
            package='paxini_tactile',
            executable='grasp_controller_node',
            name='grasp_controller_node',
            output='screen',
            parameters=[{
                'use_tactile': ParameterValue(use_tactile, value_type=bool),
                'grasp_angle_deg': ParameterValue(grasp_angle_deg, value_type=float),
                'force_threshold': 1.5,
                'middle_positions_deg': [3.0, -3.0, -1.0, -10.0, 5.0, 2.0, -7.0, 3.0],
            }]
        ),

        # 5. RViz2 視覺化
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
        )
    ])
