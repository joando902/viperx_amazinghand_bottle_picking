"""
This script makes the end-effector go to a specific pose by defining the pose components

To get started, open a terminal and type:

    ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300s

Then change to this directory and type:

    python3 ee_control.py
"""


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
from interbotix_common_modules.common_robot.robot import robot_startup, robot_shutdown
import tf_transformations

class EEInteractiveMarker(Node):
    def __init__(self):
        super().__init__('ee_interactive_marker')
        self.bot = InterbotixManipulatorXS(
            robot_model='vx300s',
            group_name='arm',
            gripper_name='gripper',
        )
        robot_startup()
        self.server = InteractiveMarkerServer(self, "ee_marker")
        self.make_6dof_marker()

    def make_6dof_marker(self):
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = "vx300s/base_link"
        int_marker.name = "ee_goal"
        int_marker.description = "Move End Effector"
        int_marker.scale = 0.2
        int_marker.pose.position.x = 0.2
        int_marker.pose.position.y = 0.0
        int_marker.pose.position.z = 0.2

        # 6-DOF controls
        for axis, orientation in zip(['x', 'y', 'z'], [(1,0,0), (0,1,0), (0,0,1)]):
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = float(orientation[0])
            control.orientation.y = float(orientation[1])
            control.orientation.z = float(orientation[2])
            control.name = f"rotate_{axis}"
            control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
            int_marker.controls.append(control)
            control = InteractiveMarkerControl()
            control.orientation.w = 1.0
            control.orientation.x = float(orientation[0])
            control.orientation.y = float(orientation[1])
            control.orientation.z = float(orientation[2])
            control.name = f"move_{axis}"
            control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
            int_marker.controls.append(control)

        self.server.insert(int_marker)
        self.server.setCallback(int_marker.name, self.process_feedback)
        self.server.applyChanges()

    def process_feedback(self, feedback):
        pose = feedback.pose
        # 取得 roll, pitch, yaw
        quat = (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w
        )
        roll, pitch, yaw = tf_transformations.euler_from_quaternion(quat)
        self.get_logger().info(f"Move to x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}, roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f}")
        self.bot.arm.set_ee_pose_components(
            x=pose.position.x,
            y=pose.position.y,
            z=pose.position.z,
            roll=roll,
            pitch=pitch,
            yaw=yaw
        )

def main(args=None):
    rclpy.init(args=args)
    node = EEInteractiveMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        robot_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()