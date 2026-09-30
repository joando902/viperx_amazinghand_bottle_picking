#!/usr/bin/env python3

"""Pick a YOLO-detected bottle with a VX300s and an Amazing Hand.

This node keeps arm motion in the Interbotix MoveIt interface, but replaces
the stock gripper commands with the AmazingHand_ROS2 task interface:

  /hand/task_cmd       std_msgs/String ("open", "grasp", or "stop")
  /hand/cmd_positions  std_msgs/Float32MultiArray (motion feedback)

The Amazing Hand grasp controller is responsible for tactile, per-finger
contact stopping.  Both ROS containers must use the same ROS_DOMAIN_ID.
"""

from collections import deque
import math
from threading import Event, Lock, Thread
import time

from geometry_msgs.msg import PointStamped, Pose
from interbotix_moveit_interface_msgs.srv import MoveItPlan
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray, String
from tf2_ros import Buffer, TransformException, TransformListener


class BottlePointBuffer:
    """Collect a short, thread-safe window of bottle coordinates."""

    def __init__(self, expected_frame, sample_count, sample_timeout,
                 latest_sample_timeout):
        self.expected_frame = expected_frame
        self.sample_timeout = sample_timeout
        self.latest_sample_timeout = latest_sample_timeout
        self.samples = deque(maxlen=sample_count)
        self.lock = Lock()

    def callback(self, message):
        if message.header.frame_id != self.expected_frame:
            return
        sample = (
            time.monotonic(),
            message.point.x,
            message.point.y,
            message.point.z,
        )
        with self.lock:
            self.samples.append(sample)

    def stable_point(self, minimum_samples, maximum_spread):
        now = time.monotonic()
        with self.lock:
            recent = [
                sample for sample in self.samples
                if now - sample[0] <= self.sample_timeout
            ]

        if not recent or now - recent[-1][0] > self.latest_sample_timeout:
            return None, 'the latest bottle point is stale'
        if len(recent) < minimum_samples:
            return None, (
                f'need {minimum_samples} fresh samples; received {len(recent)}'
            )

        coordinates = np.asarray([sample[1:] for sample in recent])
        spread = np.ptp(coordinates, axis=0)
        if float(np.max(spread)) > maximum_spread:
            return None, (
                'point is not stable; spread is '
                f'x={spread[0]:.3f}, y={spread[1]:.3f}, '
                f'z={spread[2]:.3f} m'
            )
        return np.median(coordinates, axis=0), None

    def clear(self):
        with self.lock:
            self.samples.clear()


class AmazingHandBottlePickNode(Node):
    """Coordinate MoveIt arm motion and Amazing Hand tactile grasping."""

    def __init__(self):
        super().__init__('viperx_amazing_hand_bottle_pick')

        # Perception and arm interfaces.
        self.declare_parameter(
            'bottle_topic', '/yolo/bottle_surface_point_base')
        self.declare_parameter('base_frame', 'vx300s/base_link')
        self.declare_parameter('ee_frame', 'vx300s/ee_gripper_link')
        self.declare_parameter('moveit_service', '/moveit_plan')
        self.declare_parameter('joint_states_topic', '/vx300s/joint_states')
        self.declare_parameter('lock_wrist_rotate', True)
        self.declare_parameter('wrist_rotate_tolerance', 0.02)

        # AmazingHand_ROS2 interfaces.
        self.declare_parameter('hand_task_topic', '/hand/task_cmd')
        self.declare_parameter('hand_position_topic', '/hand/cmd_positions')
        self.declare_parameter('hand_open_command', 'open')
        self.declare_parameter('hand_grasp_command', 'grasp')
        self.declare_parameter('hand_stop_command', 'stop')
        self.declare_parameter('hand_command_repetitions', 3)
        self.declare_parameter('hand_command_interval', 0.05)
        self.declare_parameter('hand_motion_quiet_time', 0.35)
        self.declare_parameter('hand_open_timeout', 6.0)
        self.declare_parameter('hand_grasp_timeout', 8.0)

        # Safe defaults: validate the plan and stop before approaching.
        self.declare_parameter('dry_run', True)
        self.declare_parameter('pregrasp_only', True)
        self.declare_parameter('approach_only', False)

        # Detection filtering.
        self.declare_parameter('sample_count', 15)
        self.declare_parameter('minimum_samples', 5)
        self.declare_parameter('sample_timeout', 4.0)
        self.declare_parameter('latest_sample_timeout', 1.5)
        self.declare_parameter('maximum_point_spread', 0.025)

        # Grasp geometry. tool_offset is the distance from the MoveIt end
        # effector origin to the Amazing Hand bottle-contact point.
        self.declare_parameter('tool_offset', 0.0)
        self.declare_parameter('approach_distance', 0.04)
        self.declare_parameter('grasp_pitch', 0.0)
        self.declare_parameter('grasp_x_offset', 0.05)
        self.declare_parameter('grasp_y_offset', 0.05)
        self.declare_parameter('grasp_z_offset', -0.04)
        self.declare_parameter('minimum_grasp_height', 0.08)
        # Placement offsets in the base frame: +Y is the robot's left side.
        self.declare_parameter('lift_distance', 0.10)
        self.declare_parameter('place_left_distance', 0.10)
        self.declare_parameter('place_lower_distance', 0.10)
        self.declare_parameter('post_grasp_velocity_scale', 0.25)

        self.declare_parameter('server_timeout', 10.0)
        self.declare_parameter('request_timeout', 60.0)

        self.points = BottlePointBuffer(
            expected_frame=self.value('base_frame'),
            sample_count=int(self.value('sample_count')),
            sample_timeout=float(self.value('sample_timeout')),
            latest_sample_timeout=float(self.value('latest_sample_timeout')),
        )
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            PointStamped,
            self.value('bottle_topic'),
            self.points.callback,
            qos,
        )

        self.wrist_rotate_lock = Lock()
        self.wrist_rotate_position = None
        self.create_subscription(
            JointState,
            self.value('joint_states_topic'),
            self.joint_state_callback,
            qos,
        )

        self.hand_feedback_lock = Lock()
        self.hand_feedback_count = 0
        self.last_hand_feedback_time = None
        self.create_subscription(
            Float32MultiArray,
            self.value('hand_position_topic'),
            self.hand_position_callback,
            qos,
        )
        self.hand_task_publisher = self.create_publisher(
            String,
            self.value('hand_task_topic'),
            qos,
        )
        self.moveit = self.create_client(
            MoveItPlan,
            self.value('moveit_service'),
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
            spin_thread=False,
        )

    def value(self, name):
        return self.get_parameter(name).value

    def joint_state_callback(self, message):
        try:
            index = message.name.index('wrist_rotate')
            position = float(message.position[index])
        except (ValueError, IndexError):
            return
        with self.wrist_rotate_lock:
            self.wrist_rotate_position = position

    def hand_position_callback(self, unused_message):
        with self.hand_feedback_lock:
            self.hand_feedback_count += 1
            self.last_hand_feedback_time = time.monotonic()

    def wait_for_future(self, future, timeout):
        completed = Event()
        future.add_done_callback(lambda unused_future: completed.set())
        if not completed.wait(timeout):
            return None
        try:
            return future.result()
        except Exception as exc:
            self.get_logger().error(f'ROS request failed: {exc}')
            return None

    @staticmethod
    def quaternion_from_euler(roll, pitch, yaw):
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    def make_pose(self, x, y, z, yaw):
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        quaternion = self.quaternion_from_euler(
            0.0,
            float(self.value('grasp_pitch')),
            float(yaw),
        )
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    @staticmethod
    def offset_pose(reference, x=0.0, y=0.0, z=0.0):
        pose = Pose()
        pose.position.x = reference.position.x + float(x)
        pose.position.y = reference.position.y + float(y)
        pose.position.z = reference.position.z + float(z)
        pose.orientation.x = reference.orientation.x
        pose.orientation.y = reference.orientation.y
        pose.orientation.z = reference.orientation.z
        pose.orientation.w = reference.orientation.w
        return pose

    @staticmethod
    def rotate_pose_about_base_z(pose, angle):
        half_angle = float(angle) * 0.5
        sine = math.sin(half_angle)
        cosine = math.cos(half_angle)
        x = pose.orientation.x
        y = pose.orientation.y
        z = pose.orientation.z
        w = pose.orientation.w
        # Left-multiply by a base-frame Z rotation.
        pose.orientation.x = cosine * x - sine * y
        pose.orientation.y = sine * x + cosine * y
        pose.orientation.z = cosine * z + sine * w
        pose.orientation.w = cosine * w - sine * z
        return pose

    def current_ee_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.value('base_frame'),
                self.value('ee_frame'),
                Time(),
                timeout=Duration(seconds=2.0),
            )
        except TransformException as exc:
            self.get_logger().error(
                'Cannot read the current end-effector pose: '
                f'{exc}.'
            )
            return None

        pose = Pose()
        pose.position.x = transform.transform.translation.x
        pose.position.y = transform.transform.translation.y
        pose.position.z = transform.transform.translation.z
        pose.orientation = transform.transform.rotation
        return pose

    def call_moveit(
            self, command, pose=None, waypoints=None, velocity_scale=1.0):
        request = MoveItPlan.Request()
        request.cmd = command
        request.velocity_scaling_factor = float(velocity_scale)
        request.lock_wrist_rotate = (
            bool(self.value('lock_wrist_rotate'))
            and command != MoveItPlan.Request.CMD_EXECUTE
        )
        if request.lock_wrist_rotate:
            with self.wrist_rotate_lock:
                wrist_position = self.wrist_rotate_position
            if wrist_position is None:
                self.get_logger().error(
                    'No wrist_rotate state received on '
                    f'{self.value("joint_states_topic")}; planning cancelled.'
                )
                return False
            request.wrist_rotate_position = wrist_position
        request.wrist_rotate_tolerance = float(
            self.value('wrist_rotate_tolerance'))
        if pose is not None:
            request.ee_pose = pose
        if waypoints is not None:
            request.waypoints = waypoints
        response = self.wait_for_future(
            self.moveit.call_async(request),
            float(self.value('request_timeout')),
        )
        if response is None:
            self.get_logger().error('MoveIt service request timed out.')
            return False
        if not response.success:
            self.get_logger().error(response.msg.data)
            return False
        self.get_logger().info(response.msg.data)
        return True

    def move_pose(self, label, pose, execute=True):
        self.get_logger().info(
            f'MoveIt {label} target: x={pose.position.x:.3f}, '
            f'y={pose.position.y:.3f}, z={pose.position.z:.3f} m.'
        )
        if not self.call_moveit(MoveItPlan.Request.CMD_PLAN_POSE, pose):
            return False
        if not execute:
            self.get_logger().info(f'{label} planned; execution skipped.')
            return True
        return self.call_moveit(MoveItPlan.Request.CMD_EXECUTE)

    def move_cartesian_waypoints(self, label, waypoints):
        velocity_scale = float(self.value('post_grasp_velocity_scale'))
        if not 0.0 < velocity_scale <= 1.0:
            self.get_logger().error(
                'post_grasp_velocity_scale must be greater than 0 and at '
                'most 1.'
            )
            return False
        self.get_logger().info(
            f'MoveIt Cartesian {label}: {len(waypoints)} continuous '
            f'waypoints at {velocity_scale * 100.0:.0f}% speed.'
        )
        if not self.call_moveit(
                MoveItPlan.Request.CMD_PLAN_CARTESIAN,
                waypoints=waypoints,
                velocity_scale=velocity_scale):
            return False
        return self.call_moveit(MoveItPlan.Request.CMD_EXECUTE)

    def wait_for_hand_controller(self):
        deadline = time.monotonic() + float(self.value('server_timeout'))
        while rclpy.ok() and time.monotonic() < deadline:
            if self.hand_task_publisher.get_subscription_count() > 0:
                return True
            time.sleep(0.1)
        self.get_logger().error(
            'Amazing Hand controller is unavailable on '
            f'{self.value("hand_task_topic")}. Make sure AmazingHand_ROS2 is '
            'running and both containers use the same ROS_DOMAIN_ID.'
        )
        return False

    def command_hand(self, command, timeout):
        """Publish a hand task and wait until position commands become quiet."""
        if not self.wait_for_hand_controller():
            return False

        with self.hand_feedback_lock:
            initial_count = self.hand_feedback_count

        repetitions = max(int(self.value('hand_command_repetitions')), 1)
        interval = max(float(self.value('hand_command_interval')), 0.01)
        message = String(data=str(command))
        self.get_logger().info(f'Amazing Hand command: {message.data}.')
        for _ in range(repetitions):
            self.hand_task_publisher.publish(message)
            time.sleep(interval)

        deadline = time.monotonic() + float(timeout)
        quiet_time = max(float(self.value('hand_motion_quiet_time')), 0.1)
        while rclpy.ok() and time.monotonic() < deadline:
            with self.hand_feedback_lock:
                count = self.hand_feedback_count
                last_time = self.last_hand_feedback_time
            received_new_feedback = count > initial_count
            motion_is_quiet = (
                last_time is not None
                and time.monotonic() - last_time >= quiet_time
            )
            if received_new_feedback and motion_is_quiet:
                self.get_logger().info(
                    f'Amazing Hand {message.data} motion completed.'
                )
                return True
            time.sleep(0.05)

        self.hand_task_publisher.publish(
            String(data=str(self.value('hand_stop_command')))
        )
        self.get_logger().error(
            f'Amazing Hand {message.data} did not complete within '
            f'{float(timeout):.1f} s; sent stop.'
        )
        return False

    def plan_targets(self, bottle_point):
        bottle_x, bottle_y, bottle_z = [float(value) for value in bottle_point]
        if not (
            0.18 <= bottle_x <= 0.60
            and -0.40 <= bottle_y <= 0.40
            and 0.0 <= bottle_z <= 0.40
        ):
            raise ValueError('bottle point is outside the configured safe workspace')

        target_x = bottle_x + float(self.value('grasp_x_offset'))
        target_y = bottle_y + float(self.value('grasp_y_offset'))
        radial_distance = math.hypot(target_x, target_y)
        if radial_distance < 0.01:
            raise ValueError('bottle point is too close to the base axis')

        unit_x = target_x / radial_distance
        unit_y = target_y / radial_distance
        tool_offset = float(self.value('tool_offset'))
        grasp_x = target_x - tool_offset * unit_x
        grasp_y = target_y - tool_offset * unit_y
        approach = float(self.value('approach_distance'))
        if approach < 0.0:
            raise ValueError('approach_distance cannot be negative')

        grasp_z = max(
            bottle_z + float(self.value('grasp_z_offset')),
            float(self.value('minimum_grasp_height')),
        )
        lift_distance = float(self.value('lift_distance'))
        place_left_distance = float(self.value('place_left_distance'))
        place_lower_distance = float(self.value('place_lower_distance'))
        if min(lift_distance, place_left_distance, place_lower_distance) < 0.0:
            raise ValueError('lift and placement distances cannot be negative')

        lift_z = grasp_z + lift_distance
        place_y = grasp_y + place_left_distance
        place_z = lift_z - place_lower_distance
        if not 0.03 <= grasp_z <= 0.40:
            raise ValueError('adjusted grasp height is outside the safe workspace')
        if not 0.03 <= lift_z <= 0.50:
            raise ValueError('lift target is outside the configured safe workspace')
        if not -0.45 <= place_y <= 0.45:
            raise ValueError('left placement target is outside the safe workspace')
        if not 0.03 <= place_z <= 0.50:
            raise ValueError('lowered placement target is outside the safe workspace')

        return {
            'pregrasp_x': grasp_x - approach * unit_x,
            'pregrasp_y': grasp_y - approach * unit_y,
            'grasp_x': grasp_x,
            'grasp_y': grasp_y,
            'grasp_z': grasp_z,
            'lift_z': lift_z,
            'place_y': place_y,
            'place_z': place_z,
            'yaw': math.atan2(target_y, target_x),
        }

    def pick(self, bottle_point):
        if not self.moveit.wait_for_service(
                timeout_sec=float(self.value('server_timeout'))):
            self.get_logger().error(
                "MoveIt service '/moveit_plan' is unavailable. Launch the "
                'Interbotix MoveIt interface first.'
            )
            return False

        point = np.asarray(bottle_point, dtype=float)
        try:
            target = self.plan_targets(point)
        except ValueError as exc:
            self.get_logger().error(f'Cannot pick bottle: {exc}.')
            return False

        self.get_logger().info(
            f'Locked bottle point: x={point[0]:.3f}, y={point[1]:.3f}, '
            f'z={point[2]:.3f} m.'
        )
        pregrasp = self.make_pose(
            target['pregrasp_x'], target['pregrasp_y'],
            target['grasp_z'], target['yaw'])
        grasp = self.make_pose(
            target['grasp_x'], target['grasp_y'],
            target['grasp_z'], target['yaw'])

        if bool(self.value('dry_run')):
            if not self.move_pose('pre-grasp', pregrasp, execute=False):
                return False
            if float(self.value('approach_distance')) > 1e-6:
                if not self.move_pose('grasp', grasp, execute=False):
                    return False
            self.get_logger().info(
                'DRY RUN complete: plans accepted; arm and hand did not move.'
            )
            return True

        if not self.command_hand(
            self.value('hand_open_command'),
            self.value('hand_open_timeout'),
        ):
            return False

        self.points.clear()
        if not self.move_pose('pre-grasp', pregrasp):
            return False
        if bool(self.value('pregrasp_only')):
            self.get_logger().info(
                'Pre-grasp-only sequence completed; stopping before approach.'
            )
            return True

        if float(self.value('approach_distance')) > 1e-6:
            if not self.move_pose('grasp', grasp):
                return False
        if bool(self.value('approach_only')):
            self.get_logger().info(
                'Approach-only sequence completed; Amazing Hand remains open.'
            )
            return True

        if not self.command_hand(
            self.value('hand_grasp_command'),
            self.value('hand_grasp_timeout'),
        ):
            return False

        # Use the measured post-grasp pose as the Cartesian start. Reusing the
        # theoretical grasp pose can make the first IK sample select a distant
        # wrist solution even though the arm is already at the bottle.
        current_pose = self.current_ee_pose()
        if current_pose is None:
            return False

        lift_distance = float(self.value('lift_distance'))
        left_distance = float(self.value('place_left_distance'))
        lower_distance = float(self.value('place_lower_distance'))
        lift = self.offset_pose(current_pose, z=lift_distance)
        place_above = self.offset_pose(
            current_pose,
            y=left_distance,
            z=lift_distance,
        )
        place = self.offset_pose(
            current_pose,
            y=left_distance,
            z=lift_distance - lower_distance,
        )

        # Turn the whole end effector with the arm's radial direction during
        # the lateral move. This lets the waist supply the yaw change instead
        # of forcing wrist_rotate to counter-rotate.
        start_radial_yaw = math.atan2(
            current_pose.position.y,
            current_pose.position.x,
        )
        place_radial_yaw = math.atan2(
            place.position.y,
            place.position.x,
        )
        radial_yaw_change = math.atan2(
            math.sin(place_radial_yaw - start_radial_yaw),
            math.cos(place_radial_yaw - start_radial_yaw),
        )
        self.rotate_pose_about_base_z(place_above, radial_yaw_change)
        self.rotate_pose_about_base_z(place, radial_yaw_change)
        if not (
            0.15 <= current_pose.position.x <= 0.65
            and -0.45 <= place.position.y <= 0.45
            and 0.03 <= place.position.z <= 0.50
            and 0.03 <= lift.position.z <= 0.50
        ):
            self.get_logger().error(
                'Measured Cartesian placement route is outside the safe '
                'workspace; motion cancelled.'
            )
            return False
        self.get_logger().info(
            'Measured Cartesian start: '
            f'x={current_pose.position.x:.3f}, '
            f'y={current_pose.position.y:.3f}, '
            f'z={current_pose.position.z:.3f} m; '
            f'radial yaw change={math.degrees(radial_yaw_change):.2f} deg.'
        )
        if not self.move_cartesian_waypoints(
                'lift, move left, and lower',
                [lift, place_above, place]):
            return False

        if not self.command_hand(
            self.value('hand_open_command'),
            self.value('hand_open_timeout'),
        ):
            return False

        self.get_logger().info(
            'VX300s + Amazing Hand bottle pick-and-place sequence completed.'
        )
        return True


def main():
    rclpy.init()
    node = AmazingHandBottlePickNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    node.get_logger().warning(
        f'Amazing Hand bottle pick ready; dry_run={node.value("dry_run")}, '
        f'pregrasp_only={node.value("pregrasp_only")}, '
        f'tool_offset={float(node.value("tool_offset")):.3f} m.'
    )

    try:
        print('Waiting for a stable bottle position...')
        point = None
        while rclpy.ok():
            point, unused_reason = node.points.stable_point(
                int(node.value('minimum_samples')),
                float(node.value('maximum_point_spread')),
            )
            if point is not None:
                break
            time.sleep(0.1)

        if rclpy.ok() and point is not None:
            locked_point = np.array(point, dtype=float, copy=True)
            print(
                'Detected and locked bottle position: '
                f'x={locked_point[0]:.3f}, y={locked_point[1]:.3f}, '
                f'z={locked_point[2]:.3f} m'
            )
            input('Press Enter to execute the pick (Ctrl+C to cancel)...')
            node.pick(locked_point)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
