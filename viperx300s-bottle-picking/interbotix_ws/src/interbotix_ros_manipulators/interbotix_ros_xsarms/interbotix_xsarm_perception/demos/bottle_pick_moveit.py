#!/usr/bin/env python3

"""Plan and execute a YOLO-guided VX300s bottle pick with MoveIt."""

from collections import deque
import math
from threading import Event, Lock, Thread
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PointStamped, Pose
from interbotix_moveit_interface_msgs.srv import MoveItPlan
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_msgs.srv import OperatingModes
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


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


class BottleMoveItNode(Node):
    """Use the Interbotix MoveIt service and trajectory gripper controller."""

    def __init__(self):
        super().__init__('bottle_pick_moveit')
        self.declare_parameter(
            'bottle_topic', '/yolo/bottle_surface_point_base')
        self.declare_parameter('base_frame', 'vx300s/base_link')
        self.declare_parameter('moveit_service', '/moveit_plan')
        self.declare_parameter(
            'gripper_action',
            '/vx300s/gripper_controller/follow_joint_trajectory')
        self.declare_parameter(
            'gripper_command_topic', '/vx300s/commands/joint_single')
        self.declare_parameter(
            'operating_modes_service', '/vx300s/set_operating_modes')
        self.declare_parameter('joint_states_topic', '/vx300s/joint_states')
        self.declare_parameter('lock_wrist_rotate', True)
        self.declare_parameter('wrist_rotate_tolerance', 0.02)
        self.declare_parameter('dry_run', True)
        self.declare_parameter('sample_count', 15)
        self.declare_parameter('minimum_samples', 5)
        self.declare_parameter('sample_timeout', 4.0)
        self.declare_parameter('latest_sample_timeout', 1.5)
        self.declare_parameter('maximum_point_spread', 0.025)
        self.declare_parameter('tool_offset', 0.0)
        self.declare_parameter('approach_distance', 0.0)
        self.declare_parameter('pregrasp_only', True)
        self.declare_parameter('approach_only', False)
        self.declare_parameter('grasp_pitch', 0.0)
        self.declare_parameter('grasp_x_offset', 0.05)
        self.declare_parameter('grasp_y_offset', 0.05)
        self.declare_parameter('grasp_z_offset', 0.0)
        self.declare_parameter('minimum_grasp_height', 0.08)
        # Placement offsets in the base frame: +Y is the robot's left side.
        self.declare_parameter('lift_distance', 0.04)
        self.declare_parameter('place_left_distance', 0.05)
        self.declare_parameter('place_lower_distance', 0.04)
        self.declare_parameter('gripper_open_position', 0.057)
        self.declare_parameter('gripper_closed_position', 0.021)
        self.declare_parameter('gripper_moving_time', 2.0)
        self.declare_parameter('use_pwm_gripper', True)
        self.declare_parameter('gripper_open_pwm', 230.0)
        self.declare_parameter('gripper_close_pwm', -350.0)
        self.declare_parameter('gripper_open_pwm_time', 1.0)
        self.declare_parameter('gripper_close_pwm_time', 0.8)
        self.declare_parameter('gripper_pwm_publish_rate', 100.0)
        self.declare_parameter('gripper_open_threshold', 0.045)
        self.declare_parameter('gripper_open_retries', 2)
        self.declare_parameter('gripper_settle_time', 0.2)
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
        self.joint_positions = {}
        self.joint_positions_lock = Lock()
        self.create_subscription(
            JointState,
            self.value('joint_states_topic'),
            self.joint_state_callback,
            qos,
        )
        self.moveit = self.create_client(
            MoveItPlan, self.value('moveit_service'))
        self.gripper = ActionClient(
            self, FollowJointTrajectory, self.value('gripper_action'))
        self.gripper_command_publisher = self.create_publisher(
            JointSingleCommand,
            self.value('gripper_command_topic'),
            10,
        )
        self.operating_modes = self.create_client(
            OperatingModes, self.value('operating_modes_service'))
        self.gripper_pwm_configured = False

    def value(self, name):
        return self.get_parameter(name).value

    def joint_state_callback(self, message):
        with self.joint_positions_lock:
            self.joint_positions.update(zip(message.name, message.position))

    def wait_for_future(self, future, timeout):
        completed = Event()
        future.add_done_callback(lambda unused_future: completed.set())
        if not completed.wait(timeout):
            return None
        try:
            return future.result()
        except Exception as exc:  # rclpy propagates service/action failures here
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
            0.0, float(self.value('grasp_pitch')), float(yaw))
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def call_moveit(self, command, pose=None, waypoints=None):
        timeout = float(self.value('request_timeout'))
        request = MoveItPlan.Request()
        request.cmd = command
        request.lock_wrist_rotate = (
            bool(self.value('lock_wrist_rotate'))
            and command != MoveItPlan.Request.CMD_EXECUTE
        )
        if request.lock_wrist_rotate:
            with self.joint_positions_lock:
                wrist_position = self.joint_positions.get('wrist_rotate')
            if wrist_position is None:
                self.get_logger().error(
                    'No wrist_rotate state received; planning cancelled.')
                return False
            request.wrist_rotate_position = float(wrist_position)
        request.wrist_rotate_tolerance = float(
            self.value('wrist_rotate_tolerance'))
        if pose is not None:
            request.ee_pose = pose
        if waypoints is not None:
            request.waypoints = waypoints
        response = self.wait_for_future(
            self.moveit.call_async(request), timeout)
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
            f'y={pose.position.y:.3f}, z={pose.position.z:.3f} m.')
        if not self.call_moveit(MoveItPlan.Request.CMD_PLAN_POSE, pose):
            return False
        if not execute:
            self.get_logger().info(f'{label} planned; execution skipped.')
            return True
        return self.call_moveit(MoveItPlan.Request.CMD_EXECUTE)

    def move_cartesian_waypoints(self, label, waypoints):
        self.get_logger().info(
            f'MoveIt Cartesian {label}: {len(waypoints)} continuous waypoints.')
        if not self.call_moveit(
                MoveItPlan.Request.CMD_PLAN_CARTESIAN,
                waypoints=waypoints):
            return False
        return self.call_moveit(MoveItPlan.Request.CMD_EXECUTE)

    def set_gripper_operating_mode(self, mode):
        if not self.operating_modes.wait_for_service(
                timeout_sec=float(self.value('server_timeout'))):
            self.get_logger().error(
                'Gripper operating-modes service is unavailable.')
            return False
        request = OperatingModes.Request()
        request.cmd_type = 'single'
        request.name = 'gripper'
        request.mode = mode
        request.profile_type = 'velocity'
        if mode == 'linear_position':
            request.profile_velocity = 131
            request.profile_acceleration = 15
        response = self.wait_for_future(
            self.operating_modes.call_async(request),
            float(self.value('request_timeout')),
        )
        if response is None:
            self.get_logger().error(
                f'Failed to set gripper operating mode to {mode}.')
            return False
        self.get_logger().info(f'Gripper operating mode set to {mode}.')
        return True

    def command_gripper_pwm(self, pwm, duration):
        if not self.gripper_pwm_configured:
            if not self.set_gripper_operating_mode('pwm'):
                return False
            self.gripper_pwm_configured = True

        pwm = float(pwm)
        duration = max(float(duration), 0.0)
        rate = max(float(self.value('gripper_pwm_publish_rate')), 1.0)
        command = JointSingleCommand(name='gripper', cmd=pwm)
        stop_command = JointSingleCommand(name='gripper', cmd=0.0)
        self.get_logger().info(
            f'Commanding gripper PWM={pwm:.0f} for {duration:.2f} s.')
        deadline = time.monotonic() + duration
        try:
            while time.monotonic() < deadline and rclpy.ok():
                self.gripper_command_publisher.publish(command)
                time.sleep(1.0 / rate)
        finally:
            self.gripper_command_publisher.publish(stop_command)
        return True

    def ensure_gripper_open(self):
        retries = max(int(self.value('gripper_open_retries')), 1)
        threshold = float(self.value('gripper_open_threshold'))
        settle_time = max(float(self.value('gripper_settle_time')), 0.0)
        for attempt in range(1, retries + 1):
            if not self.command_gripper(self.value('gripper_open_position')):
                return False
            time.sleep(settle_time)
            with self.joint_positions_lock:
                actual_position = self.joint_positions.get('left_finger')
            if actual_position is not None and actual_position >= threshold:
                self.get_logger().info(
                    f'Gripper open confirmed at {actual_position:.3f} m.')
                return True
            self.get_logger().warning(
                f'Gripper did not open on attempt {attempt}/{retries}; '
                f'left_finger={actual_position}.')
        self.get_logger().error(
            'Gripper failed to open; arm motion cancelled.')
        return False

    def restore_gripper_operating_mode(self):
        if self.gripper_pwm_configured and rclpy.ok():
            self.gripper_command_publisher.publish(
                JointSingleCommand(name='gripper', cmd=0.0))
            self.set_gripper_operating_mode('linear_position')
            self.gripper_pwm_configured = False

    def command_gripper(self, position):
        if bool(self.value('use_pwm_gripper')):
            midpoint = (
                float(self.value('gripper_open_position'))
                + float(self.value('gripper_closed_position'))
            ) * 0.5
            if float(position) >= midpoint:
                return self.command_gripper_pwm(
                    self.value('gripper_open_pwm'),
                    self.value('gripper_open_pwm_time'),
                )
            return self.command_gripper_pwm(
                self.value('gripper_close_pwm'),
                self.value('gripper_close_pwm_time'),
            )

        self.get_logger().info(
            f'Commanding gripper left_finger={float(position):.3f} m.')
        server_timeout = float(self.value('server_timeout'))
        if not self.gripper.wait_for_server(timeout_sec=server_timeout):
            self.get_logger().error('MoveIt gripper controller is unavailable.')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ['left_finger']
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(
            nanoseconds=int(
                float(self.value('gripper_moving_time')) * 1_000_000_000
            )
        ).to_msg()
        goal.trajectory.points = [point]

        timeout = float(self.value('request_timeout'))
        goal_handle = self.wait_for_future(
            self.gripper.send_goal_async(goal), timeout)
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Gripper trajectory was rejected.')
            return False
        result = self.wait_for_future(goal_handle.get_result_async(), timeout)
        if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error('Gripper trajectory did not succeed.')
            return False
        with self.joint_positions_lock:
            actual_position = self.joint_positions.get('left_finger')
        if actual_position is None:
            self.get_logger().warning(
                'Gripper trajectory succeeded, but no left_finger state '
                'was available.')
        else:
            self.get_logger().info(
                'Gripper trajectory succeeded; actual left_finger='
                f'{actual_position:.3f} m.')
        return True

    def plan_targets(self, bottle_point):
        bottle_x, bottle_y, bottle_z = [float(value) for value in bottle_point]
        if not (
            0.18 <= bottle_x <= 0.60
            and -0.40 <= bottle_y <= 0.40
            and 0.0 <= bottle_z <= 0.40
        ):
            raise ValueError('bottle point is outside the safe workspace')

        target_x = bottle_x + float(self.value('grasp_x_offset'))
        target_y = bottle_y + float(self.value('grasp_y_offset'))
        radial_distance = math.hypot(target_x, target_y)
        if radial_distance < 0.01:
            raise ValueError('bottle point is too close to the base axis')
        unit_x = target_x / radial_distance
        unit_y = target_y / radial_distance
        grasp_x = target_x - float(self.value('tool_offset')) * unit_x
        grasp_y = target_y - float(self.value('tool_offset')) * unit_y
        approach = float(self.value('approach_distance'))
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
            raise ValueError('lift target is outside the safe workspace')
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
            'place_yaw': math.atan2(place_y, grasp_x),
        }

    def pick(self, bottle_point=None):
        if not self.moveit.wait_for_service(
                timeout_sec=float(self.value('server_timeout'))):
            self.get_logger().error(
                "MoveIt service '/moveit_plan' is unavailable. Launch "
                'interbotix_xsarm_moveit_interface first.')
            return

        if bottle_point is None:
            point, reason = self.points.stable_point(
                int(self.value('minimum_samples')),
                float(self.value('maximum_point_spread')),
            )
            if point is None:
                self.get_logger().warning(f'Cannot pick bottle: {reason}.')
                return
        else:
            point = np.asarray(bottle_point, dtype=float)
        try:
            target = self.plan_targets(point)
        except ValueError as exc:
            self.get_logger().error(f'Cannot pick bottle: {exc}.')
            return

        self.get_logger().info(
            f'Locked bottle point: x={point[0]:.3f}, y={point[1]:.3f}, '
            f'z={point[2]:.3f} m.')
        pregrasp = self.make_pose(
            target['pregrasp_x'], target['pregrasp_y'],
            target['grasp_z'], target['yaw'])
        grasp = self.make_pose(
            target['grasp_x'], target['grasp_y'],
            target['grasp_z'], target['yaw'])

        dry_run = bool(self.value('dry_run'))
        if dry_run:
            self.move_pose('pre-grasp', pregrasp, execute=False)
            self.get_logger().info(
                'DRY RUN complete: MoveIt did not execute the trajectory.')
            return

        self.points.clear()
        if not self.ensure_gripper_open():
            return
        if not self.move_pose('pre-grasp', pregrasp):
            return
        if bool(self.value('pregrasp_only')):
            self.get_logger().info(
                'Pre-grasp-only sequence completed; stopping before approach.')
            return
        if float(self.value('approach_distance')) > 1e-6:
            if not self.move_pose('grasp', grasp):
                return
        else:
            self.get_logger().info(
                'Direct-grasp mode: already at the grasp target; '
                'skipping a second MoveIt plan.')
        if bool(self.value('approach_only')):
            self.get_logger().info(
                'Approach-only sequence completed; gripper remains open.')
            return
        if not self.command_gripper(self.value('gripper_closed_position')):
            return

        lift = self.make_pose(
            target['grasp_x'], target['grasp_y'],
            target['lift_z'], target['yaw'])
        place_above = self.make_pose(
            target['grasp_x'], target['place_y'],
            target['lift_z'], target['place_yaw'])
        place = self.make_pose(
            target['grasp_x'], target['place_y'],
            target['place_z'], target['place_yaw'])
        if not self.move_cartesian_waypoints(
                'lift, move left, and lower',
                [lift, place_above, place]):
            return

        if not self.ensure_gripper_open():
            return
        self.get_logger().info(
            'MoveIt bottle pick-and-place sequence completed.')

    def print_status(self):
        point, reason = self.points.stable_point(
            int(self.value('minimum_samples')),
            float(self.value('maximum_point_spread')),
        )
        if point is None:
            print(f'Bottle point unavailable: {reason}.')
        else:
            print(
                f'Bottle point: x={point[0]:.3f}, y={point[1]:.3f}, '
                f'z={point[2]:.3f} m')


def main():
    rclpy.init()
    node = BottleMoveItNode()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    node.get_logger().warning(
        f'MoveIt bottle command ready; dry_run={node.value("dry_run")}, '
        f'pregrasp_only={node.value("pregrasp_only")}.')
    try:
        print('Waiting for a stable bottle position...')
        while rclpy.ok():
            point, unused_reason = node.points.stable_point(
                int(node.value('minimum_samples')),
                float(node.value('maximum_point_spread')),
            )
            if point is not None:
                break
            time.sleep(0.1)

        if rclpy.ok():
            locked_point = np.array(point, dtype=float, copy=True)
            print(
                'Detected and locked bottle position: '
                f'x={locked_point[0]:.3f}, y={locked_point[1]:.3f}, '
                f'z={locked_point[2]:.3f} m')
            input('Press Enter to execute the pick (Ctrl+C to cancel)...')
            node.pick(locked_point)
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        node.restore_gripper_operating_mode()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
