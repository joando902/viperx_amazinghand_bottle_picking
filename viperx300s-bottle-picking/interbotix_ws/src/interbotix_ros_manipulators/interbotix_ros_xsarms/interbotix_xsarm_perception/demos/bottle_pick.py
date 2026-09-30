#!/usr/bin/env python3

# Copyright 2026 joando
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Pick the latest stable YOLO bottle point after a terminal command."""

from collections import deque
import math
from threading import Lock
import time

from geometry_msgs.msg import PointStamped
from interbotix_common_modules.common_robot.robot import (
    create_interbotix_global_node,
    robot_shutdown,
    robot_startup,
)
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy


class BottlePointBuffer:
    """Collect a short, thread-safe window of bottle coordinates."""

    def __init__(
        self,
        expected_frame,
        sample_count,
        sample_timeout,
        latest_sample_timeout,
    ):
        self.expected_frame = expected_frame
        self.sample_timeout = sample_timeout
        self.latest_sample_timeout = latest_sample_timeout
        self.samples = deque(maxlen=sample_count)
        self.lock = Lock()

    def callback(self, message):
        """Store a point together with its local arrival time."""
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
        """Return the median of recent samples if their spread is acceptable."""
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
                f'x={spread[0]:.3f}, y={spread[1]:.3f}, z={spread[2]:.3f} m'
            )

        return np.median(coordinates, axis=0), None

    def clear(self):
        """Discard coordinates captured before a robot motion."""
        with self.lock:
            self.samples.clear()


def declare_parameters(node):
    """Declare parameters that must be tuned for the physical gripper setup."""
    node.declare_parameter('bottle_topic', '/yolo/bottle_surface_point_base')
    node.declare_parameter('base_frame', 'vx300s/base_link')
    node.declare_parameter('dry_run', True)
    node.declare_parameter('sample_count', 15)
    node.declare_parameter('minimum_samples', 8)
    node.declare_parameter('sample_timeout', 3.0)
    node.declare_parameter('latest_sample_timeout', 0.75)
    node.declare_parameter('maximum_point_spread', 0.025)
    node.declare_parameter('tool_offset', 0.0)
    node.declare_parameter('approach_distance', 0.03)
    node.declare_parameter('pregrasp_only', False)
    node.declare_parameter('approach_only', False)
    node.declare_parameter('grasp_pitch', 0.0)
    node.declare_parameter('grasp_z_offset', 0.0)
    node.declare_parameter('minimum_grasp_height', 0.08)
    node.declare_parameter('lift_distance', 0.08)
    node.declare_parameter('moving_time', 5.0)
    node.declare_parameter('gripper_pressure', 0.25)
    node.declare_parameter('maximum_pose_error', 0.03)


def parameter(node, name):
    return node.get_parameter(name).value


def verify_ee_position(node, bot, label, expected):
    """Compare encoder-derived end-effector position with a commanded target."""
    bot.arm.capture_joint_positions()
    actual = np.asarray(bot.arm.T_sb[:3, 3], dtype=float)
    expected = np.asarray(expected, dtype=float)
    error = float(np.linalg.norm(actual - expected))
    node.get_logger().info(
        f'{label} actual EE: x={actual[0]:.3f}, y={actual[1]:.3f}, '
        f'z={actual[2]:.3f} m; position error={error:.3f} m.'
    )
    if error > float(parameter(node, 'maximum_pose_error')):
        node.get_logger().error(
            f'{label} EE error exceeds the configured safety limit; stopping.'
        )
        return False
    return True


def plan_targets(node, bottle_point):
    """Convert a bottle surface point to horizontal pre-grasp and grasp poses."""
    bottle_x, bottle_y, bottle_z = [float(value) for value in bottle_point]

    if not (
        0.18 <= bottle_x <= 0.60
        and -0.40 <= bottle_y <= 0.40
        and 0.0 <= bottle_z <= 0.40
    ):
        raise ValueError('bottle point is outside the configured safe workspace')

    radial_distance = math.hypot(bottle_x, bottle_y)
    if radial_distance < 0.01:
        raise ValueError('bottle point is too close to the base axis')

    unit_x = bottle_x / radial_distance
    unit_y = bottle_y / radial_distance
    tool_offset = float(parameter(node, 'tool_offset'))
    approach_distance = float(parameter(node, 'approach_distance'))

    grasp_x = bottle_x - tool_offset * unit_x
    grasp_y = bottle_y - tool_offset * unit_y
    pregrasp_x = grasp_x - approach_distance * unit_x
    pregrasp_y = grasp_y - approach_distance * unit_y
    grasp_z = max(
        bottle_z + float(parameter(node, 'grasp_z_offset')),
        float(parameter(node, 'minimum_grasp_height')),
    )
    if not 0.03 <= grasp_z <= 0.40:
        raise ValueError('adjusted grasp height is outside the safe workspace')
    yaw = math.atan2(bottle_y, bottle_x)

    return {
        'pregrasp_x': pregrasp_x,
        'pregrasp_y': pregrasp_y,
        'grasp_x': grasp_x,
        'grasp_y': grasp_y,
        'grasp_z': grasp_z,
        'yaw': yaw,
    }


def pick_bottle(node, bot, point_buffer):
    """Plan or execute one horizontal bottle pick."""
    dry_run = bool(parameter(node, 'dry_run'))
    if not dry_run and (
        bot.arm.group_info.mode != 'position'
        or bot.arm.group_info.profile_type != 'time'
        or bot.gripper.gripper_info.mode not in ('pwm', 'current')
    ):
        node.get_logger().error(
            'Robot modes are incompatible with bottle picking. The arm must '
            'use position/time and the gripper must use pwm or current.'
        )
        return

    point, reason = point_buffer.stable_point(
        int(parameter(node, 'minimum_samples')),
        float(parameter(node, 'maximum_point_spread')),
    )
    if point is None:
        node.get_logger().warning(f'Cannot pick bottle: {reason}.')
        return

    try:
        target = plan_targets(node, point)
    except ValueError as exc:
        node.get_logger().error(f'Cannot pick bottle: {exc}.')
        return

    node.get_logger().info(
        'Locked bottle point: '
        f'x={point[0]:.3f}, y={point[1]:.3f}, z={point[2]:.3f} m. '
        'Planned horizontal pre-grasp: '
        f'x={target["pregrasp_x"]:.3f}, '
        f'y={target["pregrasp_y"]:.3f}, '
        f'z={target["grasp_z"]:.3f}, yaw={target["yaw"]:.3f} rad.'
    )
    grasp_pitch = float(parameter(node, 'grasp_pitch'))

    _, pregrasp_valid = bot.arm.set_ee_pose_components(
        x=target['pregrasp_x'],
        y=target['pregrasp_y'],
        z=target['grasp_z'],
        roll=0.0,
        pitch=grasp_pitch,
        yaw=target['yaw'],
        execute=False,
    )
    _, grasp_valid = bot.arm.set_ee_pose_components(
        x=target['grasp_x'],
        y=target['grasp_y'],
        z=target['grasp_z'],
        roll=0.0,
        pitch=grasp_pitch,
        yaw=target['yaw'],
        execute=False,
    )
    node.get_logger().info(
        f'IK result: pre-grasp={pregrasp_valid}, grasp={grasp_valid}.'
    )
    if not pregrasp_valid:
        node.get_logger().error(
            'IK rejected the pre-grasp pose. Try a shorter approach distance '
            'or a higher grasp height.'
        )
        return
    if not grasp_valid:
        node.get_logger().error(
            'IK rejected the grasp pose. Try a higher grasp height or a '
            'slightly tilted pitch.'
        )
        return

    if dry_run:
        node.get_logger().info(
            'DRY RUN complete: IK accepted both poses; the robot did not move.'
        )
        return

    point_buffer.clear()
    moving_time = float(parameter(node, 'moving_time'))
    bot.gripper.set_pressure(float(parameter(node, 'gripper_pressure')))
    bot.gripper.release()

    _, success = bot.arm.set_ee_pose_components(
        x=target['pregrasp_x'],
        y=target['pregrasp_y'],
        z=target['grasp_z'],
        roll=0.0,
        pitch=grasp_pitch,
        yaw=target['yaw'],
        moving_time=moving_time,
        accel_time=min(2.0, moving_time / 2.0),
    )
    if not success:
        node.get_logger().error('Failed to reach the pre-grasp pose.')
        return
    if not verify_ee_position(
        node,
        bot,
        'Pre-grasp',
        [target['pregrasp_x'], target['pregrasp_y'], target['grasp_z']],
    ):
        return

    if bool(parameter(node, 'pregrasp_only')):
        node.get_logger().info(
            'Pre-grasp-only sequence completed; approach and grasp were skipped.'
        )
        return

    success = bot.arm.set_ee_cartesian_trajectory(
        x=float(parameter(node, 'approach_distance')),
        moving_time=moving_time,
    )
    if not success:
        node.get_logger().error('Failed to execute the horizontal approach.')
        return
    if not verify_ee_position(
        node,
        bot,
        'Grasp',
        [target['grasp_x'], target['grasp_y'], target['grasp_z']],
    ):
        return

    if bool(parameter(node, 'approach_only')):
        node.get_logger().info(
            'Approach-only sequence completed; grasp and lift were skipped.'
        )
        return

    bot.gripper.grasp(delay=2.0)
    lift_distance = float(parameter(node, 'lift_distance'))
    if lift_distance > 0.0:
        success = bot.arm.set_ee_cartesian_trajectory(
            z=lift_distance,
            moving_time=moving_time,
        )
        if not success:
            node.get_logger().error(
                'Bottle grasped, but the lift trajectory failed.'
            )
            return

    node.get_logger().info('Bottle pick sequence completed.')


def main():
    node = create_interbotix_global_node(node_name='bottle_pick')
    declare_parameters(node)

    bot = InterbotixManipulatorXS(
        robot_model='vx300s',
        robot_name='vx300s',
        group_name='arm',
        gripper_name='gripper',
        moving_time=float(parameter(node, 'moving_time')),
        accel_time=min(2.0, float(parameter(node, 'moving_time')) / 2.0),
        gripper_pressure=float(parameter(node, 'gripper_pressure')),
        node=node,
    )

    point_buffer = BottlePointBuffer(
        expected_frame=str(parameter(node, 'base_frame')),
        sample_count=int(parameter(node, 'sample_count')),
        sample_timeout=float(parameter(node, 'sample_timeout')),
        latest_sample_timeout=float(parameter(node, 'latest_sample_timeout')),
    )
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(
        PointStamped,
        str(parameter(node, 'bottle_topic')),
        point_buffer.callback,
        qos,
    )

    robot_startup(node)
    dry_run = bool(parameter(node, 'dry_run'))
    node.get_logger().warning(
        f'Bottle command ready; dry_run={dry_run}. '
        'Keep clear of the robot before enabling motion.'
    )

    try:
        while True:
            command = input(
                "Enter 'bottle' to pick, 'status' for the latest point, "
                "or 'quit': "
            ).strip().lower()
            if command in ('bottle', 'botter'):
                pick_bottle(node, bot, point_buffer)
            elif command == 'status':
                point, reason = point_buffer.stable_point(
                    int(parameter(node, 'minimum_samples')),
                    float(parameter(node, 'maximum_point_spread')),
                )
                if point is None:
                    print(f'Bottle point unavailable: {reason}.')
                else:
                    print(
                        'Bottle point: '
                        f'x={point[0]:.3f}, y={point[1]:.3f}, '
                        f'z={point[2]:.3f} m'
                    )
            elif command in ('quit', 'exit'):
                break
            elif command:
                print("Unknown command. Use 'bottle', 'status', or 'quit'.")
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        robot_shutdown(node)


if __name__ == '__main__':
    main()
