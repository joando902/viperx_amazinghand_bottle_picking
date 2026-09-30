#!/usr/bin/env python3

# Copyright 2026 joando
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
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

"""Interactively collect eye-in-hand samples and run OpenCV calibration."""

import threading

import cv2
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_rotation_matrix(quaternion):
    """Convert an xyzw quaternion to a 3-by-3 rotation matrix."""
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = np.linalg.norm([x, y, z, w])
    if norm < np.finfo(np.float64).eps:
        raise ValueError('Received a zero-length quaternion.')
    x, y, z, w = np.array([x, y, z, w]) / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rotation_matrix_to_quaternion(rotation_matrix):
    """Convert a 3-by-3 rotation matrix to an xyzw quaternion."""
    rotation_vector, _ = cv2.Rodrigues(rotation_matrix)
    angle = np.linalg.norm(rotation_vector)
    if angle < np.finfo(np.float64).eps:
        return np.array([0.0, 0.0, 0.0, 1.0])
    axis = rotation_vector.reshape(3) / angle
    return np.concatenate((axis * np.sin(angle / 2.0), [np.cos(angle / 2.0)]))


class HandEyeCalibrator(Node):
    """Collect robot and AprilTag transforms for eye-in-hand calibration."""

    def __init__(self) -> None:
        super().__init__('hand_eye_calibrator')

        self.declare_parameter('base_frame', 'vx300s/base_link')
        self.declare_parameter('gripper_frame', 'vx300s/ee_gripper_link')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('target_frame', 'tag_0')
        self.declare_parameter('minimum_rotation_span_degrees', 20.0)
        self.declare_parameter('maximum_mount_distance', 0.5)
        self.declare_parameter('maximum_target_translation_rms', 0.02)
        self.declare_parameter('maximum_target_rotation_rms_degrees', 5.0)

        self.base_frame = self.get_parameter('base_frame').value
        self.gripper_frame = self.get_parameter('gripper_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.minimum_rotation_span_degrees = self.get_parameter(
            'minimum_rotation_span_degrees'
        ).value
        self.maximum_mount_distance = self.get_parameter(
            'maximum_mount_distance'
        ).value
        self.maximum_target_translation_rms = self.get_parameter(
            'maximum_target_translation_rms'
        ).value
        self.maximum_target_rotation_rms_degrees = self.get_parameter(
            'maximum_target_rotation_rms_degrees'
        ).value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.rotations_gripper_to_base = []
        self.translations_gripper_to_base = []
        self.rotations_target_to_camera = []
        self.translations_target_to_camera = []

    def lookup_transform(self, target_frame, source_frame, time=None):
        """Look up target <- source at a requested time."""
        if time is None:
            time = Time()
        try:
            return self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                time,
                timeout=Duration(seconds=1.0),
            )
        except TransformException as exc:
            self.get_logger().error(
                f'Cannot look up {target_frame} <- {source_frame}: {exc}'
            )
            return None

    @staticmethod
    def transform_to_arrays(transform):
        """Convert a TransformStamped to a translation column and rotation."""
        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        translation_vector = np.array([
            translation.x,
            translation.y,
            translation.z,
        ], dtype=np.float64).reshape(3, 1)
        rotation_matrix = quaternion_to_rotation_matrix([
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ])
        return translation_vector, rotation_matrix

    def sample(self) -> None:
        """Capture one robot pose at the timestamp of the latest tag pose."""
        camera_tag_transform = self.lookup_transform(
            self.camera_frame, self.target_frame
        )
        if camera_tag_transform is None:
            print(f'❌ 擷取失敗；TF 中找不到 {self.target_frame}。')
            return

        tag_time = Time.from_msg(camera_tag_transform.header.stamp)
        base_ee_transform = self.lookup_transform(
            self.base_frame, self.gripper_frame, tag_time
        )
        if base_ee_transform is None:
            print(
                '❌ 無法取得 AprilTag 影像時間所對應的手臂姿態；'
                '請確認 robot_state_publisher 正在執行。'
            )
            return

        translation_base_ee, rotation_base_ee = self.transform_to_arrays(
            base_ee_transform
        )
        translation_camera_tag, rotation_camera_tag = self.transform_to_arrays(
            camera_tag_transform
        )

        self.rotations_gripper_to_base.append(rotation_base_ee)
        self.translations_gripper_to_base.append(translation_base_ee)
        self.rotations_target_to_camera.append(rotation_camera_tag)
        self.translations_target_to_camera.append(translation_camera_tag)
        print(f'✅ 成功記錄第 {len(self.translations_gripper_to_base)} 組數據！')

    @staticmethod
    def rotation_angle_degrees(rotation_matrix):
        """Return the unsigned angle represented by a rotation matrix."""
        cosine = np.clip((np.trace(rotation_matrix) - 1.0) / 2.0, -1.0, 1.0)
        return np.degrees(np.arccos(cosine))

    def rotation_span_degrees(self):
        """Return the largest relative gripper rotation in the sample set."""
        largest_angle = 0.0
        for index, first_rotation in enumerate(self.rotations_gripper_to_base):
            for second_rotation in self.rotations_gripper_to_base[index + 1:]:
                relative_rotation = first_rotation.T @ second_rotation
                largest_angle = max(
                    largest_angle,
                    self.rotation_angle_degrees(relative_rotation),
                )
        return largest_angle

    def target_pose_residuals(self, rotation_gripper_camera, translation_gripper_camera):
        """Measure how stationary the target appears in the robot base frame."""
        transform_gripper_camera = np.eye(4)
        transform_gripper_camera[:3, :3] = rotation_gripper_camera
        transform_gripper_camera[:3, 3] = translation_gripper_camera.reshape(3)

        target_transforms = []
        for rotation_base_gripper, translation_base_gripper, \
                rotation_camera_target, translation_camera_target in zip(
                    self.rotations_gripper_to_base,
                    self.translations_gripper_to_base,
                    self.rotations_target_to_camera,
                    self.translations_target_to_camera,
                ):
            transform_base_gripper = np.eye(4)
            transform_base_gripper[:3, :3] = rotation_base_gripper
            transform_base_gripper[:3, 3] = translation_base_gripper.reshape(3)

            transform_camera_target = np.eye(4)
            transform_camera_target[:3, :3] = rotation_camera_target
            transform_camera_target[:3, 3] = translation_camera_target.reshape(3)
            target_transforms.append(
                transform_base_gripper
                @ transform_gripper_camera
                @ transform_camera_target
            )

        target_translations = np.array([
            transform[:3, 3] for transform in target_transforms
        ])
        mean_translation = np.mean(target_translations, axis=0)
        translation_rms = np.sqrt(np.mean(np.sum(
            (target_translations - mean_translation) ** 2,
            axis=1,
        )))

        summed_rotation = sum(
            transform[:3, :3] for transform in target_transforms
        )
        left, _, right_transpose = np.linalg.svd(summed_rotation)
        mean_rotation = left @ right_transpose
        if np.linalg.det(mean_rotation) < 0:
            left[:, -1] *= -1
            mean_rotation = left @ right_transpose
        rotation_errors = [
            self.rotation_angle_degrees(mean_rotation.T @ transform[:3, :3])
            for transform in target_transforms
        ]
        rotation_rms_degrees = np.sqrt(np.mean(np.square(rotation_errors)))
        return translation_rms, rotation_rms_degrees

    def calculate(self) -> None:
        """Calculate, validate, and print the gripper <- camera transform."""
        sample_count = len(self.translations_gripper_to_base)
        if sample_count < 3:
            print('⚠️ 至少需要 3 組數據；實務上建議收集 10～20 組。')
            return

        rotation_span = self.rotation_span_degrees()
        print(f'姿態最大相對旋轉：{rotation_span:.1f}°')
        if rotation_span < self.minimum_rotation_span_degrees:
            print(
                '❌ 姿態旋轉變化不足；請讓夾爪繞至少兩個不同軸旋轉，'
                f'且跨度超過 {self.minimum_rotation_span_degrees:.1f}°。'
            )
            return

        print(f'🔄 使用 {sample_count} 組數據比較多種手眼校正方法...')
        methods = {
            'Tsai': cv2.CALIB_HAND_EYE_TSAI,
            'Park': cv2.CALIB_HAND_EYE_PARK,
            'Horaud': cv2.CALIB_HAND_EYE_HORAUD,
            'Andreff': cv2.CALIB_HAND_EYE_ANDREFF,
            'Daniilidis': cv2.CALIB_HAND_EYE_DANIILIDIS,
        }
        candidates = []
        for method_name, method in methods.items():
            try:
                rotation_camera_to_gripper, translation_camera_to_gripper = (
                    cv2.calibrateHandEye(
                        self.rotations_gripper_to_base,
                        self.translations_gripper_to_base,
                        self.rotations_target_to_camera,
                        self.translations_target_to_camera,
                        method=method,
                    )
                )
            except cv2.error as exc:
                self.get_logger().warning(f'{method_name} failed: {exc}')
                continue

            if (
                rotation_camera_to_gripper is None
                or translation_camera_to_gripper is None
                or not np.all(np.isfinite(rotation_camera_to_gripper))
                or not np.all(np.isfinite(translation_camera_to_gripper))
            ):
                continue
            translation_rms, rotation_rms_degrees = self.target_pose_residuals(
                rotation_camera_to_gripper,
                translation_camera_to_gripper,
            )
            mount_distance = np.linalg.norm(translation_camera_to_gripper)
            score = translation_rms + np.radians(rotation_rms_degrees) * 0.05
            candidates.append((
                score,
                method_name,
                rotation_camera_to_gripper,
                translation_camera_to_gripper,
                mount_distance,
                translation_rms,
                rotation_rms_degrees,
            ))
            print(
                f'  {method_name:10s}: mount={mount_distance:.3f} m, '
                f'target RMS={translation_rms:.4f} m / '
                f'{rotation_rms_degrees:.2f}°'
            )

        if not candidates:
            print('❌ 所有校正方法均失敗；請重新收集姿態。')
            return

        (
            _, method_name, rotation_camera_to_gripper,
            translation_camera_to_gripper, mount_distance,
            translation_rms, rotation_rms_degrees,
        ) = min(candidates, key=lambda candidate: candidate[0])

        if (
            mount_distance > self.maximum_mount_distance
            or translation_rms > self.maximum_target_translation_rms
            or rotation_rms_degrees > self.maximum_target_rotation_rms_degrees
        ):
            print(
                f'❌ {method_name} 是殘差最小的方法，但結果未通過品質檢查：\n'
                f'   mount distance: {mount_distance:.3f} m '
                f'(上限 {self.maximum_mount_distance:.3f} m)\n'
                f'   target translation RMS: {translation_rms:.4f} m '
                f'(上限 {self.maximum_target_translation_rms:.4f} m)\n'
                f'   target rotation RMS: {rotation_rms_degrees:.2f}° '
                f'(上限 {self.maximum_target_rotation_rms_degrees:.2f}°)'
            )
            return

        print(f'✅ 採用殘差最小且通過檢查的 {method_name} 結果。')

        quaternion = rotation_matrix_to_quaternion(rotation_camera_to_gripper)
        translation = translation_camera_to_gripper.reshape(3)

        print('\n' + '=' * 72)
        print('🎉 計算完成：')
        print(
            'ros2 run tf2_ros static_transform_publisher '
            f'{translation[0]:.5f} {translation[1]:.5f} {translation[2]:.5f} '
            f'{quaternion[0]:.5f} {quaternion[1]:.5f} '
            f'{quaternion[2]:.5f} {quaternion[3]:.5f} '
            f'{self.gripper_frame} {self.camera_frame}'
        )
        print('=' * 72 + '\n')


def spin_node(node) -> None:
    """Spin without printing a traceback during a normal Ctrl-C shutdown."""
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandEyeCalibrator()
    spin_thread = threading.Thread(target=spin_node, args=(node,), daemon=True)
    spin_thread.start()

    print('\n--- 📸 手眼校正程式已啟動 ---')
    print(
        f'Robot: {node.base_frame} <- {node.gripper_frame}\n'
        f'Vision: {node.camera_frame} <- {node.target_frame}'
    )

    try:
        while rclpy.ok():
            value = input(
                '👉 按 [Enter] 記錄姿態、輸入 [c] 計算、輸入 [q] 離開: '
            ).strip().lower()
            if value == 'q':
                break
            if value == 'c':
                node.calculate()
                continue
            node.sample()
    except (EOFError, KeyboardInterrupt):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
