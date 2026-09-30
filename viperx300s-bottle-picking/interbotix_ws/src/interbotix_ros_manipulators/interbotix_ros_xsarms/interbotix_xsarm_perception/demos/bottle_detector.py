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

"""Detect a COCO bottle and publish its depth-derived 3D surface point."""

import time

from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from ultralytics import YOLO
from visualization_msgs.msg import Marker


class BottleDetector(Node):
    """Run YOLOv8 on RealSense color frames and add aligned depth."""

    def __init__(self):
        super().__init__('bottle_detector')

        self.declare_parameter('model_path', '/home/interbotix_ws/yolov8n.pt')
        self.declare_parameter(
            'color_topic', '/camera/camera/color/image_raw'
        )
        self.declare_parameter(
            'depth_topic', '/camera/camera/aligned_depth_to_color/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic',
            '/camera/camera/aligned_depth_to_color/camera_info',
        )
        self.declare_parameter('base_frame', 'vx300s/base_link')
        self.declare_parameter('confidence', 0.50)
        self.declare_parameter('image_size', 640)
        self.declare_parameter('inference_rate', 5.0)
        self.declare_parameter('maximum_depth', 0.8)
        self.declare_parameter('depth_percentile', 25.0)
        self.declare_parameter('maximum_sync_difference', 0.15)
        self.declare_parameter('marker_scale', 0.05)

        self.model_path = self.get_parameter('model_path').value
        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.confidence = float(self.get_parameter('confidence').value)
        self.image_size = int(self.get_parameter('image_size').value)
        inference_rate = float(self.get_parameter('inference_rate').value)
        self.minimum_inference_period = 1.0 / max(inference_rate, 0.1)
        self.maximum_depth = float(self.get_parameter('maximum_depth').value)
        self.depth_percentile = float(
            self.get_parameter('depth_percentile').value
        )
        self.maximum_sync_difference = float(
            self.get_parameter('maximum_sync_difference').value
        )
        self.marker_scale = float(self.get_parameter('marker_scale').value)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_depth = None
        self.latest_depth_header = None
        self.latest_depth_encoding = None
        self.camera_info = None
        self.last_inference_time = 0.0
        self.last_status_time = 0.0

        self.get_logger().info(f'Loading YOLO model: {self.model_path}')
        self.model = YOLO(self.model_path)

        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.color_topic,
            self.color_callback,
            qos_profile_sensor_data,
        )

        self.annotated_image_publisher = self.create_publisher(
            Image, '/yolo/annotated_image', 10
        )
        self.camera_point_publisher = self.create_publisher(
            PointStamped, '/yolo/bottle_surface_point_camera', 10
        )
        self.base_point_publisher = self.create_publisher(
            PointStamped, '/yolo/bottle_surface_point_base', 10
        )
        self.marker_publisher = self.create_publisher(
            Marker, '/yolo/bottle_marker', 10
        )

        self.get_logger().info(
            'Bottle detector ready. It publishes coordinates only and cannot move the robot.'
        )

    def depth_callback(self, message):
        """Keep the newest depth frame aligned to the color image."""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='passthrough'
            )
            self.latest_depth_header = message.header
            self.latest_depth_encoding = message.encoding
        except Exception as exc:
            self.log_status(f'Cannot convert depth image: {exc}', error=True)

    def camera_info_callback(self, message):
        """Keep the aligned color-camera intrinsics."""
        self.camera_info = message

    def log_status(self, message, error=False):
        """Limit repeated status messages to one every two seconds."""
        now = time.monotonic()
        if now - self.last_status_time < 2.0:
            return
        self.last_status_time = now
        if error:
            self.get_logger().error(message)
        else:
            self.get_logger().warning(message)

    @staticmethod
    def stamp_seconds(header):
        return header.stamp.sec + header.stamp.nanosec * 1e-9

    def depth_metres(self, depth_image, encoding, box):
        """Return a foreground-biased depth from the inner part of a box."""
        height, width = depth_image.shape[:2]
        x1, y1, x2, y2 = box
        box_width = max(x2 - x1, 1)
        box_height = max(y2 - y1, 1)
        # Transparent bottles often return background depth through their
        # centre. Include more of the bottle edges, where stereo depth is more
        # likely to be valid, and prefer the nearer part of the distribution.
        left = int(np.clip(x1 + 0.15 * box_width, 0, width - 1))
        right = int(np.clip(x2 - 0.15 * box_width, left + 1, width))
        top = int(np.clip(y1 + 0.15 * box_height, 0, height - 1))
        bottom = int(np.clip(y2 - 0.15 * box_height, top + 1, height))
        region = np.asarray(depth_image[top:bottom, left:right])

        if encoding in ('16UC1', 'mono16') or region.dtype == np.uint16:
            depths = region.astype(np.float64) * 0.001
        else:
            depths = region.astype(np.float64)

        valid = depths[
            np.isfinite(depths)
            & (depths > 0.10)
            & (depths < self.maximum_depth)
        ]
        if valid.size < 10:
            return None
        percentile = float(np.clip(self.depth_percentile, 0.0, 100.0))
        return float(np.percentile(valid, percentile))

    def point_from_pixel(self, u, v, depth, header):
        """Deproject a color pixel using the aligned CameraInfo matrix."""
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        if fx <= 0.0 or fy <= 0.0:
            return None

        point = PointStamped()
        point.header = header
        point.header.frame_id = self.camera_info.header.frame_id
        point.point.x = (u - cx) * depth / fx
        point.point.y = (v - cy) * depth / fy
        point.point.z = depth
        return point

    def transform_to_base(self, point):
        """Transform an optical-frame point using the latest available TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                point.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.3),
            )
            return do_transform_point(point, transform)
        except TransformException as exc:
            self.log_status(
                f'Cannot transform {point.header.frame_id} to '
                f'{self.base_frame}: {exc}'
            )
            return None

    @staticmethod
    def annotated_image_message(image, header):
        """Build a bgr8 Image without OpenCV-version-dependent cv_bridge code."""
        image = np.ascontiguousarray(image, dtype=np.uint8)
        message = Image()
        message.header = header
        message.height = image.shape[0]
        message.width = image.shape[1]
        message.encoding = 'bgr8'
        message.is_bigendian = False
        message.step = image.shape[1] * 3
        message.data = image.tobytes()
        return message

    def publish_marker(self, point):
        """Show the measured bottle surface point in RViz."""
        marker = Marker()
        marker.header = point.header
        marker.ns = 'yolo_bottle'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point.point
        marker.pose.orientation.w = 1.0
        marker.scale.x = self.marker_scale
        marker.scale.y = self.marker_scale
        marker.scale.z = self.marker_scale
        marker.color.r = 0.1
        marker.color.g = 1.0
        marker.color.b = 0.2
        marker.color.a = 0.9
        marker.lifetime = Duration(seconds=0.5).to_msg()
        self.marker_publisher.publish(marker)

    def color_callback(self, message):
        """Detect the strongest bottle and publish its measured surface point."""
        now = time.monotonic()
        if now - self.last_inference_time < self.minimum_inference_period:
            return
        self.last_inference_time = now

        if (
            self.latest_depth is None
            or self.latest_depth_header is None
            or self.camera_info is None
        ):
            self.log_status('Waiting for aligned depth and CameraInfo.')
            return

        time_difference = abs(
            self.stamp_seconds(message.header)
            - self.stamp_seconds(self.latest_depth_header)
        )
        if time_difference > self.maximum_sync_difference:
            self.log_status(
                f'Color/depth timestamps differ by {time_difference:.3f} s.'
            )
            return

        try:
            color_image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
            results = self.model.predict(
                source=color_image,
                classes=[39],
                conf=self.confidence,
                imgsz=self.image_size,
                device='cpu',
                verbose=False,
            )
        except Exception as exc:
            self.log_status(f'YOLO inference failed: {exc}', error=True)
            return

        result = results[0]
        annotated = result.plot()
        annotated_message = self.annotated_image_message(
            annotated, message.header
        )
        self.annotated_image_publisher.publish(annotated_message)

        if result.boxes is None or len(result.boxes) == 0:
            self.log_status('No bottle detected.')
            return

        confidences = result.boxes.conf.detach().cpu().numpy()
        best_index = int(np.argmax(confidences))
        box = result.boxes.xyxy[best_index].detach().cpu().numpy()
        depth = self.depth_metres(
            self.latest_depth,
            self.latest_depth_encoding,
            box,
        )
        if depth is None:
            self.log_status(
                'Bottle detected, but aligned depth is invalid on the bottle body.'
            )
            return

        u = float((box[0] + box[2]) / 2.0)
        v = float((box[1] + box[3]) / 2.0)
        camera_point = self.point_from_pixel(
            u, v, depth, self.latest_depth_header
        )
        if camera_point is None:
            self.log_status('CameraInfo contains invalid focal lengths.', error=True)
            return

        base_point = self.transform_to_base(camera_point)
        if base_point is None:
            return

        self.camera_point_publisher.publish(camera_point)
        self.base_point_publisher.publish(base_point)
        self.publish_marker(base_point)
        self.get_logger().info(
            'Bottle surface in base_link: '
            f'x={base_point.point.x:.3f}, '
            f'y={base_point.point.y:.3f}, '
            f'z={base_point.point.z:.3f} m; '
            f'confidence={confidences[best_index]:.2f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = BottleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
