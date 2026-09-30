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

"""Launch YOLOv8 bottle detection without commanding the robot."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    model_path = LaunchConfiguration('model_path')
    confidence = LaunchConfiguration('confidence')
    inference_rate = LaunchConfiguration('inference_rate')
    maximum_depth = LaunchConfiguration('maximum_depth')
    depth_percentile = LaunchConfiguration('depth_percentile')

    detector = Node(
        package='interbotix_xsarm_perception',
        executable='bottle_detector.py',
        name='bottle_detector',
        output='screen',
        parameters=[{
            'model_path': model_path,
            'confidence': ParameterValue(confidence, value_type=float),
            'inference_rate': ParameterValue(inference_rate, value_type=float),
            'maximum_depth': ParameterValue(maximum_depth, value_type=float),
            'depth_percentile': ParameterValue(
                depth_percentile,
                value_type=float,
            ),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value='/home/interbotix_ws/yolov8n.pt',
            description='Path to a YOLOv8 COCO model.',
        ),
        DeclareLaunchArgument(
            'confidence',
            default_value='0.50',
            description='Minimum confidence for the bottle class.',
        ),
        DeclareLaunchArgument(
            'inference_rate',
            default_value='5.0',
            description='Maximum CPU inference rate in Hz.',
        ),
        DeclareLaunchArgument(
            'maximum_depth',
            default_value='0.8',
            description='Ignore depth measurements beyond this distance in metres.',
        ),
        DeclareLaunchArgument(
            'depth_percentile',
            default_value='25.0',
            description='Depth percentile used to prefer the bottle foreground.',
        ),
        detector,
    ])
