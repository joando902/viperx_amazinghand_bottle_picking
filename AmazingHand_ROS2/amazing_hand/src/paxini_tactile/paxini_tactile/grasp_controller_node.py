import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
import numpy as np

class GraspControllerNode(Node):
    def __init__(self):
        super().__init__('grasp_controller_node')

        self.declare_parameter('force_threshold', 1.2)
        self.declare_parameter('step_angle_deg', 2.0)
        self.declare_parameter('control_rate_hz', 25.0)
        self.declare_parameter('use_tactile', True)
        self.declare_parameter('grasp_angle_deg', 90.0)
        self.declare_parameter(
            'middle_positions_deg',
            [3.0, 0.0, -5.0, -8.0, -2.0, 5.0, -12.0, 0.0]
        )

        self.threshold = self.get_parameter('force_threshold').get_parameter_value().double_value
        self.step_angle = self.get_parameter('step_angle_deg').get_parameter_value().double_value
        self.use_tactile = (
            self.get_parameter('use_tactile').get_parameter_value().bool_value
        )
        self.grasp_angle = (
            self.get_parameter('grasp_angle_deg').get_parameter_value().double_value
        )
        rate_hz = self.get_parameter('control_rate_hz').get_parameter_value().double_value
        middle_positions = list(
            self.get_parameter('middle_positions_deg')
            .get_parameter_value().double_array_value
        )
        if len(middle_positions) != 8:
            raise ValueError('middle_positions_deg 必須包含 8 個校正角度')
        if not -35.0 <= self.grasp_angle <= 90.0:
            raise ValueError('grasp_angle_deg 必須介於 -35 與 90 度之間')

        self.middle_pos = np.array(middle_positions, dtype=float)
        # [thumb, index, middle, ring]
        self.finger_angles = np.array([-35.0, -35.0, -35.0, -35.0], dtype=float)
        self.current_forces = np.array([0.0, 0.0, 0.0, 0.0])
        self.braked_flags = [False, False, False, False]
        self.state = 'IDLE'

        self.fz_sub = None
        if self.use_tactile:
            self.fz_sub = self.create_subscription(
                Float32MultiArray, '/tactile/fz', self.fz_callback, 10
            )
        self.task_sub = self.create_subscription(String, '/hand/task_cmd', self.task_callback, 10)
        self.cmd_pub = self.create_publisher(Float32MultiArray, '/hand/cmd_positions', 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.control_loop)

        if self.use_tactile:
            self.get_logger().info(
                f'Grasp Controller 啟動！觸覺模式，接觸煞車閾值: {self.threshold} N'
            )
        else:
            self.get_logger().warning(
                'Grasp Controller 啟動！無觸覺固定角度模式，'
                f'抓握角度: {self.grasp_angle:.1f} 度'
            )

    def fz_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 4:
            self.current_forces = np.array(msg.data[:4])

    def task_callback(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'grasp':
            self.state = 'GRASPING'
            self.braked_flags = [False, False, False, False]
            if self.use_tactile:
                self.get_logger().info('收到指令：開始自適應包覆抓取...')
            else:
                self.get_logger().info(
                    f'收到指令：開始固定角度抓取 ({self.grasp_angle:.1f} 度)...'
                )
        elif cmd == 'open':
            self.state = 'OPENING'
            self.braked_flags = [False, False, False, False]
            self.get_logger().info('收到指令：開始張開...')
        elif cmd == 'stop':
            self.state = 'HOLDING'
            self.get_logger().info('收到指令：保持當前姿態。')

    def control_loop(self):
        if self.state == 'GRASPING':
            if not self.use_tactile:
                reached_target = True
                for i in range(4):
                    if self.finger_angles[i] < self.grasp_angle:
                        self.finger_angles[i] = min(self.grasp_angle, self.finger_angles[i] + self.step_angle)
                        reached_target = False
                self.publish_angles()
                if reached_target:
                    self.state = 'HOLDING'
                    self.get_logger().info(f'無觸覺抓握已到達 {self.grasp_angle:.1f} 度並保持。')
                return
            all_braked = True
            for i in range(4):
                # 如果已經煞車，則維持鎖定
                if self.braked_flags[i]:
                    continue

                # 判定受力是否達標
                if self.current_forces[i] >= self.threshold:
                    self.braked_flags[i] = True
                    self.get_logger().info(f'手指 [{i}] 碰觸物體 (力={self.current_forces[i]:.2f}N)，立即煞車鎖定！')
                elif self.finger_angles[i] < 90.0:
                    # 尚未受力且未到極限，繼續前進一步
                    self.finger_angles[i] = min(90.0, self.finger_angles[i] + self.step_angle)
                    all_braked = False
                else:
                    self.braked_flags[i] = True

            self.get_logger().info(
                f"[GRASP] 力:[T:{self.current_forces[0]:.2f}, I:{self.current_forces[1]:.2f}, M:{self.current_forces[2]:.2f}, R:{self.current_forces[3]:.2f}] | "
                f"角:{np.round(self.finger_angles, 1)}",
                throttle_duration_sec=0.2
            )

            self.publish_angles()

            if all_braked:
                self.state = 'HOLDING'
                self.get_logger().info('所有手指已完成自適應抓握並鎖定！')

        elif self.state == 'OPENING':
            all_opened = True
            open_step = self.step_angle * 4.0
            for i in range(4):
                if self.finger_angles[i] > -35.0:
                    self.finger_angles[i] = max(-35.0, self.finger_angles[i] - open_step)
                    all_opened = False

            self.publish_angles()

            if all_opened:
                self.state = 'IDLE'
                self.get_logger().info('手掌已張開完成。')

    def publish_angles(self):
        thumb_ang  = self.finger_angles[0]
        index_ang  = self.finger_angles[1]
        mid_ang    = self.finger_angles[2]
        ring_ang   = self.finger_angles[3]

        servo_deg = np.array([
            index_ang, -index_ang,
            mid_ang,   -mid_ang,
            ring_ang,  -ring_ang,
            thumb_ang, -thumb_ang
        ])

        target_rad = np.deg2rad(self.middle_pos + servo_deg)
        msg = Float32MultiArray()
        msg.data = target_rad.tolist()
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GraspControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
