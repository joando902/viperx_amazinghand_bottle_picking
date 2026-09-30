import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray
import numpy as np

try:
    from rustypot import Scs0009PyController
except ImportError:
    Scs0009PyController = None

URDF_JOINT_NAMES = [
    'joint_index_mcp',  'joint_index_pip',   # 食指
    'joint_middle_mcp', 'joint_middle_pip',  # 中指
    'joint_ring_mcp',   'joint_ring_pip',    # 無名指
    'joint_thumb_mcp',  'joint_thumb_pip'    # 大拇指
]

# MCP (基部) 為 1.0，PIP (指尖) 改為 -1.0 翻正
JOINT_SIGNS = [
    1.0, -1.0,  # index
    1.0, -1.0,  # middle
    1.0, -1.0,  # ring
    1.0, -1.0   # thumb
]

class HandDriverNode(Node):
    def __init__(self):
        super().__init__('amazing_hand_node')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('servo_ids', list(range(1, 9)))

        port = self.get_parameter('port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self.servo_ids = list(
            self.get_parameter('servo_ids').get_parameter_value().integer_array_value
        )

        if len(self.servo_ids) != 8 or len(set(self.servo_ids)) != 8:
            raise ValueError('servo_ids 必須包含 8 個不重複的馬達 ID')

        self.get_logger().info(
            f'正在連線 Amazing Hand 馬達控制器 ({port}, {baudrate})，'
            f'馬達 ID: {self.servo_ids}...'
        )

        if Scs0009PyController is None:
            self.get_logger().error('未找到 rustypot 模組！')
            raise RuntimeError('rustypot missing')

        self.controller = Scs0009PyController(
            serial_port=port,
            baudrate=baudrate,
            timeout=0.5
        )
        
        for servo_id in self.servo_ids:
            try:
                self.controller.write_torque_enable(servo_id, 1)
            except Exception as exc:
                raise RuntimeError(f'無法啟用馬達 ID {servo_id}: {exc}') from exc

        self.get_logger().info('Amazing Hand 8 軸馬達連線成功並已啟用！')

        self._current_positions = [0.0] * 8
        self._last_cmd = None

        self.cmd_sub = self.create_subscription(
            Float32MultiArray,
            '/hand/cmd_positions',
            self.cmd_callback,
            10
        )

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_joint_states)

    def cmd_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 8:
            new_cmd = [round(float(x), 2) for x in msg.data[:8]]
            if self._last_cmd == new_cmd:
                return

            try:
                for index, servo_id in enumerate(self.servo_ids):
                    rad = float(msg.data[index])
                    try:
                        self.controller.write_goal_position(servo_id, rad)
                    except Exception as exc:
                        raise RuntimeError(f'馬達 ID {servo_id}: {exc}') from exc
                    self._current_positions[index] = rad
                self._last_cmd = new_cmd
            except Exception as e:
                self.get_logger().warn(f'寫入馬達指令失敗: {e}', throttle_duration_sec=1.0)

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = URDF_JOINT_NAMES
        msg.position = [float(p * s) for p, s in zip(self._current_positions, JOINT_SIGNS)]
        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HandDriverNode()
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
