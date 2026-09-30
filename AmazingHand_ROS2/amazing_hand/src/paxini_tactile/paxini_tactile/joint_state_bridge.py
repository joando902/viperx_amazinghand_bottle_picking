import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

class JointStateBridge(Node):
    def __init__(self):
        super().__init__('joint_state_bridge')
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/hand/cmd_positions',
            self.cmd_callback,
            10
        )
        self.pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_names = [
            'joint_index_mcp', 'joint_index_pip',
            'joint_middle_mcp', 'joint_middle_pip',
            'joint_ring_mcp', 'joint_ring_pip',
            'joint_thumb_mcp', 'joint_thumb_pip'
        ]
        self.get_logger().info('Joint State Bridge 節點已啟動，準備同步 URDF 姿態！')

    def cmd_callback(self, msg: Float32MultiArray):
        if len(msg.data) >= 8:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = self.joint_names
            js.position = list(msg.data[:8])
            self.pub.publish(js)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

