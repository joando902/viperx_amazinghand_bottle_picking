import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import sys

class TactileMonitor(Node):
    def __init__(self):
        super().__init__('tactile_monitor')
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/tactile/fz',
            self.callback,
            10
        )
        self.finger_names = [
            "Thumb (Finger 0)",
            "Index (Finger 1)",
            "Middle (Finger 2)",
            "Ring (Finger 3)"
        ]

    def callback(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return

        raw_forces = list(msg.data[:4])

        sys.stdout.write("\033[H\033[J")
        print("=" * 72)
        print("                PAXINI 4-FINGER TACTILE MONITOR")
        print("=" * 72)
        print(f"{'Finger':<20} | {'Force Fz':<10} | {'Force Bar (Max 40N)':<24} | Status")
        print("-" * 72)

        for i in range(4):
            force = max(0.0, raw_forces[i])
            bar_len = int(min(force / 40.0, 1.0) * 20)
            bar = "#" * bar_len + "-" * (20 - bar_len)
            status = "[ CONTACT ]" if force >= 3.0 else "[  FREE   ]"
            print(f"{self.finger_names[i]:<20} | {force:6.2f} N   | [{bar}]  | {status}")

        print("-" * 72)
        print(">> Press on finger tips to verify if the bar length increases.")
        print(">> Press Ctrl + C to exit.")
        sys.stdout.flush()

def main(args=None):
    rclpy.init(args=args)
    node = TactileMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
