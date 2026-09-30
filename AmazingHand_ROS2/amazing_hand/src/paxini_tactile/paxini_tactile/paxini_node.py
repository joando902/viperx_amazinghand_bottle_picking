#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import time

try:
    # 正常安裝或 PYTHONPATH 指向套件根目錄時的匯入方式。
    from midas_hand_api import PaxiniHandSensor, PaxiniConfig
except ImportError:
    # Docker 將工作空間掛載到 /root/amazing_hand 時，外層專案目錄會先
    # 被 Python 視為 namespace package，實際套件位於下一層。
    from midas_hand_api.midas_hand_api import PaxiniHandSensor, PaxiniConfig

ORDERED_FINGERS = ['thumb', 'index', 'middle', 'ring']

class PaxiniNode(Node):
    def __init__(self):
        super().__init__('paxini_node')

        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baudrate', 921600)
        self.declare_parameter('pub_rate_hz', 50.0)
        self.declare_parameter('response_timeout_s', 3.0)

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        rate_hz = self.get_parameter('pub_rate_hz').get_parameter_value().double_value
        timeout_s = self.get_parameter('response_timeout_s').get_parameter_value().double_value

        self.fz_pub = self.create_publisher(Float32MultiArray, '/tactile/fz', 10)

        self.get_logger().info(f'正在初始化 Paxini 觸覺感測器 ({self.port}, {self.baudrate})...')

        config = PaxiniConfig(
            port=self.port,
            baudrate=self.baudrate,
            response_timeout_s=timeout_s,
            discard_startup_frames=0
        )
        self.sensor = PaxiniHandSensor(config)
        self.sensor.connect()
        
        time.sleep(0.3)
        self.get_logger().info('Paxini 觸覺感測器連線成功！')

        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)

    def timer_callback(self):
        try:
            fz_dict = self.sensor.read_tactile_fz()
            if not fz_dict:
                return

            finger_summary = []
            for finger in ORDERED_FINGERS:
                if finger in fz_dict:
                    arr = np.abs(np.asarray(fz_dict[finger], dtype=np.float32)).flatten()
                    if arr.size >= 3:
                        top3 = np.partition(arr, -3)[-3:]
                        val = float(np.mean(top3))
                    elif arr.size > 0:
                        val = float(np.max(arr))
                    else:
                        val = 0.0
                    finger_summary.append(val)
                else:
                    finger_summary.append(0.0)

            fz_msg = Float32MultiArray()
            fz_msg.data = finger_summary
            self.fz_pub.publish(fz_msg)

        except RuntimeError:
            pass
        except Exception as e:
            self.get_logger().warn(f'讀取感測器資料異常: {e}', throttle_duration_sec=2.0)

    def destroy_node(self):
        if hasattr(self, 'sensor') and self.sensor is not None:
            try:
                self.sensor.disconnect()
            except Exception:
                pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PaxiniNode()
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
