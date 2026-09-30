#!/usr/bin/env python3
"""
Complete VX300s controller with 6+1 axes
Based on the WORKING fixed_controller.py
"""

import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from sensor_msgs.msg import JointState
from dynamixel_sdk import *
import time
import math

class WorkingCompleteController(Node):
    def __init__(self):
        super().__init__('working_complete_controller')
        
        # DYNAMIXEL setup (完全照抄working版本)
        self.port_handler = PortHandler('/dev/ttyUSB0')
        self.packet_handler = PacketHandler(2.0)
        
        if not self.port_handler.openPort():
            self.get_logger().error("Failed to open port")
            return
            
        if not self.port_handler.setBaudRate(1000000):
            self.get_logger().error("Failed to set baudrate")
            return
        
        self.get_logger().info("✓ Port opened")
        
        # Complete motor configuration for VX300s (擴展到所有6+1軸)
        self.motors = {
            'waist': {
                'id': 1,
                'center': 2048,
                'ticks_per_radian': 651.74,  # 2048/π
                'min': 0,
                'max': 4095
            },
            'shoulder': {
                'id': 2,
                'center': 1854,  # Different center based on URDF
                'ticks_per_radian': 651.74,
                'min': 841,
                'max': 2867
            },
            'elbow': {
                'id': 4,
                'center': 1996,
                'ticks_per_radian': 651.74,
                'min': 898,
                'max': 3094
            },
            'forearm_roll': {
                'id': 6,
                'center': 2048,
                'ticks_per_radian': 651.74,
                'min': 0,
                'max': 4095
            },
            'wrist_angle': {
                'id': 7,
                'center': 2167,
                'ticks_per_radian': 651.74,
                'min': 830,
                'max': 3504
            },
            'wrist_rotate': {
                'id': 8,
                'center': 2048,
                'ticks_per_radian': 651.74,
                'min': 0,
                'max': 4095
            },
            'gripper': {
                'id': 9,
                'center': 2048,
                'ticks_per_radian': 651.74,
                'min': 0,
                'max': 4095
            }
        }
        
        # Shadow motors IDs
        self.shadow_motors = {
            'shoulder_shadow': 3,
            'elbow_shadow': 5
        }
        
        # Enable torque for all configured motors (包括shadows)
        all_motor_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9]
        for motor_id in all_motor_ids:
            result, _ = self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, 64, 1
            )
            if result == COMM_SUCCESS:
                motor_name = self.get_motor_name(motor_id)
                self.get_logger().info(f"✓ Motor {motor_id} ({motor_name}) torque enabled")
            
            # Set profile velocity for smooth movement
            velocity = 50 if motor_id == 9 else 100  # Slower for gripper
            self.packet_handler.write4ByteTxRx(
                self.port_handler, motor_id, 112, velocity
            )
        
        # Store initial positions as "zero" reference
        self.zero_positions = {}
        for name, config in self.motors.items():
            pos, _, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, config['id'], 132
            )
            self.zero_positions[name] = pos
            self.get_logger().info(f"{name} zero position: {pos}")
        
        # ROS2 Subscribers (照抄working版本的結構)
        self.joint_single_sub = self.create_subscription(
            JointSingleCommand,
            '/vx300s/commands/joint_single',
            self.joint_single_callback,
            10
        )
        
        self.joint_group_sub = self.create_subscription(
            JointGroupCommand,
            '/vx300s/commands/joint_group',
            self.joint_group_callback,
            10
        )
        
        # For Unity control
        self.unity_sub = self.create_subscription(
            JointState,
            '/vx300s/joint_states_control',
            self.unity_callback,
            10
        )
        
        # Publisher for feedback
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/vx300s/joint_states',
            10
        )
        
        self.unity_feedback_pub = self.create_publisher(
            JointState,
            '/vx300s/joint_states_feedback',
            10
        )
        
        # Timer for state publishing
        self.timer = self.create_timer(0.05, self.publish_states)  # 20Hz
        
        self.get_logger().info("✓ Working Complete Controller Ready!")
        self.get_logger().info("Commands:")
        self.get_logger().info("  - /vx300s/commands/joint_single")
        self.get_logger().info("  - /vx300s/commands/joint_group")
        self.get_logger().info("  - /vx300s/joint_states_control (Unity)")
    
    def get_motor_name(self, motor_id):
        """Helper to get motor name from ID"""
        for name, config in self.motors.items():
            if config['id'] == motor_id:
                return name
        if motor_id == 3:
            return 'shoulder_shadow'
        elif motor_id == 5:
            return 'elbow_shadow'
        return f'unknown_{motor_id}'
    
    def joint_single_callback(self, msg):
        """Handle single joint command with CORRECT conversion (照抄working版本)"""
        self.get_logger().info(f"\nCommand: {msg.name} = {msg.cmd:.3f} rad")
        
        if msg.name not in self.motors:
            self.get_logger().warning(f"Unknown joint: {msg.name}")
            return
        
        config = self.motors[msg.name]
        motor_id = config['id']
        
        # Method 1: Absolute position from center (working版本的方法)
        goal_ticks = int(config['center'] + msg.cmd * config['ticks_per_radian'])
        
        # Clamp to limits
        goal_ticks = max(config['min'], min(config['max'], goal_ticks))
        
        # Get current position for logging
        current_pos, _, _ = self.packet_handler.read4ByteTxRx(
            self.port_handler, motor_id, 132
        )
        
        self.get_logger().info(f"  Current: {current_pos} ticks")
        self.get_logger().info(f"  Goal: {goal_ticks} ticks")
        self.get_logger().info(f"  Movement: {goal_ticks - current_pos} ticks")
        
        # Send command
        result, _ = self.packet_handler.write4ByteTxRx(
            self.port_handler, motor_id, 116, goal_ticks
        )
        
        if result == COMM_SUCCESS:
            self.get_logger().info(f"  ✓ Command sent")
            
            # For shoulder and elbow, also move shadow motors
            if msg.name == 'shoulder':
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, 3, 116, goal_ticks  # shoulder_shadow
                )
            elif msg.name == 'elbow':
                self.packet_handler.write4ByteTxRx(
                    self.port_handler, 5, 116, goal_ticks  # elbow_shadow
                )
        else:
            self.get_logger().error(f"  ✗ Failed to send command")
    
    def joint_group_callback(self, msg):
        """Handle joint group command"""
        self.get_logger().info(f"\nGroup command: {msg.name}")
        
        if msg.name == 'arm':
            # Complete arm with all 6 joints
            joints = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
            for i, joint_name in enumerate(joints):
                if i < len(msg.cmd):
                    self.move_joint(joint_name, msg.cmd[i])
        elif msg.name == 'gripper':
            if len(msg.cmd) > 0:
                self.move_joint('gripper', msg.cmd[0])
    
    def unity_callback(self, msg):
        """Handle Unity control - supports 6 or 7 values"""
        # Full 6 arm joints
        arm_joints = ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']
        
        # Move arm joints based on available positions
        for i, joint_name in enumerate(arm_joints):
            if i < len(msg.position):
                self.move_joint(joint_name, msg.position[i])
        
        # Handle gripper if 7th value exists
        if len(msg.position) > 6:
            self.move_joint('gripper', msg.position[6])
    
    def move_joint(self, name, radians):
        """Helper to move a joint (照抄working版本的邏輯)"""
        if name not in self.motors:
            return
        
        config = self.motors[name]
        
        # Special handling for gripper
        if name == 'gripper':
            # Gripper uses different scaling
            # 0.015 (closed) to 0.037 (open) in radians
            # Map to ticks appropriately
            if abs(radians) < 0.1:  # Small values, likely gripper positions
                # Direct mapping for gripper
                goal_ticks = int(2048 + radians * 30000)  # Amplify for gripper
            else:
                # Normal conversion
                goal_ticks = int(config['center'] + radians * config['ticks_per_radian'])
        else:
            # Normal joint conversion
            goal_ticks = int(config['center'] + radians * config['ticks_per_radian'])
        
        goal_ticks = max(config['min'], min(config['max'], goal_ticks))
        
        self.packet_handler.write4ByteTxRx(
            self.port_handler, config['id'], 116, goal_ticks
        )
        
        # Handle shadows
        if name == 'shoulder':
            self.packet_handler.write4ByteTxRx(self.port_handler, 3, 116, goal_ticks)
        elif name == 'elbow':
            self.packet_handler.write4ByteTxRx(self.port_handler, 5, 116, goal_ticks)
    
    def publish_states(self):
        """Publish current joint states (照抄working版本)"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for name, config in self.motors.items():
            pos, result, _ = self.packet_handler.read4ByteTxRx(
                self.port_handler, config['id'], 132
            )
            if result == COMM_SUCCESS:
                # Convert ticks to radians
                radians = (pos - config['center']) / config['ticks_per_radian']
                msg.name.append(name)
                msg.position.append(radians)
        
        self.joint_state_pub.publish(msg)
        self.unity_feedback_pub.publish(msg)
    
    def test_movement(self):
        """Test movement sequence for all joints"""
        self.get_logger().info("\n=== Testing All Joints ===")
        
        # Test each joint individually
        test_sequence = [
            ('waist', 0.5, -0.5),
            ('shoulder', 0.3, -0.3),
            ('elbow', 0.3, -0.3),
            ('forearm_roll', 0.5, -0.5),
            ('wrist_angle', 0.3, -0.3),
            ('wrist_rotate', 0.5, -0.5),
            ('gripper', 0.037, 0.015)  # Open and close
        ]
        
        for joint_name, pos1, pos2 in test_sequence:
            self.get_logger().info(f"\nTesting {joint_name}...")
            self.move_joint(joint_name, pos1)
            time.sleep(1.5)
            self.move_joint(joint_name, pos2)
            time.sleep(1.5)
            self.move_joint(joint_name, 0.0)
            time.sleep(1.0)
        
        self.get_logger().info("\n✓ All joints tested")
    
    def destroy_node(self):
        # Disable torque for all motors
        all_motor_ids = [1, 2, 3, 4, 5, 6, 7, 8, 9]
        for motor_id in all_motor_ids:
            self.packet_handler.write1ByteTxRx(
                self.port_handler, motor_id, 64, 0
            )
        self.port_handler.closePort()
        super().destroy_node()

def main():
    rclpy.init()
    node = WorkingCompleteController()
    
    print("\n" + "="*60)
    print("WORKING COMPLETE CONTROLLER (6+1 axes)")
    print("Based on the working fixed_controller.py")
    print("="*60)
    print("\nTest commands:")
    print("\n1. Test individual joint:")
    print("   ros2 topic pub --once /vx300s/commands/joint_single \\")
    print("     interbotix_xs_msgs/msg/JointSingleCommand \\")
    print("     \"{name: 'wrist_angle', cmd: 0.3}\"")
    print("\n2. Full arm control (6 joints):")
    print("   ros2 topic pub --once /vx300s/commands/joint_group \\")
    print("     interbotix_xs_msgs/msg/JointGroupCommand \\")
    print("     \"{name: 'arm', cmd: [0.0, 0.3, 0.3, 0.0, 0.3, 0.0]}\"")
    print("\n3. Unity control (6 joints):")
    print("   ros2 topic pub /vx300s/joint_states_control \\")
    print("     sensor_msgs/msg/JointState \\")
    print("     \"{position: [0.0, 0.3, 0.3, 0.0, 0.3, 0.0]}\" -r 10")
    print("\n4. Unity control with gripper (7 values):")
    print("   ros2 topic pub /vx300s/joint_states_control \\")
    print("     sensor_msgs/msg/JointState \\")
    print("     \"{position: [0.0, 0.3, 0.3, 0.0, 0.3, 0.0, 0.037]}\" -r 10")
    print("\n5. Test gripper:")
    print("   ros2 topic pub --once /vx300s/commands/joint_single \\")
    print("     interbotix_xs_msgs/msg/JointSingleCommand \\")
    print("     \"{name: 'gripper', cmd: 0.037}\"  # Open")
    print("\nNote: Add '-r 10' for continuous control at 10Hz")
    
    # Optional: run automatic test
    import sys
    if len(sys.argv) > 1 and sys.argv[1] == '--test':
        import threading
        test_thread = threading.Thread(target=lambda: (time.sleep(2), node.test_movement()))
        test_thread.start()
    
    try:
        rclpy.spin(node)  # 關鍵：保持節點運行！
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

#這個可以對全關節做控制