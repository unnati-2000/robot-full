#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from servo_controller_msgs.msg import ServosPosition, ServoPosition
import time

class ArmAndMovementNode(Node):
    """
    A ROS2 node that combines arm movement with robot movement.
    This class provides methods to control both the robotic arm and the robot's base movement.
    Input: Commands to move the arm and robot base
    Output: Publishes commands to both servo controller and cmd_vel topics
    """
    
    def __init__(self):
        super().__init__('arm_and_movement')
        
        # Create publishers for both arm and movement
        self.arm_publisher = self.create_publisher(ServosPosition, 'servo_controller', 10)
        self.movement_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Define home position for all joints
        self.home_position = {
            1: 500,  # Base joint
            2: 500,  # Shoulder joint
            3: 500,  # Elbow joint
            4: 500,  # Wrist joint
            5: 500,  # Wrist rotation
            6: 500   # Gripper
        }
        
        self.get_logger().info('Arm and Movement node initialized')

    def move_arm(self, joint_positions: dict, duration: float = 1.0):
        """
        Move multiple arm joints simultaneously
        
        Args:
            joint_positions (dict): Dictionary mapping joint IDs to their target positions
            duration (float): Time in seconds for the movement to complete
        """
        msg = ServosPosition()
        msg.position_unit = 'pulse'
        msg.duration = duration
        
        for joint_id, position in joint_positions.items():
            servo = ServoPosition()
            servo.id = joint_id
            servo.position = float(position)
            msg.position.append(servo)
            
        self.arm_publisher.publish(msg)
        self.get_logger().info(f"Moving arm joints: {joint_positions}")

    def move_robot(self, linear_speed: float, angular_speed: float, duration: float):
        """
        Move the robot base
        
        Args:
            linear_speed (float): Forward/backward speed in m/s
            angular_speed (float): Rotational speed in rad/s
            duration (float): Time to move in seconds
        """
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        
        end_time = time.time() + duration
        while time.time() < end_time and rclpy.ok():
            self.movement_publisher.publish(twist)
            time.sleep(0.1)
        
        # Stop the robot
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.movement_publisher.publish(twist)
        self.get_logger().info(f"Robot movement complete: linear={linear_speed}, angular={angular_speed}")

    def perform_combined_movement(self):
        """
        Perform a sequence of combined arm and robot movements in a snake-like dance pattern.
        Moves each joint sequentially while the robot moves forward and backward.
        """
        try:
            # 1. Start with arm in home position
            self.get_logger().info("Moving arm to home position")
            self.move_arm(self.home_position)
            time.sleep(2.0)

            # 2. Move forward while doing sequential joint movements
            self.get_logger().info("Moving forward with sequential joint movements")
            self.move_robot(0.2, 0.0, 1.5)  # Move forward for 1.5 seconds
            
            # Sequential joint movements while moving forward
            self.move_arm({1: 700})  # Base joint
            time.sleep(0.5)
            self.move_arm({2: 700})  # Shoulder joint
            time.sleep(0.5)
            self.move_arm({3: 700})  # Elbow joint
            time.sleep(0.5)
            self.move_arm({4: 700})  # Wrist joint
            time.sleep(0.5)
            self.move_arm({5: 700})  # Wrist rotation
            time.sleep(0.5)
            self.move_arm({6: 700})  # Gripper
            time.sleep(0.5)

            # 3. Move backward while reversing the joint movements
            self.get_logger().info("Moving backward with reverse sequential joint movements")
            self.move_robot(-0.2, 0.0, 1.5)  # Move backward for 1.5 seconds
            
            # Reverse sequential joint movements while moving backward
            self.move_arm({6: 300})  # Gripper
            time.sleep(0.5)
            self.move_arm({5: 300})  # Wrist rotation
            time.sleep(0.5)
            self.move_arm({4: 300})  # Wrist joint
            time.sleep(0.5)
            self.move_arm({3: 300})  # Elbow joint
            time.sleep(0.5)
            self.move_arm({2: 300})  # Shoulder joint
            time.sleep(0.5)
            self.move_arm({1: 300})  # Base joint
            time.sleep(0.5)

            # 4. Return to home position
            self.get_logger().info("Returning to home position")
            self.move_arm(self.home_position)
            time.sleep(2.0)

        except KeyboardInterrupt:
            self.get_logger().info("Movement interrupted by user")
            # Ensure we return to home position
            self.move_arm(self.home_position)
            self.move_robot(0.0, 0.0, 0.1)  # Stop robot

def main(args=None):
    rclpy.init(args=args)
    node = ArmAndMovementNode()
    
    try:
        node.perform_combined_movement()
    except Exception as e:
        node.get_logger().error(f'Error during movement: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 