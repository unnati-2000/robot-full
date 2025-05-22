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
        Perform a complex sequence of combined arm and robot movements
        """
        try:
            # 1. Start with arm in home position
            self.get_logger().info("Moving arm to home position")
            self.move_arm(self.home_position)
            time.sleep(2.0)

            # 2. Complex forward movement with arm wave
            self.get_logger().info("Complex forward movement with arm wave")
            for i in range(3):
                # Move forward with varying speeds
                self.move_robot(0.3, 0.0, 1.0)  # Faster forward
                self.move_arm({2: 700, 3: 600, 4: 400})  # Raise and curl
                time.sleep(0.5)
                
                self.move_robot(0.1, 0.0, 1.0)  # Slower forward
                self.move_arm({2: 300, 3: 400, 4: 600})  # Lower and extend
                time.sleep(0.5)

            # 3. Large circle movement with dynamic arm positions
            self.get_logger().info("Large circle movement with dynamic arm positions")
            # First quarter circle
            self.move_robot(0.3, 0.4, 3.0)  # Larger circle, faster movement
            self.move_arm({1: 600, 2: 700, 3: 500, 4: 400, 5: 600})  # Complex arm position
            time.sleep(1.0)
            
            # Second quarter circle
            self.move_robot(0.3, 0.4, 3.0)
            self.move_arm({1: 400, 2: 300, 3: 700, 4: 600, 5: 400})  # Different arm position
            time.sleep(1.0)
            
            # Third quarter circle
            self.move_robot(0.3, 0.4, 3.0)
            self.move_arm({1: 700, 2: 500, 3: 300, 4: 700, 5: 500})  # Another arm position
            time.sleep(1.0)
            
            # Fourth quarter circle
            self.move_robot(0.3, 0.4, 3.0)
            self.move_arm({1: 300, 2: 600, 3: 600, 4: 300, 5: 700})  # Final arm position
            time.sleep(1.0)

            # 4. Zigzag movement with arm dance
            self.get_logger().info("Zigzag movement with arm dance")
            for _ in range(4):
                # Zig right
                self.move_robot(0.2, 0.3, 1.5)
                self.move_arm({1: 600, 2: 700, 3: 300, 4: 500, 5: 600})
                time.sleep(0.5)
                
                # Zag left
                self.move_robot(0.2, -0.3, 1.5)
                self.move_arm({1: 400, 2: 300, 3: 700, 4: 500, 5: 400})
                time.sleep(0.5)

            # 5. Spiral movement with continuous arm motion
            self.get_logger().info("Spiral movement with continuous arm motion")
            for i in range(8):
                # Gradually increase angular speed for spiral effect
                angular_speed = 0.2 + (i * 0.1)
                self.move_robot(0.2, angular_speed, 1.0)
                
                # Continuous arm motion
                arm_positions = [
                    {1: 600, 2: 700, 3: 500, 4: 400, 5: 600},
                    {1: 400, 2: 300, 3: 700, 4: 600, 5: 400},
                    {1: 700, 2: 500, 3: 300, 4: 700, 5: 500},
                    {1: 300, 2: 600, 3: 600, 4: 300, 5: 700}
                ]
                self.move_arm(arm_positions[i % 4])
                time.sleep(0.5)

            # 6. Final flourish - quick movements in all directions
            self.get_logger().info("Final flourish")
            movements = [
                (0.3, 0.0, 1.0, {1: 700, 2: 700, 3: 700}),  # Forward
                (-0.3, 0.0, 1.0, {1: 300, 2: 300, 3: 300}),  # Backward
                (0.0, 0.5, 1.0, {1: 600, 2: 500, 3: 600}),  # Rotate right
                (0.0, -0.5, 1.0, {1: 400, 2: 500, 3: 400})  # Rotate left
            ]
            
            for linear, angular, duration, arm_pos in movements:
                self.move_robot(linear, angular, duration)
                self.move_arm(arm_pos)
                time.sleep(0.5)

            # 7. Return to home position
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
