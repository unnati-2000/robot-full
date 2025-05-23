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
        Perform a 3-minute dance sequence with various patterns including snake-like movements,
        wave patterns, and synchronized arm-robot movements.
        """
        try:
            start_time = time.time()
            dance_duration = 180  # 3 minutes in seconds
            
            # 1. Start with arm in home position
            self.get_logger().info("Starting dance sequence - Moving to home position")
            self.move_arm(self.home_position)
            time.sleep(2.0)

            while time.time() - start_time < dance_duration:
                # Randomly select a dance pattern
                pattern = (int(time.time()) % 5)  # Cycle through 5 patterns
                
                if pattern == 0:
                    # Snake-like sequential movement
                    self.get_logger().info("Performing snake-like sequential movement")
                    self.move_robot(0.2, 0.0, 1.5)
                    for joint in range(1, 7):
                        self.move_arm({joint: 700})
                        time.sleep(0.3)
                    
                    self.move_robot(-0.2, 0.0, 1.5)
                    for joint in range(6, 0, -1):
                        self.move_arm({joint: 300})
                        time.sleep(0.3)

                elif pattern == 1:
                    # Wave pattern
                    self.get_logger().info("Performing wave pattern")
                    for _ in range(3):
                        self.move_robot(0.15, 0.1, 1.0)
                        self.move_arm({2: 700, 3: 600, 4: 500})
                        time.sleep(0.5)
                        self.move_arm({2: 300, 3: 400, 4: 700})
                        time.sleep(0.5)
                    
                    for _ in range(3):
                        self.move_robot(0.15, -0.1, 1.0)
                        self.move_arm({2: 700, 3: 600, 4: 500})
                        time.sleep(0.5)
                        self.move_arm({2: 300, 3: 400, 4: 700})
                        time.sleep(0.5)

                elif pattern == 2:
                    # Spiral dance
                    self.get_logger().info("Performing spiral dance")
                    for i in range(4):
                        angular_speed = 0.2 + (i * 0.1)
                        self.move_robot(0.15, angular_speed, 2.0)
                        for joint in range(1, 7):
                            self.move_arm({joint: 500 + (100 if i % 2 == 0 else -100)})
                            time.sleep(0.2)

                elif pattern == 3:
                    # Zigzag with arm wave
                    self.get_logger().info("Performing zigzag with arm wave")
                    for _ in range(4):
                        self.move_robot(0.2, 0.3, 1.0)
                        self.move_arm({1: 600, 2: 700, 3: 400})
                        time.sleep(0.5)
                        
                        self.move_robot(0.2, -0.3, 1.0)
                        self.move_arm({1: 400, 2: 300, 3: 600})
                        time.sleep(0.5)

                elif pattern == 4:
                    # Circular wave
                    self.get_logger().info("Performing circular wave")
                    for _ in range(2):
                        self.move_robot(0.15, 0.4, 2.0)
                        for joint in range(1, 7):
                            self.move_arm({joint: 700})
                            time.sleep(0.2)
                        
                        self.move_robot(0.15, -0.4, 2.0)
                        for joint in range(6, 0, -1):
                            self.move_arm({joint: 300})
                            time.sleep(0.2)

                # Brief pause between patterns
                time.sleep(1.0)
                
                # Return to home position periodically
                if int(time.time() - start_time) % 30 == 0:  # Every 30 seconds
                    self.get_logger().info("Returning to home position")
                    self.move_arm(self.home_position)
                    time.sleep(2.0)

            # Final return to home position
            self.get_logger().info("Dance sequence complete - Returning to home position")
            self.move_arm(self.home_position)
            time.sleep(2.0)

        except KeyboardInterrupt:
            self.get_logger().info("Dance interrupted by user")
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