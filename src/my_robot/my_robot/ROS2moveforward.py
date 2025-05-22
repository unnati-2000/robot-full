#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class LastForwardNode(Node):
    def __init__(self):
        super().__init__('lastforward')
        # Create a publisher on the 'cmd_vel' topic with a queue size of 10.
        # Try both topic names to see which one works with your robot
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # Uncomment the line below if the above doesn't work
        # self.pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
        
        # Use a timer to run the callback periodically.
        self.timer = self.create_timer(0.1, self.timer_callback)
        # Record the start time.
        self.start_time = self.get_clock().now()
        self.get_logger().info('LastForwardNode initialized')

    def move_in_circle(self, linear_speed, angular_speed, duration):
        twist = Twist()
        twist.linear.x = linear_speed  # Forward
        twist.angular.z = angular_speed  # Turning rate    	
        self.get_logger().info(f"Moving in a circle: linear={linear_speed} m/s, angular={angular_speed} rad/s for {duration} sec")
        
        # Publish the twist message for the specified duration
        end_time = time.time() + duration
        while time.time() < end_time and rclpy.ok():
            self.pub.publish(twist)
            time.sleep(0.1)  # Sleep for 0.1 seconds (10 Hz)
        
        # Stop the robot after the duration
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        self.get_logger().info("Stopped moving in circle")

    def timer_callback(self):
        # Calculate the elapsed time in seconds.
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        # Determine which phase we are in:
        # 0: forward, 1: backward, 2: stop; each lasting 4 seconds.
        phase = int(elapsed // 4) % 3

        twist = Twist()
        if phase == 0:
            # Move forward.
            twist.linear.x = 0.4
            self.get_logger().info("Moving forward")
        elif phase == 1:
            # Move backward.
            twist.linear.x = -0.4
            self.get_logger().info("Moving backward")
        elif phase == 2:
            # Stop.
            twist.linear.x = 0.0
            self.get_logger().info("Stopping")

        # Publish the Twist message.
        self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = LastForwardNode()
    
    # Try moving in a circle first
    node.get_logger().info("Starting circle movement")
    node.move_in_circle(0.2, 0.5, 10.0)  # linear_speed=0.2 m/s, angular_speed=0.5 rad/s, duration=10 seconds
    
    # Then continue with the regular pattern
    node.get_logger().info("Starting regular movement pattern")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
