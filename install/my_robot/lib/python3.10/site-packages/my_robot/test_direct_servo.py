#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

# Use correct message type for the topic
from ros_robot_controller_msgs.msg import SetBusServoState, BusServoState

class DirectServoControl(Node):
    def __init__(self):
        super().__init__('direct_servo_control')
        
        # Direct servo control topic
        self.topic_name = '/ros_robot_controller/bus_servo/set_state'
        
        # Create the publisher with the correct message type
        self.publisher = self.create_publisher(SetBusServoState, self.topic_name, 10)
        
        # Wait a moment for the publisher to be established
        time.sleep(1)
        
        # Send test position commands
        self.send_servo_command()
        
    def send_servo_command(self):
        # Create the message based on the correct type
        msg = SetBusServoState()
        
        # Based on servo_controller.yaml, we need to target these IDs
        # joint1 -> ID 1, joint2 -> ID 2, etc.
        servo_ids = [1, 2, 3, 4, 5]
        
        # Use positions in the middle of the range (around 500)
        # The yaml file has init=500, min=1000, max=0 for most servos
        # Try slightly different positions to see movement
        positions = [400, 550, 450, 500, 600]
        
        # Create a BusServoState object
        servo_state = BusServoState()
        
        # Set target_id array field in the BusServoState
        servo_state.target_id = servo_ids
        
        # Set position array field in the BusServoState 
        servo_state.position = positions
        
        # Add the BusServoState to the state array in the SetBusServoState
        msg.state.append(servo_state)
        
        # Set the duration field
        msg.duration = 2.0
            
        # Log what we're sending
        self.get_logger().info(f"Sending position commands to servos {servo_state.target_id}: {servo_state.position}")
        self.get_logger().info(f"Using duration: {msg.duration} seconds")
        
        # Publish the message
        self.publisher.publish(msg)
        self.get_logger().info(f"Command sent to {self.topic_name}")
        
        # Wait and send again after a few seconds
        self.timer = self.create_timer(5.0, self.send_again)
        
    def send_again(self):
        # Cancel the timer so it doesn't fire again
        self.timer.cancel()
        
        # Send slightly different positions as a second test
        msg = SetBusServoState()
        
        servo_ids = [1, 2, 3, 4, 5]
        # Different positions from before
        positions = [500, 450, 550, 400, 500]
        
        # Create and populate the BusServoState
        servo_state = BusServoState()
        servo_state.target_id = servo_ids
        servo_state.position = positions
        
        # Add to message state array
        msg.state.append(servo_state)
        
        # Set duration
        msg.duration = 2.0
        
        self.get_logger().info(f"Sending second position commands: {servo_state.position}")
        self.publisher.publish(msg)
        self.get_logger().info(f"Second command sent. Keeping node alive for 10 more seconds...")
        
        # Shut down after a while
        self.timer = self.create_timer(10.0, self.shutdown)
        
    def shutdown(self):
        self.get_logger().info("Shutting down node.")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DirectServoControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 
