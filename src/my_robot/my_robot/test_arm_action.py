#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class ArmActionTester(Node):

    def __init__(self):
        super().__init__('arm_action_tester')

        # --- IMPORTANT CONFIGURATION ---
        # Targeting the arm_controller's action server
        action_name = '/arm_controller/follow_joint_trajectory'

        # These joint names are specific to your robot as defined in servo_controller.yaml
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        # Define the target positions (in radians) - modify as needed
        self.target_positions = [0.5, 0.2, 0.3, 0.4, 0.5]

        # Time (in seconds) to reach the target position from the start
        self.time_to_reach = 2.0
        # --- END OF CONFIGURATION ---

        # Validate configuration
        if len(self.joint_names) != len(self.target_positions):
            self.get_logger().error(f"Error: Number of joint names ({len(self.joint_names)}) " + 
                                    f"does not match number of target positions ({len(self.target_positions)}).")
            return

        # Create action client
        self.action_client = ActionClient(self, FollowJointTrajectory, action_name)
        
        # Wait for the action server to be available
        self.get_logger().info(f'Waiting for action server {action_name}...')
        self.action_client.wait_for_server()
        
        self.get_logger().info('Server found! Sending goal...')
        self.send_goal()

    def send_goal(self):
        """Send a trajectory goal to the action server."""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Set joint names in the goal
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Set the goal timestamp to now
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Create waypoint
        point = JointTrajectoryPoint()
        point.positions = self.target_positions
        point.velocities = [0.0] * len(self.joint_names)  # Zero velocity at goal
        point.accelerations = [0.0] * len(self.joint_names)  # Zero acceleration at goal
        point.time_from_start = Duration(sec=int(self.time_to_reach),
                                         nanosec=int((self.time_to_reach % 1) * 1e9))
        
        # Add the point to the trajectory
        goal_msg.trajectory.points.append(point)
        
        # Send the goal
        self.get_logger().info(f'Sending trajectory goal with positions: {self.target_positions}')
        self.future = self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # Add callback for when the future is complete
        self.future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """Called when the action server responds to our goal request."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted!')
        
        # Request to be notified when the goal is complete
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        """Called when the action is complete."""
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info('Goal succeeded!')
        else:
            self.get_logger().error(f'Goal failed with status: {status}')
        
        # Shut down after receiving the result
        self.get_logger().info('Shutting down node.')
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        """Called when feedback is received from the action server."""
        feedback = feedback_msg.feedback
        # Optionally process feedback 
        # self.get_logger().info(f'Received feedback: {feedback}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmActionTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # Explicit cleanup not necessary due to shutdown in the result callback
        # but included for robustness
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 
