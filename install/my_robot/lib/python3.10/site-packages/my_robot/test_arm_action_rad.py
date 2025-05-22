#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.msg import JointTolerance

class ArmActionTester(Node):

    def __init__(self):
        super().__init__('arm_action_tester')

        # --- IMPORTANT CONFIGURATION ---
        # Targeting the arm_controller's action server
        action_name = '/arm_controller/follow_joint_trajectory'

        # These joint names are specific to your robot as defined in servo_controller.yaml
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']

        # Define the target positions in RADIANS (not raw values)
        # Using modest values for safety (approximately -0.5 to 0.5 radians)
        self.target_positions = [0.3, 0.1, -0.2, 0.15, -0.1]  # In radians

        # Time (in seconds) to reach the target position from the start
        self.time_to_reach = 5.0
        # --- END OF CONFIGURATION ---

        # Validate configuration
        if len(self.joint_names) != len(self.target_positions):
            self.get_logger().error(f"Error: Number of joint names ({len(self.joint_names)}) " + 
                                    f"does not match number of target positions ({len(self.target_positions)}).")
            return

        # Create action client
        self.action_client = ActionClient(self, FollowJointTrajectory, action_name)
        
        # Wait for the action server to be available with timeout
        self.get_logger().info(f'Waiting for action server {action_name}...')
        server_found = self.action_client.wait_for_server(timeout_sec=5.0)
        
        if not server_found:
            self.get_logger().error("Action server not available after timeout!")
            return
            
        self.get_logger().info('Server found! Sending goal...')
        self.send_goal()

    def send_goal(self):
        """Send a trajectory goal to the action server."""
        goal_msg = FollowJointTrajectory.Goal()
        
        # Set joint names in the goal
        goal_msg.trajectory.joint_names = self.joint_names
        
        # Set the goal timestamp to now
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        
        # Create waypoint - use two points: initial and final position
        # First point - "current" position to start from (0 radians is a safe start)
        initial_point = JointTrajectoryPoint()
        initial_point.positions = [0.0, 0.0, 0.0, 0.0, 0.0]  # All zeros in radians
        initial_point.velocities = [0.0] * len(self.joint_names)
        initial_point.accelerations = [0.0] * len(self.joint_names)
        initial_point.time_from_start = Duration(sec=0, nanosec=0)
        
        # Final point - target position (MUST BE FLOATS)
        final_point = JointTrajectoryPoint()
        final_point.positions = [float(p) for p in self.target_positions]  # Convert to float explicitly
        final_point.velocities = [0.0] * len(self.joint_names)
        final_point.accelerations = [0.0] * len(self.joint_names)
        final_point.time_from_start = Duration(sec=int(self.time_to_reach),
                                              nanosec=int((self.time_to_reach % 1) * 1e9))
        
        # Add points to the trajectory
        goal_msg.trajectory.points.append(initial_point)
        goal_msg.trajectory.points.append(final_point)
        
        # Set goal tolerance - be more permissive (in radians)
        for joint in self.joint_names:
            tolerance = JointTolerance()
            tolerance.name = joint
            tolerance.position = 0.1  # 0.1 radian tolerance
            tolerance.velocity = 0.1
            tolerance.acceleration = 0.1
            goal_msg.goal_tolerance.append(tolerance)
            
        # Set goal time tolerance
        goal_msg.goal_time_tolerance = Duration(sec=1, nanosec=0)
        
        # Send the goal
        self.get_logger().info(f'Sending trajectory goal with positions: {final_point.positions}')
        self.future = self.action_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.feedback_callback
        )
        
        # Add callback for when the future is complete
        self.future.add_done_callback(self.goal_response_callback)
        
        # Set a timer to handle potential timeout if no response is received
        self.response_timer = self.create_timer(10.0, self.response_timeout_callback)
    
    def response_timeout_callback(self):
        """Called if we don't get a response in time."""
        self.get_logger().error("No response received from the action server after 10 seconds!")
        self.get_logger().info("The server might have received the goal but not responded.")
        self.get_logger().info("Continuing to wait for action completion...")
        # Cancel this timer so it doesn't fire again
        self.response_timer.cancel()
    
    def goal_response_callback(self, future):
        """Called when the action server responds to our goal request."""
        self.response_timer.cancel()  # Cancel timeout timer
        
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by the action server!')
            self.get_logger().error('The arm will not move. Shutting down.')
            rclpy.shutdown()
            return
        
        self.get_logger().info('Goal accepted by the action server!')
        self.get_logger().info('The arm should start moving now...')
        
        # Request to be notified when the goal is complete
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.get_result_callback)
        
        # Set a timer to keep the node alive longer, regardless of result
        self.shutdown_timer = self.create_timer(60.0, self.delayed_shutdown)
    
    def delayed_shutdown(self):
        """Shutdown node after a delay to allow seeing all logs."""
        self.get_logger().info("Timed shutdown of node.")
        rclpy.shutdown()
    
    def get_result_callback(self, future):
        """Called when the action is complete."""
        try:
            result = future.result().result
            status = future.result().status
            
            status_dict = {
                1: "ACTIVE",
                2: "PREEMPTED", 
                3: "ABORTED",
                4: "SUCCEEDED",
                5: "CANCELED",
                6: "REJECTED"
            }
            
            status_str = status_dict.get(status, f"UNKNOWN({status})")
            
            self.get_logger().info(f'Final status: {status_str}')
            
            if status == 4:  # SUCCEEDED
                self.get_logger().info('Goal succeeded! The arm should have moved to the target position.')
            else:
                self.get_logger().error(f'Goal failed with status: {status_str}')
                if hasattr(result, 'error_string') and result.error_string:
                    self.get_logger().error(f'Error: {result.error_string}')
                
            # Don't shut down yet - keep node alive to see the logs
            self.get_logger().info("Node will remain active for 30 more seconds...")
            
        except Exception as e:
            self.get_logger().error(f'Exception in result callback: {str(e)}')
    
    def feedback_callback(self, feedback_msg):
        """Called when feedback is received from the action server."""
        feedback = feedback_msg.feedback
        # Log position feedback to see what's happening
        self.get_logger().info(f'Feedback received - Actual positions: {feedback.actual.positions}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmActionTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down.')
    finally:
        # Clean shutdown
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 
