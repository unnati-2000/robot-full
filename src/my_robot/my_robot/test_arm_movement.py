import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys

class ArmMovementTester(Node):

    def __init__(self):
        super().__init__('arm_movement_tester')

        # --- IMPORTANT CONFIGURATION ---
        # Replace with the actual topic name your robot controller subscribes to
        # Common examples: '/joint_trajectory_controller/joint_trajectory', '/arm_controller/joint_trajectory'
        # Check your robot's URDF or controller configuration files.
        trajectory_topic = '/joint_trajectory_controller/joint_trajectory'

        # Replace with the actual joint names for your robot arm in the correct order
        # Check your robot's URDF file (usually defined in <joint> tags).
        self.joint_names = ['joint1', 'joint2', 'joint3'] # Example for a 3-DOF arm

        # Define the target joint positions (in radians)
        self.target_positions = [0.5, -0.5, 1.0] # Example target angles

        # Time (in seconds) to reach the target position from the start
        self.time_to_reach = 2.0
        # --- END OF CONFIGURATION ---

        if len(self.joint_names) != len(self.target_positions):
            self.get_logger().error(f"Error: Number of joint names ({len(self.joint_names)}) does not match number of target positions ({len(self.target_positions)}). Please check configuration.")
            sys.exit(1)


        self.publisher_ = self.create_publisher(
            JointTrajectory,
            trajectory_topic,
            10) # QoS profile depth

        # Use a one-shot timer to publish the command shortly after startup
        self.timer = self.create_timer(1.0, self.publish_trajectory_once)

        self.get_logger().info('Arm Movement Tester node started.')
        self.get_logger().info(f"Will publish one trajectory command for joints: {self.joint_names}")
        self.get_logger().info(f"Target topic: {self.publisher_.topic_name}")
        self.get_logger().info(f"Target positions (radians): {self.target_positions}")
        self.get_logger().info(f"Time to reach target: {self.time_to_reach} seconds")


    def publish_trajectory_once(self):
        """Creates and publishes a single JointTrajectory message."""
        if not self.timer.is_canceled(): # Ensure it only runs once
            msg = JointTrajectory()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = self.joint_names

            point = JointTrajectoryPoint()
            point.positions = [float(p) for p in self.target_positions] # Ensure positions are floats
            point.time_from_start = Duration(sec=int(self.time_to_reach),
                                             nanosec=int((self.time_to_reach % 1) * 1e9))

            msg.points.append(point)

            self.publisher_.publish(msg)
            self.get_logger().info(f'Published trajectory command: {point.positions}')

            # Cancel the timer so it doesn't publish again
            self.timer.cancel()
            self.get_logger().info('Command sent. Timer cancelled.')
            # Optional: shutdown node after sending command
            # self.get_logger().info('Shutting down node.')
            # rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    arm_movement_tester = ArmMovementTester()

    try:
        rclpy.spin(arm_movement_tester)
    except KeyboardInterrupt:
        arm_movement_tester.get_logger().info('Keyboard interrupt detected, shutting down.')
    except SystemExit:
        arm_movement_tester.get_logger().info('Node exited.')
    finally:
        # Ensure node is destroyed cleanly
        if rclpy.ok():
            arm_movement_tester.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main() 
