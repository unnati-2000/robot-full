#!/usr/bin/env python3

import sys
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import argparse


class CameraNode(Node):
    def __init__(self, save_path='/home/ubuntu/captured_images', num_max=1):
        super().__init__('camera_node')
        # Subscribe to the RGB camera topic
        self.cam_subscription = self.create_subscription(
            Image, 
            '/depth_cam/rgb/image_raw', 
            self.image_callback, 
            1
        )
        self.cv_bridge = CvBridge()
        self.save_path = save_path
        os.makedirs(self.save_path, exist_ok=True)
        self.num_max = num_max
        self.save_id = 0
        self.get_logger().info(f'Camera node initialized. Saving images to: {save_path}')
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            image_bgr = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            
            # Display the image
            cv2.imshow("Camera Feed", image_rgb)
            
            # Save the image
            save_file = os.path.join(self.save_path, f'image{self.save_id}.jpg')
            cv2.imwrite(save_file, image_bgr)
            self.get_logger().info(f'Saved image to: {save_file}')
            
            # Update save counter
            self.save_id = (self.save_id + 1) % self.num_max
            
            # Process any OpenCV window events
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Camera node for capturing and saving images')
    parser.add_argument('--save_path', 
                       type=str, 
                       default='/home/ubuntu/captured_images',
                       help='Directory to save captured images')
    parser.add_argument('--num_max', 
                       type=int, 
                       default=1,
                       help='Maximum number of images to keep (cycles after this)')
    
    # Parse known args and ignore unknown
    parsed_args, unknown = parser.parse_known_args()
    
    try:
        # Create and run the node
        node = CameraNode(parsed_args.save_path, parsed_args.num_max)
        rclpy.spin(node)
    except Exception as e:
        print(f'Error running node: {str(e)}')
    finally:
        # Clean up
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 