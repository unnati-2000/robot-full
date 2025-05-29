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
import select
import tensorflow as tf
import tensorflow_hub as hub


class CameraNode(Node):
    """
    A ROS2 node that captures images from a camera and performs bird detection using TensorFlow.
    Input: Camera feed from /depth_cam/rgb/image_raw topic
    Output: Displays camera feed with bird detection annotations
    """
    def __init__(self, save_path='/home/ubuntu/captured_images', max_images=150):
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
        self.max_images = max_images
        self.save_id = 0
        self.latest_image = None
        
        # Load the TensorFlow model
        self.get_logger().info('Loading TensorFlow model...')
        self.model = hub.load('https://tfhub.dev/google/faster_rcnn/openimages_v4/inception_resnet_v2/1')
        self.get_logger().info('Model loaded successfully')
        
        # Define bird-related classes (from COCO dataset)
        self.bird_classes = ['bird', 'chicken', 'duck', 'eagle', 'owl', 'parrot', 'penguin']
        
        self.get_logger().info(f'Camera node initialized. Will save up to {max_images} images to: {save_path}')
        self.get_logger().info('Press Enter to capture an image. Press Ctrl+C to exit.')
        
    def detect_birds(self, image):
        """
        Detect birds in the given image using the TensorFlow model.
        Input: OpenCV image in BGR format
        Output: List of detected birds with their bounding boxes and confidence scores
        """
        # Convert image to RGB and resize for the model
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image_resized = cv2.resize(image_rgb, (640, 480))
        
        # Convert to tensor and add batch dimension
        image_tensor = tf.convert_to_tensor(image_resized)
        image_tensor = tf.expand_dims(image_tensor, 0)
        
        # Run detection
        results = self.model(image_tensor)
        
        # Process results
        detections = []
        boxes = results['detection_boxes'][0].numpy()
        scores = results['detection_scores'][0].numpy()
        classes = results['detection_class_entities'][0].numpy()
        
        for box, score, class_name in zip(boxes, scores, classes):
            class_name = class_name.decode('utf-8').lower()
            if class_name in self.bird_classes and score > 0.5:
                ymin, xmin, ymax, xmax = box
                detections.append({
                    'class': class_name,
                    'score': float(score),
                    'box': (int(xmin * 640), int(ymin * 480), 
                           int(xmax * 640), int(ymax * 480))
                })
        
        return detections
        
    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            image_bgr = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            image_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
            
            # Store the latest image
            self.latest_image = image_bgr
            
            # Detect birds
            detections = self.detect_birds(image_bgr)
            
            # Draw detections on the image
            for det in detections:
                xmin, ymin, xmax, ymax = det['box']
                cv2.rectangle(image_rgb, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
                label = f"{det['class']}: {det['score']:.2f}"
                cv2.putText(image_rgb, label, (xmin, ymin-10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Display the image
            cv2.imshow("Camera Feed with Bird Detection", image_rgb)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')
    
    def capture_image(self):
        if self.latest_image is None:
            self.get_logger().warn('No image available yet')
            return False
            
        if self.save_id >= self.max_images:
            self.get_logger().info(f'Reached maximum number of images ({self.max_images})')
            return False
            
        try:
            # Save the image
            save_file = os.path.join(self.save_path, f'image{self.save_id:03d}.jpg')
            cv2.imwrite(save_file, self.latest_image)
            self.get_logger().info(f'Saved image {self.save_id + 1}/{self.max_images} to: {save_file}')
            
            # Update save counter
            self.save_id += 1
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error saving image: {str(e)}')
            return False


def main(args=None):
    rclpy.init(args=args)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Camera node for capturing and saving images')
    parser.add_argument('--save_path', 
                       type=str, 
                       default='/home/ubuntu/captured_images',
                       help='Directory to save captured images')
    parser.add_argument('--max_images', 
                       type=int, 
                       default=150,
                       help='Maximum number of images to capture')
    
    # Parse known args and ignore unknown
    parsed_args, unknown = parser.parse_known_args()
    
    try:
        # Create and run the node
        node = CameraNode(parsed_args.save_path, parsed_args.max_images)
        
        # Main loop
        while rclpy.ok() and node.save_id < node.max_images:
            rclpy.spin_once(node, timeout_sec=0.1)  # Process callbacks
            
            # Check for Enter key press
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                line = sys.stdin.readline()
                if line.strip() == '':  # Enter was pressed
                    if not node.capture_image():
                        break
                        
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt detected, shutting down.')
    except Exception as e:
        node.get_logger().error(f'Error running node: {str(e)}')
    finally:
        # Clean up
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 