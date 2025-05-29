#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import torch
import torchvision.transforms as transforms
from PIL import Image as PILImage
import numpy as np
import os

class BirdClassifierNode(Node):
    """
    ROS2 node that subscribes to camera feed and performs real-time bird classification.
    Input: Camera feed from /depth_cam/rgb/image_raw topic
    Output: Displays camera feed with classification results
    """
    def __init__(self):
        super().__init__('bird_classifier_node')
        
        # Subscribe to the RGB camera topic
        self.cam_subscription = self.create_subscription(
            Image, 
            '/depth_cam/rgb/image_raw', 
            self.image_callback, 
            1
        )
        
        self.cv_bridge = CvBridge()
        self.latest_image = None
        
        # Load the PyTorch model
        self.get_logger().info('Loading PyTorch model...')
        self.model = self.load_model("best_bird_classifier.pth")
        self.get_logger().info('Model loaded successfully')
        
        # Define image preprocessing
        self.transform = transforms.Compose([
            transforms.Resize(256),
            transforms.CenterCrop(224),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            )
        ])
        
        self.get_logger().info('Bird classifier node initialized')
        
    def load_model(self, model_path):
        """
        Loads the trained PyTorch model from the specified path.
        
        Args:
            model_path (str): Path to the .pth model file
            
        Returns:
            torch.nn.Module: Loaded model in evaluation mode
        """
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found at {model_path}")
            
        # Initialize model (adjust architecture as needed)
        model = torch.hub.load('pytorch/vision:v0.10.0', 'resnet50', pretrained=False)
        num_classes = 1000  # Adjust based on your model's number of classes
        
        # Modify the final layer to match your number of classes
        model.fc = torch.nn.Linear(model.fc.in_features, num_classes)
        
        # Load the trained weights
        model.load_state_dict(torch.load(model_path))
        model.eval()
        return model

    def preprocess_image(self, cv_image):
        """
        Preprocesses the OpenCV image for model prediction.
        
        Args:
            cv_image (numpy.ndarray): OpenCV image in BGR format
            
        Returns:
            torch.Tensor: Preprocessed image tensor
        """
        # Convert BGR to RGB
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        
        # Convert to PIL Image
        pil_image = PILImage.fromarray(rgb_image)
        
        # Apply transformations
        image_tensor = self.transform(pil_image)
        return image_tensor.unsqueeze(0)  # Add batch dimension

    def classify_image(self, image_tensor):
        """
        Classifies the input image using the loaded model.
        
        Args:
            image_tensor (torch.Tensor): Preprocessed image tensor
            
        Returns:
            tuple: (predicted_class, confidence_score)
        """
        with torch.no_grad():
            outputs = self.model(image_tensor)
            probabilities = torch.nn.functional.softmax(outputs[0], dim=0)
            predicted_class = torch.argmax(probabilities).item()
            confidence = probabilities[predicted_class].item()
        return predicted_class, confidence

    def image_callback(self, msg):
        """
        Callback function for processing incoming camera images.
        
        Args:
            msg (sensor_msgs.msg.Image): ROS2 Image message
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_image = cv_image
            
            # Preprocess image
            image_tensor = self.preprocess_image(cv_image)
            
            # Classify image
            predicted_class, confidence = self.classify_image(image_tensor)
            
            # Draw results on the image
            result_text = f"Class: {predicted_class}, Confidence: {confidence:.2%}"
            cv2.putText(cv_image, result_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            
            # Display the image
            cv2.imshow("Bird Classification", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        # Create and run the node
        node = BirdClassifierNode()
        rclpy.spin(node)
        
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