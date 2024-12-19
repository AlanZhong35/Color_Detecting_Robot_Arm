#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from std_msgs.msg import String

class ColorDetectionNode(Node):
    def __init__(self):
        super().__init__('color_detection_node')
        
        # Publisher for detected colors
        self.color_publisher = self.create_publisher(String, 'detected_color', 10)
        
        # Timer to check camera every 0.5 seconds
        self.timer = self.create_timer(0.5, self.detect_color)
        
        # Open the default camera (usually the first webcam)
        self.cap = cv2.VideoCapture(0)
        
        # Check if camera opened successfully
        if not self.cap.isOpened():
            self.get_logger().error('Could not open camera')
    
    def detect_color(self):
        # Capture frame-by-frame
        ret, frame = self.cap.read()
        
        if not ret:
            self.get_logger().warn('Cannot receive frame')
            return
        
        # Get the center region of the frame (about 25% of the frame)
        height, width = frame.shape[:2]
        center_x_start = int(width * 0.375)
        center_x_end = int(width * 0.625)
        center_y_start = int(height * 0.375)
        center_y_end = int(height * 0.625)
        
        center_region = frame[center_y_start:center_y_end, center_x_start:center_x_end]
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        
        # Define color ranges in HSV
        # Red (wraps around in HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # Blue range
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([140, 255, 255])
        
        # Create masks for red and blue
        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 + mask_red2
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
        
        # Count pixels for each color
        red_pixels = np.sum(mask_red)
        blue_pixels = np.sum(mask_blue)
        
        # Publish color if significant color detected
        color_msg = String()
        if red_pixels > 1000:
            color_msg.data = 'red'
            self.color_publisher.publish(color_msg)
            self.get_logger().info('Detected RED')
        elif blue_pixels > 1000:
            color_msg.data = 'blue'
            self.color_publisher.publish(color_msg)
            self.get_logger().info('Detected BLUE')
        else:
            color_msg.data = 'none'
            self.color_publisher.publish(color_msg)
            self.get_logger().info('Detected None')
        
        # Optional: Show the center region for debugging
        # cv2.imshow('Center Region', center_region)
        # cv2.waitKey(1)
        
        # Draw a marker at the center
        # height, width = frame.shape[:2]
        # center_x = width // 2
        # center_y = height // 2
        
        # # Draw cross at the center
        # cv2.drawMarker(frame, (center_x, center_y), (0, 255, 0), 
        #                cv2.MARKER_CROSS, 20, 2)
        
        # # Put text for color detection
        # cv2.putText(frame, f"Center Color: {color_msg.data}", 
        #             (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, 
        #             (255, 255, 255), 2, cv2.LINE_AA)
        
        # # Display the frame
        # cv2.imshow('Center Color Detection', frame)
        # cv2.waitKey(1)
       
    
    def __del__(self):
        # Release the camera when the node is destroyed
        if hasattr(self, 'cap'):
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetectionNode()
    
    try:
        rclpy.spin(color_detection_node)
    except KeyboardInterrupt:
        pass
    finally:
        color_detection_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()