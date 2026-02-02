#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import cv2
import numpy as np

class MockFilterNode(Node):
    def __init__(self):
        super().__init__('mock_filter_node')
        
        self.bridge = CvBridge()
        
        # Define QoS Profile: Best Effort (matches RealSense default)
        # This prevents the "Incompatible QoS" errors
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers (Output)
        self.depth_pub = self.create_publisher(
            Image, 
            '/filtered_depth_image', 
            self.sensor_qos)
            
        self.info_pub = self.create_publisher(
            CameraInfo, 
            '/filtered_depth_camera_info', 
            self.sensor_qos)
        
        # Subscribers (Input)
        self.depth_sub = self.create_subscription(
            Image, 
            '/camera/camera/aligned_depth_to_color/image_raw', 
            self.depth_callback, 
            self.sensor_qos) # Must match RealSense QoS
            
        self.info_sub = self.create_subscription(
            CameraInfo, 
            '/camera/camera/aligned_depth_to_color/camera_info', 
            self.info_callback, 
            self.sensor_qos)
            
        self.latest_info = None
        self.get_logger().info("Mock Filter Node Started with BEST_EFFORT QoS.")

    def info_callback(self, msg):
        self.latest_info = msg

    def depth_callback(self, msg):
        if self.latest_info is None:
            return

        try:
            # 1. Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
            
            # 2. Create a Mask (Black everywhere, White square in middle)
            height, width = cv_image.shape
            mask = np.zeros((height, width), dtype=np.uint8)
            
            # Draw square in center (150x150 pixels)
            center_x, center_y = width // 2, height // 2
            start_x, start_y = center_x - 75, center_y - 75
            end_x, end_y = center_x + 75, center_y + 75
            
            # Fill the square with white (255)
            mask[start_y:end_y, start_x:end_x] = 255
            
            # 3. Apply Mask to Depth Image (Bitwise AND)
            filtered_depth = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            
            # 4. Convert back to ROS message
            filtered_msg = self.bridge.cv2_to_imgmsg(filtered_depth, encoding="16UC1")
            
            # 5. CRITICAL: Sync Timestamps exactly
            filtered_msg.header = msg.header
            
            current_info = self.latest_info
            current_info.header = msg.header # Sync info header too
            
            # 6. Publish
            self.info_pub.publish(current_info)
            self.depth_pub.publish(filtered_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MockFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()