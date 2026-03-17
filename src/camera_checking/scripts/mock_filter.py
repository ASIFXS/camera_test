#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import cv2
import numpy as np
import message_filters  # <-- 1. ADD THIS IMPORT

class MockFilterNode(Node):
    def __init__(self):
        super().__init__('mock_filter_node')
        
        self.bridge = CvBridge()
        
        # QoS Profile: Best Effort (matches RealSense default)
        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # QoS Profile: Reliable (Highly recommended for depth_image_proc inputs)
        self.publish_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Publishers (Output) - Changed to Reliable
        self.depth_pub = self.create_publisher(
            Image, 
            '/filtered_depth_image', 
            self.publish_qos)
            
        self.info_pub = self.create_publisher(
            CameraInfo, 
            '/filtered_depth_camera_info', 
            self.publish_qos)
        
        # 2. Subscribers using message_filters
        self.depth_sub = message_filters.Subscriber(
            self, Image, 
            '/camera/camera/aligned_depth_to_color/image_raw', 
            qos_profile=self.sensor_qos) 
            
        self.info_sub = message_filters.Subscriber(
            self, CameraInfo, 
            '/camera/camera/aligned_depth_to_color/camera_info', 
            qos_profile=self.sensor_qos)
            
        # 3. ApproximateTimeSynchronizer connects the two subscribers
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.info_sub], 
            queue_size=10, 
            slop=0.05) # Allows up to 50ms of jitter between messages
        self.ts.registerCallback(self.sync_callback)
            
        self.get_logger().info("Mock Filter Node Started with Approximate Sync.")

    # 4. Combine your old depth_callback and info_callback into one
    def sync_callback(self, depth_msg, info_msg):
        try:
            # 1. Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="16UC1")
            
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
            
            # 5. CRITICAL FIX: Generate ONE exact timestamp and force it on both
            exact_stamp = self.get_clock().now().to_msg()
            
            filtered_msg.header.stamp = exact_stamp
            filtered_msg.header.frame_id = depth_msg.header.frame_id
            
            info_msg.header.stamp = exact_stamp
            info_msg.header.frame_id = depth_msg.header.frame_id 
            
            # 6. Publish perfectly synchronized pair
            self.info_pub.publish(info_msg)
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