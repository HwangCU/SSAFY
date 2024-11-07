#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np


class RealSenseRGBPublisher(Node):
    def __init__(self):
        super().__init__('rgb_publisher')
        self.publisher_ = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.timer = self.create_timer(0.0333, self.timer_callback)  # 30Hz
        
        # RealSense 초기화
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)

        # cv_bridge 초기화
        self.bridge = CvBridge()
        self.get_logger().info("RealSense RGB Publisher Node has been started.")

    def timer_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            self.get_logger().warn("No color frame available.")
            return

        # 이미지 데이터를 numpy 배열로 변환
        color_image = np.asanyarray(color_frame.get_data())

        # 이미지를 ROS Image 메시지로 변환하여 Publish
        msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
        self.publisher_.publish(msg)
        self.get_logger().info("Published an RGB image frame.")

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    rgb_publisher = RealSenseRGBPublisher()

    try:
        rclpy.spin(rgb_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        rgb_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()