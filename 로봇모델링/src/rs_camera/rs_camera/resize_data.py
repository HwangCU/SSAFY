import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class ResizeData(Node):
    def __init__(self):
        super().__init__('resize_data')
        
        # 구독자 설정 (컬러 이미지)
        self.color_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.color_callback,
            10)
        
        # 구독자 설정 (깊이 이미지)
        self.depth_subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.depth_callback,
            10)
        
        self.color_subscription  # 방지: 콜백이 사용되지 않는다고 경고하지 않도록 설정
        self.depth_subscription  # 방지: 콜백이 사용되지 않는다고 경고하지 않도록 설정

        # CvBridge를 사용하여 ROS 이미지 메시지를 OpenCV 이미지로 변환
        self.bridge = CvBridge()

    def color_callback(self, msg):
        self.get_logger().info('Received color image frame')

        # ROS Image 메시지를 OpenCV 이미지로 변환
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 이미지 사이즈를 640x480으로 변환
        resized_color_image = cv2.resize(color_image, (640, 480))

        # OpenCV로 컬러 이미지 시각화
        cv2.imshow("Resized Color Image", resized_color_image)
        cv2.waitKey(1)

    def depth_callback(self, msg):
        self.get_logger().info('Received depth image frame')

        # ROS Image 메시지를 OpenCV 이미지로 변환 (16UC1 형식)
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # 깊이 값을 시각화하기 위해 0에서 255 사이의 값으로 변환 (정규화)
        depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)

        # 이미지 사이즈를 640x480으로 변환
        resized_depth_image = cv2.resize(depth_image_normalized, (640, 480))

        # 8비트로 변환 후 시각화
        depth_image_8u = np.uint8(resized_depth_image)

        # OpenCV로 깊이 이미지 시각화
        cv2.imshow("Resized Depth Image", depth_image_8u)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    resize_data_node = ResizeData()

    try:
        rclpy.spin(resize_data_node)
    except KeyboardInterrupt:
        resize_data_node.get_logger().info("Node stopped cleanly")
    except Exception as e:
        resize_data_node.get_logger().error(f"Error: {str(e)}")
    finally:
        resize_data_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
