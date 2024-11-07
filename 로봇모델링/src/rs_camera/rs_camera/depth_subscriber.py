import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DepthSubscriber(Node):
    def __init__(self):
        super().__init__('depth_subscriber')
        
        # 구독자 설정
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',
            self.listener_callback,
            10)
        
        self.subscription  # 방지: 콜백이 사용되지 않는다고 경고하지 않도록 설정
        
        # CvBridge를 사용하여 ROS 이미지 메시지를 OpenCV 이미지로 변환
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Received depth image frame')

        # ROS Image 메시지를 OpenCV 이미지로 변환 (16UC1 형식)
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # 깊이 값을 시각화하기 위해 0에서 255 사이의 값으로 변환 (정규화)
        depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)

        # 8비트로 변환
        depth_image_8u = np.uint8(depth_image_normalized)

        # OpenCV로 깊이 이미지 시각화
        cv2.imshow("Depth Image", depth_image_8u)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    depth_subscriber = DepthSubscriber()

    try:
        rclpy.spin(depth_subscriber)
    except KeyboardInterrupt:
        depth_subscriber.get_logger().info("Node stopped cleanly")
    except Exception as e:
        depth_subscriber.get_logger().error(f"Error: {str(e)}")
    finally:
        depth_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
