import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        
        # 구독자 설정
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10)
        
        self.subscription  # 방지: 콜백이 사용되지 않는다고 경고하지 않도록 설정
        
        # CvBridge를 사용하여 ROS 이미지 메시지를 OpenCV 이미지로 변환
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Received image frame')

        # ROS Image 메시지를 OpenCV 이미지로 변환
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # OpenCV로 이미지 시각화
        cv2.imshow("Camera Image", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        image_subscriber.get_logger().info("Node stopped cleanly")
    except Exception as e:
        image_subscriber.get_logger().error(f"Error: {str(e)}")
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
