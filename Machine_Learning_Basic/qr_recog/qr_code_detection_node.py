import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class QRCodeDetectionNode(Node):

    def __init__(self):
        super().__init__('qr_code_detection_node')
        
        # RGB 이미지 구독 설정
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # RGB 이미지 토픽 이름
            self.rgb_callback,
            10  # QoS 설정
        )

        # QR 코드 데이터 퍼블리셔 설정
        self.qr_publisher = self.create_publisher(String, 'qr_code_data', 10)

        self.bridge = CvBridge()
        self.rgb_image = None
        self.qr_detector = cv2.QRCodeDetector()

    def rgb_callback(self, msg):
        # RGB 데이터를 OpenCV 형식으로 변환
        rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # 이미지를 640x480으로 리사이즈
        self.rgb_image = cv2.resize(rgb_image, (640, 480))
        
        # QR 코드 감지 및 데이터 퍼블리시
        self.detect_and_publish_qr_code()

        # 이미지 시각화
        self.display_images()

    def detect_and_publish_qr_code(self):
        # QR 코드 감지
        qr_data, bbox, _ = self.qr_detector.detectAndDecode(self.rgb_image)
        
        if qr_data:
            # QR 코드가 감지된 경우 데이터 퍼블리시
            self.get_logger().info(f"QR Code Detected: {qr_data}")
            
            msg = String()
            msg.data = qr_data
            self.qr_publisher.publish(msg)
            
            # QR 코드 위치에 사각형 그리기
            if bbox is not None:
                for i in range(len(bbox[0])):
                    pt1 = tuple(bbox[0][i].astype(int))
                    pt2 = tuple(bbox[0][(i + 1) % len(bbox[0])].astype(int))
                    cv2.line(self.rgb_image, pt1, pt2, (0, 255, 0), 2)
                
                # QR 코드 데이터 표시
                cv2.putText(self.rgb_image, qr_data, (int(bbox[0][0][0]), int(bbox[0][0][1]) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

    def display_images(self):
        if self.rgb_image is not None:
            cv2.imshow("QR Code Detection", self.rgb_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)

    qr_code_detection_node = QRCodeDetectionNode()

    rclpy.spin(qr_code_detection_node)

    qr_code_detection_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()