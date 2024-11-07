import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import torch

class YOLOv5ObjectDetection(Node):
    def __init__(self):
        super().__init__('yolov5_object_detection')

        # FaceCheck 모델 불러오기
        model_path = '/home/mj/yolov5/runs/train/exp9/weights/best.pt'
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)

        # CvBridge를 사용하여 ROS 이미지 메시지를 OpenCV 이미지로 변환
        self.bridge = CvBridge()

        # 이미지 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # 이미지 토픽 이름을 실제 사용 중인 토픽 이름으로 변경
            self.image_callback,
            10
        )
        
        # 객체 검출된 이미지를 퍼블리시할 토픽 생성
        self.publisher = self.create_publisher(
            Image,
            '/detected_face',  # 객체 검출된 이미지를 퍼블리시할 토픽 이름
            10
        )

        # OpenCV 윈도우 설정
        cv2.namedWindow('YOLOv5 Object Detection', cv2.WINDOW_NORMAL)

    def image_callback(self, msg):
        # ROS 이미지 메시지를 OpenCV 이미지로 변환
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # OpenCV의 BGR 포맷을 RGB로 변환
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 모델을 사용해 객체 검출 수행
        results = self.model(rgb_frame)

        # 결과를 원래 BGR 포맷의 프레임에 그리기
        results.render()  # 결과를 프레임에 그립니다
        annotated_frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)  # 다시 BGR로 변환

        # 객체 검출된 이미지를 ROS 메시지로 변환하여 퍼블리시
        detected_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.publisher.publish(detected_msg)

        # 결과 출력
        cv2.imshow('YOLOv5 Object Detection', annotated_frame)

        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info("종료 신호가 감지되었습니다.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv5ObjectDetection()
    rclpy.spin(node)

    # 종료 시 OpenCV 창 닫기
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
