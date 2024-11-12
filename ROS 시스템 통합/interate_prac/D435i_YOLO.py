import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import pyrealsense2 as rs
import cv2
import numpy as np
import torch
from cv_bridge import CvBridge

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        
        # ROS2 퍼블리셔 설정
        self.publisher_ = self.create_publisher(String, 'detected_objects', 10)
        self.image_publisher = self.create_publisher(Image, 'detection_image', 10)
        self.bridge = CvBridge()
        
        # RealSense 카메라 초기화
        self.pipeline = self.initialize_camera()

        # YOLOv5 모델 로드
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        self.model.eval()  # 추론 모드로 설정

        # 타이머 설정 (0.1초 주기)
        self.timer = self.create_timer(0.1, self.process_frame)

    def initialize_camera(self):
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipeline.start(config)
        return pipeline

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return None
        frame = np.asanyarray(color_frame.get_data())
        return frame

    def process_frame(self):
        frame = self.get_frame()
        if frame is not None:
            # YOLOv5 추론 수행
            results = self.model(frame)
            detected_objects = results.pandas().xyxy[0]['name'].tolist()

            # 객체 이름 퍼블리싱
            if detected_objects:
                msg = String()
                msg.data = ', '.join(detected_objects)
                self.publisher_.publish(msg)
                self.get_logger().info(f'Detected objects: {msg.data}')

            # 시각화된 이미지 퍼블리싱
            results.render()  # 이미지에 결과 렌더링
            annotated_frame = results.ims[0]  # 렌더링된 이미지를 가져오기
            # annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_RGB2BGR)  # OpenCV 호환을 위해 색상 변환

            # ROS2 이미지 메시지로 퍼블리시
            # self.image_publisher.publish(self.bridge.cv2_to_imgmsg(annotated_frame, 'bgr8'))

            cv2.imshow('cam',annotated_frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
