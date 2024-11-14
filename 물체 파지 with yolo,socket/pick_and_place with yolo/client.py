import pyrealsense2 as rs
import numpy as np
import cv2
import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket

class ObjectDetectionNode(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        # Socket 클라이언트 설정

        HOST = '192.168.26.48'  # 서버 IP 주소 (예: '192.168.0.10')
        PORT = 12345            # 서버와 동일한 포트 번호

        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client_socket.connect((HOST, PORT))
        print("서버에 연결되었습니다.")
        
        # YOLOv5 모델 로드
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

        # RealSense 초기화
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        # ROS 퍼블리셔 초기화
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, 'detected_image', 10)
        self.label_publisher = self.create_publisher(String, 'detected_labels', 10)
        self.coord_publisher = self.create_publisher(String, 'detected_coords', 10)  # 좌표 퍼블리셔

        # 주기적 타이머 설정 (30Hz로 업데이트)
        self.timer = self.create_timer(1/30, self.process_frame)

    def process_frame(self):
        # 프레임 수신
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        # OpenCV 이미지 변환
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # YOLOv5 객체 검출 수행
        results = self.model(color_image)

        self.detection_result = String()
        
        bbox_info = results.xyxy[0].numpy()

        for result in bbox_info:
            x1, y1, x2, y2, confidence, class_id = map(int, result[:6])

            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            depth_value = depth_image[center_y, center_x] * 0.001

            label = self.model.names[class_id] + ','
            # 서버로 메시지 전송
            self.client_socket.sendall(label.encode('utf-8'))
            
            self.detection_result.data = f'{label}, distance: {depth_value:.2f}m' 

            # 카메라의 깊이 센서 정보 가져오기
            depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics

            # 픽셀의 깊이값 (미터 단위)
            depth = depth_frame.get_distance(center_x, center_y)

            # x, y, z 좌표 계산
            x, y, z = rs.rs2_deproject_pixel_to_point(depth_intrin, [center_x, center_y], depth)



            print(f"label : {label}, x = {x}, y = {y}, z = {z}")
            cv2.rectangle(color_image, (x1,y1), (x2,y2),(0,255,0),2)
            cv2.circle(color_image,(center_x,center_y),5,(0,0,255),-1)
            cv2.putText(color_image,f'{label} {depth_value:.2f}m',(x1,y1-10),cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

        # OpenCV 이미지로 변환
        image_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')

        # ROS2 토픽 메시지 퍼블리시
        self.image_publisher.publish(image_msg)
        # publish하는거는 일단 pass


    def stop(self):
        # 리소스 해제
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()