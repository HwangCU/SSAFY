import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2

class FaceRecognizerNode(Node):

    def __init__(self):
        super().__init__('face_recognizer_node')
        
        # RGB 이미지 구독 설정
        self.rgb_subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # RGB 이미지 토픽 이름
            self.rgb_callback,
            10  # QoS 설정
        )
        self.bridge = CvBridge()
        self.rgb_image = None
        cascade_path = '/usr/share/opencv4/haarcascades/'
        self.face_cascade = cv2.CascadeClassifier(cascade_path + 'haarcascade_frontalface_default.xml')

        # 얼굴 정보 저장용 딕셔너리
        self.faces_dict = {}

    def rgb_callback(self, msg):
        # RGB 데이터를 OpenCV 형식으로 변환
        rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # 이미지를 640x480으로 리사이즈
        self.rgb_image = cv2.resize(rgb_image, (640, 480))

        # 이미지 시각화
        self.display_images()

    def display_images(self):
        # 흑백으로 변환하여 얼굴 감지 정확도를 높임
        gray_frame = cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray_frame, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

        for idx, (x, y, w, h) in enumerate(faces):
            face_id = (x, y, w, h)  # 얼굴 위치를 이용한 고유 ID 생성
            if face_id not in self.faces_dict:
                # 새로운 얼굴인 경우 이름을 입력받아 저장
                name = input(f"Enter name for face {idx+1}: ")
                self.faces_dict[face_id] = name
            
            # 얼굴에 사각형을 그리고 이름 표시
            name = self.faces_dict[face_id]
            cv2.rectangle(self.rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(self.rgb_image, name, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

        # 감지된 얼굴 수를 화면에 표시
        num_faces = len(faces)
        cv2.putText(self.rgb_image, f'Faces detected: {num_faces}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)

        # 화면에 처리된 이미지 표시
        cv2.imshow('Face Detection', self.rgb_image)

        # 's' 키를 누르면 감지된 얼굴이 있는 이미지를 저장
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and num_faces > 0:
            cv2.imwrite('detected_face.png', self.rgb_image)
            print("Frame with detected face saved as 'detected_face.png'.")

def main(args=None):
    rclpy.init(args=args)

    face_recognizer_node = FaceRecognizerNode()

    rclpy.spin(face_recognizer_node)

    face_recognizer_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
