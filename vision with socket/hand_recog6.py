
import cv2
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np

#MediaPipe 모듈 초기화
mp_hands=mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

#Realsense 파이프라인 초기화
pipeline = rs.pipeline()
config=rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
#파이프 라인 시작
pipeline.start(config)

try:
  with mp_hands.Hands(
    model_complexity=1, #값이 올라가면 정확도가 올라가지만 느려집니다.
    max_num_hands=2, #손을 몇개나 탐지할 것인가
    min_detection_confidence=0.5, #신뢰도 확률 0~1
    min_tracking_confidence=0.5) as hands:

    while True:
      frames = pipeline.wait_for_frames()
      color_frame = frames.get_color_frame()
      depth_frame = frames.get_depth_frame()

      if not color_frame or not depth_frame:
        continue
      # RGB데이터를 NumPy 배열로 변환
      color_image = np.asanyarray(color_frame.get_data())

      #Mediapipe를 위한 이미지 변환
      image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
      image.flags.writeable = False

      #손 랜드마크 탐지
      results = hands.process(image)
      #이미지 다시 RGB로 변환
      image.flags.writeable = True
      image=cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

      #랜드마크 그리기
      if results.multi_hand_landmarks:
        for hand_landmarks in results.multi_hand_landmarks:
          mp_drawing.draw_landmarks(
            image,
            hand_landmarks,
            mp_hands.HAND_CONNECTIONS,
            mp_drawing.DrawingSpec(color=(0,255,0), thickness=2, circle_radius=4),
            mp_drawing.DrawingSpec(color=(0,0,255), thickness=2))
          
          # 각 랜드마크의 3D좌표를 출력
          for id, landmark in enumerate(hand_landmarks.landmark):
            h, w, c = image.shape
            cx, cy = int(landmark.x*w), int(landmark.y*h)

            if (0<= cx <2) and (0<= cy<h) :
              depth = depth_frame.get_distance(cx, cy)
              print(f"ID: {id}, X: {cx}, Y: {cy}, Depth: {depth:.2f}m")
            else:
              print(f"X={cx}, Y={cy}")

      cv2.imshow('RealSense Hand Tracking', image)

      # 'q' 키를 누르면 종료
      if cv2.waitKey(1) & 0xFF == ord('q'):
        break

finally:
  pipeline.stop()
  cv2.destroyAllWindows()

