import cv2
import mediapipe as mp
import pyrealsense2 as rs
import numpy as np

# MediaPipe 모듈 초기화
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

# Realsense 파이프라인 초기화
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 파이프라인 시작
pipeline.start(config)

# 출력할 랜드마크 ID 리스트
selected_ids = [4, 8, 12, 16, 20]

def count_fingers(hand_landmarks, image_width, image_height):
    """
    손가락 개수를 계산하는 함수
    """
    fingers = [False] * 5  # 각 손가락 상태를 저장 (True = 펴짐)

    # 랜드마크 좌표 계산
    for i, id in enumerate(selected_ids):
        tip_x = hand_landmarks.landmark[id].x * image_width
        tip_y = hand_landmarks.landmark[id].y * image_height
        pip_y = hand_landmarks.landmark[id - 2].y * image_height  # PIP 관절 좌표

        # 손가락 펴짐 여부 판단
        if id == 4:  # 엄지
            pip_x = hand_landmarks.landmark[id - 1].x * image_width
            fingers[i] = tip_x < pip_x  # 엄지는 x축 기준
        else:  # 다른 손가락은 y축 기준
            fingers[i] = tip_y < pip_y

    return sum(fingers)  # 펴진 손가락 개수 반환

try:
    with mp_hands.Hands(
        model_complexity=1,
        max_num_hands=2,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5) as hands:

        while True:
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            # RGB 데이터를 NumPy 배열로 변환
            color_image = np.asanyarray(color_frame.get_data())

            # Mediapipe를 위한 이미지 변환
            image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            image.flags.writeable = False

            # 손 랜드마크 탐지
            results = hands.process(image)

            # 이미지 다시 RGB로 변환
            image.flags.writeable = True
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            # 랜드마크 그리기 및 특정 ID 값 출력
            if results.multi_hand_landmarks:
                for hand_landmarks in results.multi_hand_landmarks:
                    mp_drawing.draw_landmarks(
                        image,
                        hand_landmarks,
                        mp_hands.HAND_CONNECTIONS,
                        mp_drawing.DrawingSpec(color=(0, 255, 0), thickness=2, circle_radius=4),
                        mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2))

                    h, w, c = image.shape
                    fingers_count = count_fingers(hand_landmarks, w, h)

                    # 가위, 바위, 보 판단
                    if fingers_count == 0:
                        print("바위")
                        #cv2.putText(image, "Rock", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    elif fingers_count == 2:
                        print("가위")
                        #cv2.putText(image, "Scissors", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    elif fingers_count == 5:
                        print("보")
                        #cv2.putText(image, "Paper", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    else:
                        print("알 수 없는 동작")
                        #cv2.putText(image, "Unknown", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # 이미지 출력
            cv2.imshow('RealSense Hand Tracking', image)

            # 'q' 키를 누르면 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

