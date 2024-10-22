import pyrealsense2 as rs
import numpy as np
import cv2

# Realsense 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()

# 컬러 스트림만 활성화
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)

# 피부색 HSV 범위 설정
lower_skin = np.array([0, 20, 70], dtype=np.uint8)
upper_skin = np.array([20, 255, 255], dtype=np.uint8)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        # 컬러 프레임을 NumPy 배열로 변환
        color_image = np.asanyarray(color_frame.get_data())

        # BGR을 HSV로 변환
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # 피부색 범위를 사용하여 마스크 생성
        mask = cv2.inRange(hsv_image, lower_skin, upper_skin)

        # 마스크에 대해 모폴로지 연산을 적용하여 잡음을 제거
        mask = cv2.erode(mask, np.ones((3, 3), np.uint8), iterations=1)
        mask = cv2.dilate(mask, np.ones((3, 3), np.uint8), iterations=1)

        # 블러링을 적용하여 마스크를 부드럽게 함
        mask = cv2.GaussianBlur(mask, (5, 5), 100)

        # 마스크에서 컨투어 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # 손 크기에 해당하는 컨투어만 처리
            if cv2.contourArea(contour) > 3000:
                # 손 윤곽선 그리기
                cv2.drawContours(color_image, [contour], -1, (0, 255, 0), 2)

                # 손이 감지되면 "손" 메시지 출력
                print("손")

        # 결과 이미지 출력
        cv2.imshow("Hand Detection", color_image)

        # ESC 키를 누르면 종료 (ASCII 코드 27)
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    # 파이프라인 정리
    pipeline.stop()
    cv2.destroyAllWindows()
