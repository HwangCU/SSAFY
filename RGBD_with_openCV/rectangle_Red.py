import pyrealsense2 as rs
import numpy as np
import cv2


def start_realsense_2d_stream():
    # RealSense 파이프라인 설정
    pipeline = rs.pipeline()
    config = rs.config()

    # 컬러 스트림만 활성화
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 스트림 시작
    pipeline.start(config)

    try:
        while True:
            # 프레임을 가져오기
            frames = pipeline.wait_for_frames()

            # 컬러 프레임을 가져옴
            color_frame = frames.get_color_frame()

            if not color_frame:
                continue

            # 컬러 이미지를 numpy 배열로 변환
            color_image = np.asanyarray(color_frame.get_data())

            # BGR 색상 공간에서 빨간색의 범위 설정
            lower_red = np.array([0, 0, 150])  # 빨간색의 하한값 (BGR)
            upper_red = np.array([100, 100, 255])  # 빨간색의 상한값 (BGR)

            # 빨간색에 해당하는 부분을 마스크로 추출
            red_mask = cv2.inRange(color_image, lower_red, upper_red)

            # 컨투어(윤곽선) 찾기
            contours, _ = cv2.findContours(
                red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # 각 컨투어에 대해 외곽에 사각형 그리기
            for contour in contours:
                if cv2.contourArea(contour) > 200:  # 너무 작은 컨투어는 무시
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(
                        color_image, (x, y), (x + w, y + h), (0, 255, 255), 2
                    )  # 노란색 사각형

            # 화면에 표시
            cv2.imshow("RealSense 2D - Red Object Detection", color_image)

            # ESC를 눌러 종료
            if cv2.waitKey(1) & 0xFF == 27:
                break

    finally:
        # 정리 작업
        pipeline.stop()
        cv2.destroyAllWindows()


# 함수를 호출하여 2D 스트리밍 시작
start_realsense_2d_stream()
