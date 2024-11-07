import pyrealsense2 as rs
import numpy as np
import cv2
import socket


def server_open():
    sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock1.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock1.bind(("127.0.0.1", 20002))
    sock1.listen(5)
    clint_socket, addr = sock1.accept()
    print(f"connected {addr}")
    clint_socket.sendall(b"start")
    sock1.close()


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

            # BGR 이미지를 HSV 색상 공간으로 변환
            hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

            # 빨간색의 HSV 범위 설정 (두 범위로 나눔, 빨간색은 HSV에서 두 가지 범위를 차지함)
            lower_red1 = np.array([0, 100, 100])  # 첫 번째 범위의 하한값
            upper_red1 = np.array([10, 255, 255])  # 첫 번째 범위의 상한값
            lower_red2 = np.array([160, 100, 100])  # 두 번째 범위의 하한값
            upper_red2 = np.array([180, 255, 255])  # 두 번째 범위의 상한값

            # 첫 번째 및 두 번째 범위에 해당하는 마스크 생성
            red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
            red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

            # 두 마스크를 합침
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)

            # 컨투어(윤곽선) 찾기
            contours, _ = cv2.findContours(
                red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # 각 컨투어에 대해 외곽에 사각형 그리기 및 중심점 표시
            for contour in contours:
                if cv2.contourArea(contour) > 500:  # 너무 작은 컨투어는 무시
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(
                        color_image, (x, y), (x + w, y + h), (0, 255, 255), 2
                    )  # 노란색 사각형
                    # Enter를 눌러 좌표 출력
                    if cv2.waitKey(1) & 0xFF == 13:
                        server_open()
                        print(f"좌표: x={x}, y={y}, 너비={w}, 높이={h}")

                        # 컨투어 개수 확인 (디버깅용)
                        contour_count = len(contours)
                        print(f"컨투어 개수: {contour_count}")

                    # 컨투어의 중심점 계산
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])

                        # 중심점에 원 그리기 및 좌표 텍스트 표시
                        cv2.circle(
                            color_image, (cx, cy), 5, (0, 255, 255), -1
                        )  # 노란색 원
                        cv2.putText(
                            color_image,
                            f"({cx}, {cy})",
                            (cx - 50, cy - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 255),
                            2,
                        )

            # 화면에 표시
            cv2.imshow(
                "RealSense 2D - Red Object Detection with Centroid (HSV)", color_image
            )

            # ESC를 눌러 종료
            if cv2.waitKey(1) & 0xFF == 27:
                break

    finally:
        # 정리 작업
        pipeline.stop()
        cv2.destroyAllWindows()


# 함수를 호출하여 2D 스트리밍 시작
start_realsense_2d_stream()
