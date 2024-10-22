import pyrealsense2 as rs
import numpy as np
import cv2
import socket
import threading


def socket_server():
    global latest_coords
    # 소켓 서버 설정
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind(("127.0.0.1", 20002))
    server_socket.listen(1)
    print("소켓 서버가 시작되었습니다. 클라이언트의 연결을 기다립니다...")

    conn, addr = server_socket.accept()
    print(f"클라이언트가 연결되었습니다: {addr}")
    try:
        while True:
            data = conn.recv(1024).decode().strip()
            if not data:
                continue
            print(f"클라이언트로부터 받은 명령어: {data}")
            if data.lower() == "shot":
                # 최신 좌표를 클라이언트에게 전송
                with coords_lock:
                    if latest_coords != (None, None):
                        coord_str = f"{latest_coords[0]},{latest_coords[1]}"
                    else:
                        coord_str = "No object detected"
                conn.sendall(coord_str.encode())
            elif data.lower() == "exit":
                break
    finally:
        conn.close()
        server_socket.close()
        print("소켓 서버가 종료되었습니다.")


def start_realsense_2d_stream():
    global latest_coords
    # RealSense 파이프라인 설정
    pipeline = rs.pipeline()
    config = rs.config()

    # 컬러 스트림만 활성화
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # 스트림 시작
    pipeline.start(config)

    # 윤곽선 표시 여부를 저장하는 변수 초기화
    display_contours = False

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

            # 빨간색의 HSV 범위 설정 (두 범위로 나눔)
            lower_red1 = np.array([0, 100, 100])
            upper_red1 = np.array([10, 255, 255])
            lower_red2 = np.array([160, 100, 100])
            upper_red2 = np.array([180, 255, 255])

            # 두 마스크를 생성하고 합침
            red_mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
            red_mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)

            # 디버그: 마스크의 크기 출력
            # print(len(red_mask))

            # 컨투어 찾기
            contours, _ = cv2.findContours(
                red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # 컨투어 개수 확인
            contour_count = len(contours)
            # print(f"컨투어 개수: {contour_count}")

            # 가장 큰 컨투어를 찾아서 중심점 계산
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                if cv2.contourArea(largest_contour) > 500:
                    x, y, w, h = cv2.boundingRect(largest_contour)
                    M = cv2.moments(largest_contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        with coords_lock:
                            latest_coords = (cx, cy)
                    else:
                        with coords_lock:
                            latest_coords = (None, None)
                else:
                    with coords_lock:
                        latest_coords = (None, None)
            else:
                with coords_lock:
                    latest_coords = (None, None)

            # 윤곽선 표시 여부에 따라 처리
            if display_contours:
                if contours:
                    for contour in contours:
                        if cv2.contourArea(contour) > 500:
                            x, y, w, h = cv2.boundingRect(contour)
                            cv2.rectangle(
                                color_image, (x, y), (x + w, y + h), (0, 255, 255), 2
                            )
                            M = cv2.moments(contour)
                            if M["m00"] != 0:
                                cx = int(M["m10"] / M["m00"])
                                cy = int(M["m01"] / M["m00"])
                                cv2.circle(color_image, (cx, cy), 5, (0, 255, 255), -1)
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

            # 키 입력 처리
            key = cv2.waitKey(1) & 0xFF

            if key == 13:  # Enter 키를 누르면 윤곽선 표시 토글
                display_contours = not display_contours
            elif key == 27:  # ESC 키를 누르면 종료
                break

    finally:
        # 정리 작업
        pipeline.stop()
        cv2.destroyAllWindows()


# 글로벌 변수 및 락 초기화
latest_coords = (None, None)
coords_lock = threading.Lock()

# 소켓 서버 스레드 시작
server_thread = threading.Thread(target=socket_server, daemon=True)
server_thread.start()

# 함수를 호출하여 2D 스트리밍 시작
start_realsense_2d_stream()
