import pyrealsense2 as rs
import numpy as np
import cv2
from pymodbus.client import ModbusTcpClient  # 수정된 부분
import time

# 모드버스 클라이언트 설정
client = ModbusTcpClient('192.168.26.32', port=502)

# RealSense 카메라 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 스트리밍 시작
pipeline.start(config)

def write_modbus_data(box_type, x, y, depth):
    try:
        client.write_register(99, box_type, 255)  # 박스 종류 (1, 2, 3) #modbus 100
        client.write_register(100, x, 255)        # X 좌표
        client.write_register(101, y, 255)        # Y 좌표
        client.write_register(102, int(depth * 1000), 255)  # Depth 값 (mm 단위로 전송)
    except Exception as e:
        print(f"Modbus write error: {e}")

try:
    while True:
        # 프레임 수신
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # 컬러 프레임과 Depth 프레임을 numpy 배열로 변환
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # BGR에서 HSV로 변환
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # 색상 범위 설정 (빨간색, 초록색, 파란색)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        lower_green = np.array([40, 40, 40])
        upper_green = np.array([70, 255, 255])

        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        # 각 색상 마스크 생성
        mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask_red = mask_red1 + mask_red2

        mask_green = cv2.inRange(hsv_image, lower_green, upper_green)
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # 노이즈 제거 (모폴로지 연산으로 작은 노이즈 제거)
        kernel = np.ones((5, 5), np.uint8)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_CLOSE, kernel)

        # 컨투어(윤곽선) 검출
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        box_type = 0  # 박스 타입 초기화
        cx, cy, depth_value = 0, 0, 0  # 중심점과 depth 초기화

        # 빨간색 상자 검출
        for contour in contours_red:
            area = cv2.contourArea(contour)
            if area > 500:  # 최소 크기 필터
                box_type = 1  # 빨간 상자 검출 시
                x, y, w, h = cv2.boundingRect(contour)
                cx = x + w // 2
                cy = y + h // 2
                depth_value = depth_frame.get_distance(cx, cy)

                # 외곽선 그리기
                cv2.drawContours(color_image, [contour], -1, (0, 255, 0), 2)
                cv2.putText(color_image, f"Red Box - X:{cx} Y:{cy} Depth:{depth_value:.3f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        # 초록색 상자 검출
        for contour in contours_green:
            area = cv2.contourArea(contour)
            if area > 500:  # 최소 크기 필터
                box_type = 2  # 초록 상자 검출 시
                x, y, w, h = cv2.boundingRect(contour)
                cx = x + w // 2
                cy = y + h // 2
                depth_value = depth_frame.get_distance(cx, cy)

                # 외곽선 그리기
                cv2.drawContours(color_image, [contour], -1, (0, 255, 0), 2)
                cv2.putText(color_image, f"Green Box - X:{cx} Y:{cy} Depth:{depth_value:.3f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        # 파란색 상자 검출
        for contour in contours_blue:
            area = cv2.contourArea(contour)
            if area > 500:  # 최소 크기 필터
                box_type = 3  # 파란 상자 검출 시
                x, y, w, h = cv2.boundingRect(contour)
                cx = x + w // 2
                cy = y + h // 2
                depth_value = depth_frame.get_distance(cx, cy)

                # 외곽선 그리기
                cv2.drawContours(color_image, [contour], -1, (0, 255, 0), 2)
                cv2.putText(color_image, f"Blue Box - X:{cx} Y:{cy} Depth:{depth_value:.3f}m", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)

        # 박스가 검출되면 모드버스로 값 전송
        if box_type != 0:
            #print(f"Box type: {box_type}, X: {cx}, Y: {cy}, Depth: {depth_value:.3f}")
            #write_modbus_data(box_type, cx, cy, depth_value)
            if box_type==1: # 빨강
                write_modbus_data(999, 1, 0, 0)
            elif box_type==2: # 초록
                write_modbus_data(999, 0, 1, 0)
            elif box_type==3: # 파랑
                write_modbus_data(999, 0, 0, 1)

            

        # 결과 이미지 표시
        cv2.imshow('Box Detection', color_image)

        # 'q'를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        # 0.1초마다 업데이트
        time.sleep(0.1)

finally:
    # 스트리밍 및 모드버스 클라이언트 종료
    pipeline.stop()
    client.close()
    cv2.destroyAllWindows()