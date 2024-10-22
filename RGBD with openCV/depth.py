import pyrealsense2 as rs
import numpy as np
import cv2

# Realsense 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()

# 컬러와 깊이 스트림 모두 활성화
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 파이프라인 시작
pipeline.start(config)

# 깊이 스케일 설정 (실제 깊이 값을 얻기 위해 필요)
depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# 프레임 맞춤을 위한 align 객체 생성
align_to = rs.stream.color
align = rs.align(align_to)

# HSV에서 빨간색 범위 설정 (2개의 범위를 사용하여 빨간색의 양쪽 끝을 포함)
lower_red_1 = np.array([0, 100, 100])
upper_red_1 = np.array([10, 255, 255])
lower_red_2 = np.array([160, 100, 100])
upper_red_2 = np.array([180, 255, 255])

try:
    while True:
        frames = pipeline.wait_for_frames()

        # 프레임을 색상과 깊이로 정렬
        aligned_frames = align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            continue

        # 컬러 및 깊이 프레임을 NumPy 배열로 변환
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # BGR을 HSV로 변환
        hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)

        # 두 범위를 사용하여 빨간색 영역 마스크 생성
        mask1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
        mask2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)

        # 두 마스크를 합쳐서 최종 마스크 생성
        mask = mask1 + mask2

        # 마스크에서 컨투어 찾기
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            # 네모로 인식될 수 있는 일정 크기의 컨투어만 처리
            if cv2.contourArea(contour) > 1000:
                x, y, w, h = cv2.boundingRect(contour)

                # 노란색 외곽선 그리기
                cv2.rectangle(color_image, (x, y), (x + w, y + h), (0, 255, 255), 2)

                # 중심점 계산
                cx = x + w // 2
                cy = y + h // 2

                # 깊이 값 추출 (깊이 값은 m 단위로 변환됨)
                depth_value = (
                    depth_image[cy, cx] * depth_scale
                )  # 깊이 값 가져오기 (단위: 미터)

                # 중심점에 원 그리기
                cv2.circle(color_image, (cx, cy), 5, (0, 255, 255), -1)

                # 중심점과 깊이 값 텍스트 출력
                cv2.putText(
                    color_image,
                    f"Center: ({cx}, {cy}) Depth: {depth_value:.2f}m",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    2,
                )

        # 결과 이미지 출력
        cv2.imshow("Red Box Detection with Depth", color_image)

        # ESC 키를 누르면 종료 (ASCII 코드 27)
        if cv2.waitKey(1) & 0xFF == 27:
            break

finally:
    # 파이프라인 정리
    pipeline.stop()
    cv2.destroyAllWindows()
