import pyrealsense2 as rs
import numpy as np
import cv2

# RealSense 카메라 파이프라인 설정
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

# 스트리밍 시작
pipeline.start(config)

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
        contours_red, _ = cv2.findContours(
            mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        contours_green, _ = cv2.findContours(
            mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )
        contours_blue, _ = cv2.findContours(
            mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
        )

        # 빨간색 상자 검출
        for contour in contours_red:
            area = cv2.contourArea(contour)
            if area > 500:  # 최소 크기 필터
                # 외곽선 그리기
                cv2.drawContours(color_image, [contour], -1, (0, 255, 0), 2)

                # 중심점 계산
                x, y, w, h = cv2.boundingRect(contour)
                cx = x + w // 2
                cy = y + h // 2

                # 중심점 그리기
                cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)

                # 중심점의 depth 정보 가져오기
                depth_value = depth_frame.get_distance(cx, cy)
                print(
                    f"Red Box detected - Center: (x: {cx}, y: {cy}), Depth: {depth_value:.3f} meters"
                )
                print("1")  # 빨간 상자 검출

        # 초록색 상자 검출
        for contour in contours_green:
            area = cv2.contourArea(contour)
            if area > 500:  # 최소 크기 필터
                # 외곽선 그리기
                cv2.drawContours(color_image, [contour], -1, (255, 0, 0), 2)

                # 중심점 계산
                x, y, w, h = cv2.boundingRect(contour)
                cx = x + w // 2
                cy = y + h // 2

                # 중심점 그리기
                cv2.circle(color_image, (cx, cy), 5, (0, 255, 0), -1)

                # 중심점의 depth 정보 가져오기
                depth_value = depth_frame.get_distance(cx, cy)
                print(
                    f"Green Box detected - Center: (x: {cx}, y: {cy}), Depth: {depth_value:.3f} meters"
                )
                print("2")  # 초록 상자 검출

        # 파란색 상자 검출
        for contour in contours_blue:
            area = cv2.contourArea(contour)
            if area > 500:  # 최소 크기 필터
                # 외곽선 그리기
                cv2.drawContours(color_image, [contour], -1, (0, 255, 255), 2)

                # 중심점 계산
                x, y, w, h = cv2.boundingRect(contour)
                cx = x + w // 2
                cy = y + h // 2

                # 중심점 그리기
                cv2.circle(color_image, (cx, cy), 5, (255, 0, 0), -1)

                # 중심점의 depth 정보 가져오기
                depth_value = depth_frame.get_distance(cx, cy)
                print(
                    f"Blue Box detected - Center: (x: {cx}, y: {cy}), Depth: {depth_value:.3f} meters"
                )
                print("3")  # 파란 상자 검출

        # 결과 이미지 표시
        cv2.imshow("Box Detection", color_image)

        # 'q'를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

finally:
    # 스트리밍 종료
    pipeline.stop()
    cv2.destroyAllWindows()
