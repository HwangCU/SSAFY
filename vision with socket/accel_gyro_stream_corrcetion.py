import pyrealsense2 as rs
import time
import numpy as np

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
pipeline.start(config)

prev_time = time.time()
angle = np.array([0., 0., 0.])

try:
    while True:
        frames = pipeline.wait_for_frames()

        gyro_frame = frames.first_or_default(rs.stream.gyro)
        accel_frame = frames.first_or_default(rs.stream.accel)

        if gyro_frame:
            # 현재시간
            current_time = time.time()
            # 자이로 데이터 읽기
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()

            dt = current_time - prev_time

            gyro_rates = np.array([gyro_data.x, gyro_data.y, gyro_data.z])
            angle += gyro_rates * dt
            prev_time = current_time

        if accel_frame:
            accel_data = accel_frame.as_motion_frame().get_motion_data()
            # 각도 보정
            accel_angle = np.arctan2(accel_data.y, accel_data.z)
            angle[0] = 0.98*angle[0] + 0.02*accel_angle

            print(f'Angle X = {np.degrees(angle[0])}, Angle Y = {np.degrees(angle[1])}, Angle Z = {np.degrees(angle[2])}')
finally:
    pipeline.stop()
