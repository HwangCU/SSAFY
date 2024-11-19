import pyrealsense2 as rs
import time

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)
pipeline.start()

try:
    while True:
        frames = pipeline.wait_for_frames()
        gyro_frame = frames.first_or_default(rs.stream.gyro)
        accel_frame = frames.first_or_default(rs.stream.accel)

        if gyro_frame:
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()
            print(f'Gyro X=: {gyro_data.x}, Gyro Y=: {gyro_data.y}, Gyro Z=:{gyro_data.z}')

        if accel_frame:
            accel_data = accel_frame.as_motion_frame().get_motion_data()
            print(f'Accel X=: {accel_data.x}, Accel Y=: {accel_data.y}, Accel Z=:{accel_data.z}')

        time.sleep(0.5)


finally:
    pipeline.stop()