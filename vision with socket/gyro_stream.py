import pyrealsense2 as rs
import time

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
pipeline.start(config)

try:
    while True:
        frame = pipeline.wait_for_frames()
        gyro_frame = frame.first_or_default(rs.stream.gyro)

        if gyro_frame:
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()
            print(f'X=: {gyro_data.x}, Y=: {gyro_data.y}, Z=:{gyro_data.z}')
            time.sleep(1)
finally:
    pipeline.stop()