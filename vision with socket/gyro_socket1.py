import pyrealsense2 as rs
import time
import socket

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
pipeline.start(config)

H = '192.168.110.125'
P = 20002
def server_open(Host=H,Port=P):
    sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock1.bind((H, P))
    sock1.listen(5)
    clint_socket, addr = sock1.accept()
    return clint_socket

try:
    sock1 = server_open()

    while True:
        frame = pipeline.wait_for_frames()
        gyro_frame = frame.first_or_default(rs.stream.gyro)

        if gyro_frame:
            gyro_data = gyro_frame.as_motion_frame().get_motion_data()
            data_str = f"{gyro_data.x}, {gyro_data.y}, {gyro_data.z}"
            sock1.sendall(data_str.encode('utf-8'))
            print(f'X=: {gyro_data.x}, Y=: {gyro_data.y}, Z=:{gyro_data.z}')
            time.sleep(0.5)
finally:
    pipeline.stop()