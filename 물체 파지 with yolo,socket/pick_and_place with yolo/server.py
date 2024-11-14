import socket
from robodk import robolink, robomath, Mat
import threading

# RoboDK 설정
RDK = robolink.Robolink()    # RoboDK와 연결
RB1 = RDK.Item('RB1')  # RoboDK에서 설정한 로봇 이름으로 찾기

def move_robot():
    RB1.MoveL(Mat([
        [0.000000,     1.000000,     0.000000,   502.700000],
        [1.000000,    -0.000000,    -0.000000,  -110.700000],
        [-0.000000,     0.000000,    -1.000000,   498.400000],
        [0.000000,     0.000000,     0.000000,     1.000000] 
        ]))

    RB1.MoveL(Mat([    
        [-0.000000,     1.000000,     0.000000,   502.700000],
        [1.000000,     0.000000,    -0.000000,  -110.700000 ],
        [-0.000000,     0.000000,    -1.000000,   288.400000 ],
        [0.000000,     0.000000,     0.000000,     1.000000 ]
        ]))

    RB1.MoveL(Mat([
        [0.000000,     1.000000,     0.000000,   502.700000],
        [1.000000,    -0.000000,    -0.000000,  -110.700000],
        [-0.000000,     0.000000,    -1.000000,   498.400000],
        [0.000000,     0.000000,     0.000000,     1.000000] 
        ]))

    RB1.MoveL(Mat([    
        [-0.000000,     1.000000,     0.000000,   502.700000],
        [1.000000,     0.000000,    -0.000000,  -110.700000 ],
        [-0.000000,     0.000000,    -1.000000,   288.400000 ],
        [0.000000,     0.000000,     0.000000,     1.000000 ]
        ]))

# Socket 서버 설정
HOST = '0.0.0.0'   # 모든 인터페이스에서 접속 허용
PORT = 12345       # 포트 번호 (임의 설정)

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)
print("서버가 시작되었습니다. 연결을 기다리는 중입니다...")

conn, addr = server_socket.accept()
print(f"연결되었습니다: {addr}")

# 데이터 수신 및 RoboDK로 송신
try:
    while True:
        # 클라이언트로부터 데이터 수신
        data = conn.recv(1024).decode('utf-8')
        if not data:
            break

        print(data)
        data_list = [d.strip() for d in data.split(',')]

        if 'mouse' in data_list:
            print('mouse received')
            server_thread = threading.Thread(target=move_robot())
            server_thread.daemon = True
            server_thread.start()

except Exception as e:
    print("오류 발생:", e)
finally:
    conn.close()
    server_socket.close()
    print("서버가 종료되었습니다.")