import socket
import time
import threading

HOST = '127.0.0.1'
PORT = 65432

def connect_to_server():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST,PORT))
            print(f'Connected to {HOST}:{PORT}')
            return s
        except socket.error as e:
            print(f'Connection failed: {e}. Retrying in 5 seconds...')
            s.close()
            time.sleep(5)
try:
    while True:
        s = connect_to_server()
        
        while True:
            try:
                data = s.recv(1024)
                if not data:
                    break
                command = data.decode('utf-8')

                if command == '1':
                    print('command 1 recieved')
                elif command == '2':
                    print('command 2 recieved ')
                elif command == '3':
                    print('command 3 recieved ')
                elif command == '4':
                    print('command 4 recieved ')
                else:
                    print('Unknow command recieved')

                time.sleep(0.1)

            except socket.error as e:
                print(f'Socket error: {e}. Reconnecting...')
                break
        s.close()
except KeyboardInterrupt:
    print("Program terminated")
except Exception as e:
    print(f'An error occurred: {e}')
finally:
    print('Resources releases and motor stopped')
