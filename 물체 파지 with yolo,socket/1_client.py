import socket
import time
import threading

HOST = '192.168.110.125'
PORT = 65432

def connect_to_server():
    while True:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((HOST, PORT))
            print(f'Connected to {HOST}:{PORT}')
            return s
            
        except socket.error as e:
        
            print(f'Connection failed: {e}. Retrying in 5 sec...')
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
                
                print(f'Recived msg: {data}')
                
            except socket.error as e:
                print(f'Socket error: {e}. Reconnection...')
                break
                
except KeyboardInterrupt:
    print("Program terminated")
except Exception as e:
    print(f'An error occurred: {e}')
    
finally:
    print('Resources release')