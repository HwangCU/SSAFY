import socket

host = '192.168.110.125'
port = 65432

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((host, port))
    s.listen()
    print(f'Server is listening on {host}: {port}....')

    conn, addr = s.accept()
    with conn:
        print(f'Connected by {addr}')
        while True:
            command = input('Enter the msg: ')
            if command.lower() == 'exit':
                print('Closing connection.')
                break

            conn.sendall(command.encode('utf-8'))
            print(f'Sent: {command}')