import socket


def server_open():
    sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock1 = setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock1.bind(("127.0.0.1", 20002))
    sock1.listen(5)
    clint_socket, addr = sock1.accept()
    clint_socket.sendall(b"hello")
    sock1.close()


def connect_client():
    sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock1.connect(("127.0.0.1", "20002"))
    msg = sock1.recv(1024).decode()
    sock1.close()
    return msg
