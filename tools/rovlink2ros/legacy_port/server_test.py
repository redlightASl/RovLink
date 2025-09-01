import socket
import time

HOST = '127.0.0.1'  # Server IP
PORT = 9001 # Server端口

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen()

print('Waiting for a connection...')
client_socket, client_address = server_socket.accept()

while True:
    data = client_socket.recv(1024)
    if not data:
        continue
    rov_msg = data.decode('utf-8').split(':')
    print(rov_msg)
