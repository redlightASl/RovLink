import sys
import socket


class Ethernet(object):
    def __init__(self,host_ip="192.168.137.168",port_num=8008,dest_ip="192.168.137.100",dest_port=9999) -> None:
        self._host_ip = host_ip
        self._port_num = port_num
        self._dest_ip = dest_ip
        self._dest_port = dest_port
        
        self._tx_comm = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._tx_comm.bind(("", self._port_num))
        
        def transmit(self, msg, dbg_print=False) -> None:
            if dbg_print:
                print(msg)
            hex_bytes = bytes(msg)
            self._tx_comm.sendall(hex_bytes)
        
        def receive(self):
            data = self._rx_comm.recv(1024).decode()
            return data

if __name__ == '__main__':
    comm = Ethernet(debug=True)
    comm.transmit(b"Hello World!", dbg_print=True)
