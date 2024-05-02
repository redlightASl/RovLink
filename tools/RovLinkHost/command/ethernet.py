import sys
import socket


class Ethernet(object):
    def __init__(self,host_ip="192.168.137.2",port_num=8008) -> None:
        self.host_ip = host_ip
        self.port_num = port_num
        
        #UDP
        udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_socket.bind(("", 8010))
        
        def send_msg(self, dest_ip, dest_port, msg):
            pass
        