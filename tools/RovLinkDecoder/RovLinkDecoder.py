import serial
import socket
import binascii
import re
import pandas as pd
import numpy as np
import time

from RovLink import RovLink

COM_BAUDRATE = 115200


class RovLinkDecoder(object):
    def __init__(self, protocol="Serial", com="COM3", ip="127.0.0.1", port=9999):
        self._buffer_size = 20
        self._protocol = protocol
        self._rovlink_frame = []
        self._rovlink = RovLink()

        if self._protocol == "Serial":
            self.dev = serial.Serial(com, COM_BAUDRATE, timeout=0.1)
        elif self._protocol == "TCP_CLIENT":
            self.dev = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dev.connect((ip, port))
        elif self._protocol == "TCP_SERVER":
            self.dev = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.dev.bind((ip, port))
            self.dev.listen(1)
            self._client_socket, self._client_address = self.dev.accept()
        elif self._protocol == "UDP":
            self.dev = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        else:
            raise ValueError("Unknown protocol")

    def RLD_read(self):
        if self._protocol == "Serial":
            inputdata = self.dev.read(self._buffer_size)
            inputdata = str(binascii.b2a_hex(inputdata))[2:-1]
            is_framehead = inputdata.find("fd")  # 寻找帧头
            if is_framehead != -1:  # 找到帧头
                frame_raw = inputdata[is_framehead : is_framehead + (10 * 2)]
                hex_array = [
                    int(frame_raw[i : i + 2], 16) for i in range(0, len(frame_raw), 2)
                ]
                self._rovlink.rovlink_decode(hex_array)
                # self._rovlink.rovlink_checkout()
        elif self._protocol == "TCP_CLIENT":
            inputdata = self.dev.recv(self._buffer_size * 100)
            inputdata_hex = inputdata.hex()
            hex_array = [
                inputdata_hex[i : i + 2] for i in range(0, len(inputdata_hex), 2)
            ]
            byte_array = bytearray(binascii.unhexlify("".join(hex_array)))
            self._rovlink.rovlink_decode(byte_array)
            self._rovlink.rovlink_checkout()
        elif self._protocol == "TCP_SERVER":
            inputdata = self._client_socket.recv(self._buffer_size * 100)
            inputdata_hex = inputdata.hex()
            hex_array = [
                inputdata_hex[i : i + 2] for i in range(0, len(inputdata_hex), 2)
            ]
            byte_array = bytearray(binascii.unhexlify("".join(hex_array)))
            self._rovlink.rovlink_decode(byte_array)
            self._rovlink.rovlink_checkout()

    def RLD_write(self, Opcode, Identify, DLC, Vbit, Sbit, Payload):
        self._rovlink.rovlink_encode(Opcode, Identify, DLC, Vbit, Sbit, Payload)
        self._rovlink_frame = self._rovlink.rovlink_generate()
        self.dev.write(self._rovlink_frame)
        # for i in self._rovlink_frame:
        #     self.dev.write(i)

    def RLD_analysis(self):
        self._rovlink.rovlink_checkout()


if __name__ == "__main__":
    # rld = RovLinkDecoder(protocol="Serial", port="COM31")
    rld = RovLinkDecoder(protocol="TCP_SERVER", ip="127.0.0.1", port=1111)
    data = [1500, 1600, 1400]
    # rld.RLD_write(
    #     Opcode="LightA", Identify="Host", DLC="Triple", Vbit=1, Sbit=1, Payload=data
    # )

    while True:
        rld.RLD_read()
        # rld.RLD_write(
        #     Opcode="LightA", Identify="Host", DLC="Triple", Vbit=1, Sbit=1, Payload=data
        # )
        # time.sleep(1)
