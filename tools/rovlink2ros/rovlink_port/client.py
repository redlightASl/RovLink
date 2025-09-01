import socket
from rovlink import *
import time

# 机器人
server_1_ip = '127.0.0.1'
server_1_port = 9999
server_1_address = (server_1_ip, server_1_port)

# wsl
server_2_ip = '127.0.0.1'
server_2_port = 9001
server_2_address = (server_2_ip, server_2_port)


BUFFER_SIZE = 50

class RovLinkDataDecoder:
    def __init__(self):
        self.rovlink_sensor_data = RovlinkFullFrame()
        self.rovlink_sensor_data.header = 0xfd
        self.rovlink_sensor_data.device = RovDeviceType.COMM_CAB
        self.rovlink_sensor_data.valid  = False
        self.rovlink_sensor_data.sensor = True

        self.bno055 = RovSensorEulerAngleData()
        self.m10 = RovSensorWaterTempDepthPressData()
        self.ina226 = RovBatteryVoltageData()

        self.x= 0
        self.y= 0
        self.z= 0

        self.temperature = 0
        self.depth = 0


    def decode(self, data):
        print(data.hex())
        if len(data) != 10:
            print(f"Received data length {len(data)} is not 10, skipping...")
            return
        
        self.rovlink_sensor_data.decode(data)
        if self.rovlink_sensor_data.opcode == RovlinkFrameType.SENSOR_EULER_ANGLE:
            self.bno055.decode(self.rovlink_sensor_data.payload)
            print(f"Received Euler angle data: {self.bno055.roll}, {self.bno055.pitch}, {self.bno055.yaw}")

        # elif self.rovlink_sensor_data.opcode == RovlinkFrameType.SENSOR_CAB_TEMP_HUMID_PRESS:
        elif self.rovlink_sensor_data.opcode == RovlinkFrameType.SENSOR_WATER_TEMP_DEPTH_PRESS:
            self.m10.decode(self.rovlink_sensor_data.payload)
            print(f"Received water temperature, depth, and pressure data: {self.m10.temperature}, {self.m10.depth}, {self.m10.pressure}")


    def get_data(self):
        return self.bno055.roll , self.bno055.pitch, self.bno055.yaw, self.m10.depth/100, self.m10.temperature/100

DataDecoder = RovLinkDataDecoder()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket_1:
    client_socket_1.connect(server_1_address)
    print(f"Connected to server 1 at {server_1_ip}:{server_1_port}")

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket_2:
        client_socket_2.connect(server_2_address)
        print(f"Connected to server 2 at {server_2_ip}:{server_2_port}")

        while True:
            data = client_socket_1.recv(BUFFER_SIZE)
            if not data:
                print("No data received, closing connection.")
                break
            # print(data.hex())
            DataDecoder.decode(data)
            RovDataList =  DataDecoder.get_data()

            send_data = ':'.join(str(item) for item in RovDataList)
            print(send_data)
            client_socket_2.send(send_data.encode('utf-8'))
            time.sleep(0.02)

print("Connection closed.")
