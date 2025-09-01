import socket
import serial
import time
from threading import Thread

class EightData(object):
    def __init__(self, port="COM4", baud=115200, debug = False):
        self.ser = serial.Serial(port, baud, timeout=0.5)
        self.sensor_data = []
        self.RawData = [0]* 33
        self.FrameState = 0
        self.ByteNum = 0
        self.CheckSum = 0x00
        self.debug = debug
        self.flag = False


    def read_thr(self):
        def read_loop():
            while True:
                self.read_data()
                time.sleep(0.015)

        th = Thread(target=read_loop)
        th.start()
        th.join()

    def reset(self):
        self.CheckSum = 0
        self.ByteNum = 0
        self.FrameState = 0
        self.RawData = [0]* 35
        self.testdata = ''

    def get_data(self):
        data = self.sensor_data
        return data

    def deal_data(self,datahex):
        axl = datahex[12]
        axh = datahex[13]
        ayl = datahex[14]
        ayh = datahex[15]
        azl = datahex[16]
        azh = datahex[17]
        # 仓温
        tcl = datahex[24]
        tch = datahex[25]
        # 水温
        twl = datahex[26]
        twh = datahex[27]
        # 水深
        dl = datahex[28]
        dh = datahex[29]


        k_angle = 180.0

        angle_x = (axh << 8 | axl) / 32768.0 * k_angle
        angle_y = (ayh << 8 | ayl) / 32768.0 * k_angle
        angle_z = (azh << 8 | azl) / 32768.0 * k_angle
        if angle_x >= k_angle:
            angle_x -= 2 * k_angle
        if angle_y >= k_angle:
            angle_y -= 2 * k_angle
        if angle_z >= k_angle:
            angle_z -= 2 * k_angle
        angle_x = angle_x * -1
        angle_y = angle_y
        if angle_z <= 0:
            angle_z = 360 - abs(angle_z)
        else:
            angle_z = angle_z
            if angle_z == 360:
                angle_z = 0

        deep = dh << 8 | dl

        # deep <0
        if deep > 32768:
            # deep = (65536-deep) * -1
            deep = 0

        deep /= 100
        water_temp = (twh << 8 | twl)/100

        #cabin_temp = (tch << 8 | tcl) / 100
        #print(cabin_temp)
        # print([angle_x, angle_y, angle_z, deep, water_temp])
        return [angle_x, angle_y * -1, angle_z, deep, water_temp]

    def read_data(self, data_len=74):
        #self.ser.write(bytes.fromhex(cmd_dict_to_cmd_str()))
        input_data = self.ser.read(100)
        self.ser.flushOutput()
        # print(input_data.hex())
        for data in input_data:
            if self.FrameState == 0:
                if data == 0x25 and self.ByteNum == 0:
                    self.CheckSum = data
                    self.ByteNum = 1
                elif self.ByteNum == 1:
                    self.CheckSum += data
                    self.ByteNum = 2
                elif self.ByteNum == 2:
                    self.CheckSum += data
                    self.ByteNum = 3
                elif self.ByteNum == 3:
                    self.CheckSum += data
                    self.FrameState = 1
                    self.ByteNum = 4
            elif self.FrameState == 1:
                if self.ByteNum < 37:
                    if self.ByteNum == 36:
                        if data == 0xff:
                            self.flag = True
                    self.RawData[self.ByteNum - 4] = data
                    self.CheckSum += data
                    self.ByteNum += 1
                else:
                    if self.flag == True:
                        self.sensor_data = self.deal_data(self.RawData)
                        print(self.sensor_data)
                    self.CheckSum = 0
                    self.ByteNum = 0
                    self.FrameState = 0
                    self.RawData = [0]* 33
                    self.flag = False


def cmd_dict_to_cmd_str():
    '''
    指令字典（10进制） -> 指令字符串（16进制）
    '''
    key_list = [
        'first', 'portrait', 'rotate', 'vertical', 'over', 'light', 'ptz', 'clip', 'flip', 'side', 'orient',
        'kdeep', 'mode', 'gesture', 'last'
    ]
    cmd_dict = com_dict = {
				'first': 37, 'portrait': 1500, 'rotate': 1500, 'vertical': 1500,'over': 1500, 'light': 0,
				'ptz': 1390, 'clip': 1480, 'flip': 900, 'side': 0, 'orient': 0, 'kdeep': 0, 'mode': 1,
				'gesture': 0, 'last': 33
			}
    com_list = [
        int(cmd_dict[key]) for key in key_list
    ]
    com_list = [
        hex(c)[2:].zfill(2) if idx in [0, 9, 10, 11, 12, 13, 14]
        else hex(c)[2:].zfill(4)
        for idx, c in enumerate(com_list)
    ]
    com_list = [
        c if idx in [0, 9, 10, 11, 12, 13, 14]
        else c[2:] + c[:2]
        for idx, c in enumerate(com_list)
    ]
    com_str = ''.join(com_list)
    return com_str

def main():
    HOST = '127.0.0.1'  # Server IP   172.19.123.136
    PORT = 9001  # Server端口
    # PORT = 9010  # Server端口

    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((HOST, PORT))
    # eight = EightData(port="/dev/ttyUSB0")
    eight = EightData(port="/dev/ttyUSB3")

    while True:
        try:
            eight.read_data()
            rov_msg = eight.get_data()
            print(rov_msg)

            if (len(rov_msg)==0):
                continue
            # [angle_x, angle_y, angle_z, deep, water_temp]
            send_data = str(rov_msg[0])+':'+str(rov_msg[1])+':'+str(rov_msg[2])+':'+str(rov_msg[3])+':'+str(rov_msg[4])
            print(send_data)
            client_socket.send(send_data.encode('utf-8'))
            time.sleep(0.015)
        except:
            pass

    client_socket.close()



if __name__ == '__main__':
    main()