import sys
import serial
import pygame
import parser
import argparse
import time


BYTESIZE = 8
PARITY = 'N'
STOPBITS = 1
TIMEOUT = 0.1

# Parser
parser = argparse.ArgumentParser(description = 'ROV_CommandPlatform')
parser.add_argument('--read_port',default = "COM31", type=str,help ='receive data uart port')
parser.add_argument('--send_port',default = "COM5", type=str,help='send command uart port')
parser.add_argument('--baudrate',default = "115200", type=str,help='uart baudrate')
args = parser.parse_args()

read_port = args.read_port
send_port = args.send_port
baudrate = args.baudrate
# print(read_port)
# print(send_port)
# print(baudrate)

read_ser = serial.Serial(port=read_port,
                         baudrate=baudrate,
                         bytesize=BYTESIZE,
                         parity=PARITY,
                         stopbits=STOPBITS,
                         timeout=TIMEOUT)

send_ser = serial.Serial(port=send_port,
                         baudrate=baudrate,
                         bytesize=BYTESIZE,
                         parity=PARITY,
                         stopbits=STOPBITS,
                         timeout=TIMEOUT)

# 指令列表
CommandTable = [21,1500,1500,1500]

# 数据列表
DataTable = [21,24.3,0xFF,0xFF]

# def check_events(self):
#     """监视键盘和鼠标事件"""
#     for event in pygame.event.get():
#         if event.type == pygame.QUIT:
#             sys.exit()
#         # 响应键盘按下/抬起
#         elif event.type == pygame.KEYDOWN:
#             self.check_keydown_events(event)
#         elif event.type == pygame.KEYUP:
#             self.check_keyup_events(event)
#         # elif event.type == pygame.MOUSEBUTTONDOWN:
#         #     mouse_pos = pygame.mouse.get_pos()
#         #     self._check_play_button(mouse_pos)
               
# def check_keydown_events(self, event):
#     """按键按下"""
#     if event.key == pygame.K_RIGHT:
#         self.ship.moving_right = True
#     elif event.key == pygame.K_LEFT:
#         self.ship.moving_left = True
#     elif event.key == pygame.K_UP:
#         self.ship.moving_up = True
#     elif event.key == pygame.K_DOWN:
#         self.ship.moving_down = True
#     elif event.key == pygame.K_p:
#         self._check_P_press()
#     elif event.key == pygame.K_q:
#         sys.exit()
#     elif event.key == pygame.K_z:
#         self._fire_bullet()

# def check_keyup_events(self, event):
#     """按键抬起"""
#     if event.key == pygame.K_RIGHT:
#         self.ship.moving_right = False
#     elif event.key == pygame.K_LEFT:
#         self.ship.moving_left = False
#     elif event.key == pygame.K_UP:
#         self.ship.moving_up = False
#     elif event.key == pygame.K_DOWN:
#         self.ship.moving_down = False

if __name__ == '__main__':
    while True:
        send_ser.write(1)
        # data = read_ser.read(1)
        # time.sleep(0.1)
        # if data:
        #     send_ser.write(data)
        # time.sleep(0.1)
    ser.close()
