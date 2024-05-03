import sys
import time
import parser
import argparse
import threading

import PySimpleGUI as sg
import cv2

from command import *
from peripheral import *


# Parser
parser = argparse.ArgumentParser(description='ROV_CommandPlatform')
parser.add_argument('--read_port', default="COM31",
                    type=str, help='receive data uart port')
parser.add_argument('--send_port', default="COM5",
                    type=str, help='send command uart port')
parser.add_argument('--baudrate', default="115200",
                    type=str, help='uart baudrate')
parser.add_argument('--host_ip', default="192.168.137.2",
                    type=str, help='host device ip')
parser.add_argument('--host_port', default=8008,
                    type=int, help='host device port')
parser.add_argument('--target_ip', default="192.168.137.100",
                    type=str, help='target device ip')
parser.add_argument('--target_port', default=8008,
                    type=int, help='target device port')
parser.add_argument('--dev_type', default="Joystick",
                    type=str, help='command peripheral type')
args = parser.parse_args()


def num_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def data_transmit():
    straight_buf = 1500
    rotate_buf = 1500
    vertical_buf = 1500
    aux_buf = 1500
    ATTITUDE_FRAME_ENABLE = True
    PAN_FRAME_ENABLE = True

    periph = Peripheral(periph_type="Joystick", max_val=8, debug=False)
    phy = Phylayer(protocol="Serial", frame_type="STD", read_port="COM5", send_port="COM5",
                   baudrate="115200", debug=False)

    while not quit_flag.is_set():
        [lx, ly, rx, ry, Xbutton, Abutton, Bbutton, Ybutton] = periph.get_cmd()
        straight = int(num_map(ry, -1, 1, 1100, 1900))
        rotate = int(num_map(rx, -1, 1, 1100, 1900))
        vertical = int(num_map(ly, -1, 1, 1100, 1900))
        aux = int(num_map(lx, -1, 1, 1100, 1900))
        attitude_payload = [straight, rotate, vertical]
        thp_payload = [aux, 1500, 1500]

        if abs(straight - straight_buf) >= 2:
            straight_buf = straight
            ATTITUDE_FRAME_ENABLE = True
        if abs(rotate - rotate_buf) >= 2:
            rotate_buf = rotate
            ATTITUDE_FRAME_ENABLE = True
        if abs(vertical - vertical_buf) >= 2:
            vertical_buf = vertical
            ATTITUDE_FRAME_ENABLE = True
        if abs(aux - aux_buf) >= 2:
            aux_buf = aux
            PAN_FRAME_ENABLE = True

        if ATTITUDE_FRAME_ENABLE:
            ATTITUDE_FRAME_ENABLE = False
            phy.phy_encode(opcode_name="ATTITUDE", vbit=False,
                           sbit=False, payload=attitude_payload)
            phy.phy_send(dbg_print=True)
        if PAN_FRAME_ENABLE:
            PAN_FRAME_ENABLE = False
            phy.phy_encode(opcode_name="PAN", vbit=True,
                           sbit=False, payload=thp_payload)
            phy.phy_send(dbg_print=True)

        time.sleep(0.01)
    phy.phy_release()


def img_transmit():
    current_frame = 0

    cap = cv2.VideoCapture(3)
    num_frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
    fps = cap.get(cv2.CAP_PROP_FPS)

    sg.theme("Black")
    layout = [[sg.Text('Video Transmit', size=(15, 1), font="Helvetica 20")],
              [sg.Image(filename='', key='-image-')],
              [sg.Button('退出', size=(7, 1), pad=((600, 0), 3), font="Helvetica 14")]]
    window = sg.Window('Video Transmit', layout,
                       no_titlebar=False, location=(0, 0), finalize=True)
    image_elem = window['-image-']

    while cap.isOpened():
        event, values = window.read(timeout=0) # 读取GUI窗口事件
        # 如果点击窗口的关闭按钮或退出按钮，则退出循环
        if event in (sg.WINDOW_CLOSED, '退出'):
            break
        # 读取视频帧
        ret, frame = cap.read()
        if ret:
            resized_frame = cv2.resize(frame, (640, 480))
            img_bytes = cv2.imencode('.png', resized_frame)[1].tobytes()
            image_elem.update(data=img_bytes)
            current_frame += 1
            window.finalize()
    quit_flag.set()
    cap.release()
    window.close()


if __name__ == '__main__':
    quit_flag = threading.Event()

    t1 = threading.Thread(target=data_transmit)
    t2 = threading.Thread(target=img_transmit)
    t1.start()
    t2.start()
