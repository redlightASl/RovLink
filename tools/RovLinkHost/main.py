import parser
import argparse

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


if __name__ == '__main__':
    periph = Peripheral(periph_type="Joystick", max_val=14, debug=False)
    phy = Phylayer(protocol="Serial", frame_type="STD", send_port="COM9",
                   baudrate="115200", debug=False)

    while True:
        cmd = periph.get_cmd()
        cmd[0] = int(num_map(cmd[0], -1, 1, 1100, 1900))
        cmd[1] = int(num_map(cmd[1], -1, 1, 1100, 1900))
        cmd[2] = int(num_map(cmd[2], -1, 1, 1100, 1900))
        attitude_payload = [cmd[0], cmd[1], cmd[2]]
        phy.phy_encode(opcode_name="ATTITUDE", vbit=False,
                       sbit=False, payload=attitude_payload)
        phy.phy_send(dbg_print=True)
