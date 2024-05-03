from .ethernet import Ethernet
from .rs485 import Rs485
from .rovlink import RovLink


class Phylayer(object):
    def __init__(self, protocol="Serial", frame_type="STD", **kwargs):
        self._buffer_size = 20
        self._protocol = protocol
        self._rovlink = RovLink(frame_type=frame_type, debug=kwargs["debug"])
        self._rovlink_frame = []

        if self._protocol == "Serial":
            self._dev = Rs485(
                read_port=kwargs["read_port"], send_port=kwargs["send_port"], baudrate=kwargs["baudrate"], debug=kwargs["debug"])
        elif self._protocol == "Ethernet":
            self._dev = Ethernet(ip=kwargs["ip"], port=kwargs["port"])
        else:
            print("Error: Unknown protocol type!")

    def phy_encode(self, opcode_name, vbit, sbit, payload):
        if len(payload) == 3:
            dlc = 0x3
        elif len(payload) == 2:
            dlc = 0x2
        elif len(payload) == 1:
            dlc = 0x1
        elif len(payload) == 6:
            dlc = 0x0
        else:
            return False
        self._rovlink_frame = self._rovlink.generate(
            opcode_name=opcode_name, did=0x5, dlc=dlc, vbit=vbit, sbit=sbit, payload=payload)
        return True

    def phy_send(self, dbg_print=False):
        if self._protocol == "Serial":
            self._dev.transmit(self._rovlink_frame, dbg_print=dbg_print)

    def phy_read(self):
        # TODO
        pass

    def phy_release(self):
        self._dev.release()


if __name__ == "__main__":
    phy = Phylayer(protocol="Serial", frame_type="STD",
                   send_port="COM9", baudrate="115200", debug=True)
    phy.phy_encode(opcode_name="ATTITUDE", vbit=False,
                   sbit=False, payload=[1500, 1500, 1500])
    phy.phy_send()
    phy.phy_release()
