import numpy as np
from binascii import hexlify


OPCODE_PARSER = [
    ["NOP", "TEMP_HUMI", "WATER_STA", "ACC", "GYR", "EUL", "MAG", "SONAR_HIGHT", "SONAR_DIST", "SONAR_CIRCLE", "SONAR_SIDE", "QUATERNION", "WATERBODY", "", "", ""],
    ["WATER_DET", "HEART_BEAT", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "THRUSTER_A", "THRUSTER_B", "THRUSTER_C", "THRUSTER_D", "LIGHT_A", "LIGHT_B", "PAN", "SERVO_A", "SERVO_B", "SERVO_C", "ATTITUDE", "MOVEMENT", "DIRECTION", "", ""],
    ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "PID_KP", "PID_KI", "PID_KD", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "ENVIRONMENT_STA", "SPEED_STA", "POSITION_STA", "MACHINEARM_A_STA", "MACHINEARM_B_STA", "", "", "", "", "", "", "", "", "", ""],
    ["", "PERIPHERAL_ENA", "MODULER_ENA", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "MODE_SWITCH_A", "MODE_SWITCH_B", "MODE_SWITCH_C", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "MANIPULATOR", "MACHINE_ARM_1", "MACHINE_ARM_2", "MACHINE_ARM_3", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["VOLTAGE", "CURRENT_GAIN", "CURRENT_BIAS", "CURRENT_OUTPUT", "RESERVE_CAP", "BATTERY_A", "BATTERY_B", "", "", "", "", "", "", "", "", ""],
    ["", "STAT_THRUSTER_A", "STAT_THRUSTER_B", "STAT_THRUSTER_C", "STAT_THRUSTER_D", "STAT_LIGHT_A", "STAT_LIGHT_B", "STAT_PAN", "STAT_SERVO_A", "STAT_SERVO_B", "STAT_SERVO_C", "", "", "", "", ""],
    ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
]


def crc32_lut_init():
    # CRC32查找表初始化
    CRC32_LUT = []
    crc32_poly = 0xEDB88320
    for i in range(256):
        crc_l = i
        for _ in range(8):
            if crc_l & 1:
                crc_l = (crc_l >> 1) ^ crc32_poly
            else:
                crc_l >>= 1
        CRC32_LUT.append(crc_l)
    return CRC32_LUT


def crc32_check_lut(LUT, rovlink_frame):
    # 查表计算CRC32
    crc_val = 0xFFFFFFFF
    for data in rovlink_frame[1:-1]:
        crc_val = (crc_val >> 8) ^ LUT[(crc_val ^ data) & 0xFF]
    crc_val = crc_val ^ 0xFFFFFFFF  # 取反
    return crc_val


class RovLink(object):
    def __init__(self, frame_type="STD", debug=False):
        self._frame_type = frame_type  # STD or INN
        self._debug = debug

        self._FrameHead = 0xFD
        self._Opcode = 0x00
        self._Characode = 0x00
        self._Device_ID = 0x00
        self._DLC = 0x00
        self._Valid = 0x00
        self._Sensor = 0x00
        self._Payload = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self._FrameEnd = 0x00

        self._CRC32_LUT = crc32_lut_init()

    def Opcode_parse(self):
        return OPCODE_PARSER[self._Opcode & 0xF0][self._Opcode & 0x0F]

    def _Opcode_encode(self, opcode_name):
        for i, lines in enumerate(OPCODE_PARSER):
            for j, o in enumerate(lines):
                if (opcode_name == o):
                    self._Opcode = (i << 4) | j
                    if self._debug:
                        print("Opcode:", hex(self._Opcode))

    def _Characode_encode(self, dev_id, dlc, vbit, sbit):
        self._Device_ID = dev_id
        self._DLC = dlc
        self._Valid = vbit
        self._Sensor = sbit
        self._Characode = (self._Device_ID << 4) | (
            self._DLC << 2) | (int(self._Valid) << 1) | int(self._Sensor)
        if self._debug:
            print("Characode:", hex(self._Characode))

    def _FrameEnd_encode(self, raw_frame):
        crc32_val = crc32_check_lut(self._CRC32_LUT, raw_frame)
        self._FrameEnd = crc32_val & 0xFF
        if self._debug:
            print("FrameEnd:", hex(self._FrameEnd))

    def _Payload_encode(self, payload_data):
        if self._DLC == 0x0:  # multiple data
            self._Payload[0] = payload_data[0]
            self._Payload[1] = payload_data[1]
            self._Payload[2] = payload_data[2]
            self._Payload[3] = payload_data[3]
            self._Payload[4] = payload_data[4]
            self._Payload[5] = payload_data[5]
        elif self._DLC == 0x1:  # 1 data
            self._Payload[0] = (payload_data >> 40) & 0xFF
            self._Payload[1] = (payload_data >> 32) & 0xFF
            self._Payload[2] = (payload_data >> 24) & 0xFF
            self._Payload[3] = (payload_data >> 16) & 0xFF
            self._Payload[4] = (payload_data >> 8) & 0xFF
            self._Payload[5] = payload_data & 0xFF
        elif self._DLC == 0x2:  # 2 data
            self._Payload[0] = (payload_data[0] >> 24) & 0xFF
            self._Payload[1] = (payload_data[0] >> 16) & 0xFF
            self._Payload[2] = (payload_data[0] >> 8) & 0xFF
            self._Payload[3] = payload_data[0] & 0xFF
            self._Payload[4] = (payload_data[1] >> 8) & 0xFF
            self._Payload[5] = payload_data[1] & 0x00FF
        elif self._DLC == 0x3:  # 3 data
            self._Payload[0] = (payload_data[0] >> 8) & 0xFF
            self._Payload[1] = payload_data[0] & 0x00FF
            self._Payload[2] = (payload_data[1] >> 8) & 0xFF
            self._Payload[3] = payload_data[1] & 0x00FF
            self._Payload[4] = (payload_data[2] >> 8) & 0xFF
            self._Payload[5] = payload_data[2] & 0x00FF

        self._Payload[0], self._Payload[1] = self._Payload[1], self._Payload[0]
        self._Payload[2], self._Payload[3] = self._Payload[3], self._Payload[2]
        self._Payload[4], self._Payload[5] = self._Payload[5], self._Payload[4]

        if self._debug:
            for i in range(0, 6):
                print("Payload", i, "bit:", hex(self._Payload[i]))

    def generate(self, opcode_name: str, did: int, dlc: int, vbit: bool, sbit: bool, payload: list) -> list:
        if self._frame_type == "STD":
            self._Opcode_encode(opcode_name)
            self._Characode_encode(did, dlc, vbit, sbit)
            self._Payload_encode(payload)
            self._FrameEnd_encode([self._FrameHead, self._Opcode, self._Characode, self._Payload[0], self._Payload[1],
                                   self._Payload[2], self._Payload[3], self._Payload[4], self._Payload[5]])
            return [self._FrameHead, self._Opcode, self._Characode, self._Payload[0], self._Payload[1],
                    self._Payload[2], self._Payload[3], self._Payload[4], self._Payload[5], self._FrameEnd]
        elif self._frame_type == "INN":
            self._Opcode_encode(opcode_name)
            self._Characode_encode(did, dlc, vbit, sbit)
            self._Payload_encode(payload)
            return [self._Opcode, self._Characode, self._Payload[0], self._Payload[1], self._Payload[2], self._Payload[3], self._Payload[4], self._Payload[5]]

    def show(self, frame):
        if self._debug:
            arr = np.array(frame, dtype=np.uint8)
            hex_arr = hexlify(arr.tobytes())
            print(hex_arr.decode('ascii'))
        else:
            pass


if __name__ == "__main__":
    rovlink = RovLink("STD", debug=True)
    attitude_payload = [1500, 1500, 1500]
    leakage_payload = [0, 0, 0, 1, 1, 1]
    PID_Kp_payload = [int(123.4 * 1000), 6]
    heartbeat_payload = 12345

    # frame = rovlink.generate("ATTITUDE", 0x5, 0x3,
    #                          False, False, attitude_payload)
    # frame = rovlink.generate("WATER_DET", 0x5, 0x0,
    #                          False, False, leakage_payload)
    # frame = rovlink.generate("PID_KP", 0x5, 0x2,
    #                          False, False, PID_Kp_payload)
    frame = rovlink.generate("HEART_BEAT", 0x5, 0x1,
                             False, False, heartbeat_payload)
    rovlink.show(frame)
