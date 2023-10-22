from utils import *


class RovLink(object):
    def __init__(self):
        self.FrameHead = 0xFD
        self.Opcode = 0x00
        self.Identify = 0x00
        self.DLC = 0x00
        self.Vbit = 0x00
        self.Sbit = 0x00
        self.Payload = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self.FrameEnd = 0x00

        self.real_data = []

        self.CRC32_LUT = crc32_lut_init()
        self.opcdoe_lut = {
            0x01: "NOP",
            0x11: "HeartBeat",
            0x25: "LightA",
            0x26: "LightB",
            0x27: "CameraPan"
        }

    def rovlink_verify(self, rovlink_frame):
        # 帧尾校验位
        crc32_val = crc32_check_lut(self.CRC32_LUT, rovlink_frame)
        return crc32_val & 0xFF

    def rovlink_decode(self, raw_frame):
        # RovLink译码
        if raw_frame[0] == self.FrameHead:
            if raw_frame[2] & 0x02:  # Verify Enable
                self.FrameEnd = self.rovlink_verify(raw_frame)
                if raw_frame[9] == self.FrameEnd:
                    self.Opcode = raw_frame[1]
                    self.Identify = raw_frame[2] & 0xF0
                    self.DLC = raw_frame[2] & 0x0C
                    self.Vbit = raw_frame[2] & 0x02
                    self.Sbit = raw_frame[2] & 0x01
                    self.Payload[0] = raw_frame[3]
                    self.Payload[1] = raw_frame[4]
                    self.Payload[2] = raw_frame[5]
                    self.Payload[3] = raw_frame[6]
                    self.Payload[4] = raw_frame[7]
                    self.Payload[5] = raw_frame[8]
            else:  # Verify Disable
                self.FrameEnd = raw_frame[9]
                self.Opcode = raw_frame[1]
                self.Identify = raw_frame[2] & 0xF0
                self.DLC = raw_frame[2] & 0x0C
                self.Vbit = raw_frame[2] & 0x02
                self.Sbit = raw_frame[2] & 0x01
                self.Payload[0] = raw_frame[3]
                self.Payload[1] = raw_frame[4]
                self.Payload[2] = raw_frame[5]
                self.Payload[3] = raw_frame[6]
                self.Payload[4] = raw_frame[7]
                self.Payload[5] = raw_frame[8]

    def rovlink_encode(self, Opcode, Identify, DLC, Vbit, Sbit, Payload):
        # RovLink编码
        self.rovlink_generateOpcode(Opcode)
        self.rovlink_generateID(Identify)
        self.rovlink_generateDLC(DLC)
        self.rovlink_generateVbit(Vbit)
        self.rovlink_generateSbit(Sbit)
        if self.DLC == 0x00:  # Multi
            self.Payload[0] = Payload[0]
            self.Payload[1] = Payload[1]
            self.Payload[2] = Payload[2]
            self.Payload[3] = Payload[3]
            self.Payload[4] = Payload[4]
            self.Payload[5] = Payload[5]
        elif self.DLC == 0x04:  # Single
            self.Payload[0] = (Payload[0] & 0xFF000000) >> 24
            self.Payload[1] = (Payload[0] & 0x00FF0000) >> 16
            self.Payload[2] = (Payload[0] & 0x0000FF00) >> 8
            self.Payload[3] = Payload[0] & 0x000000FF
        elif self.DLC == 0x08:  # Double
            self.Payload[0] = (Payload[0] & 0xFF000000) >> 24
            self.Payload[1] = (Payload[0] & 0x00FF0000) >> 16
            self.Payload[2] = (Payload[0] & 0x0000FF00) >> 8
            self.Payload[3] = Payload[0] & 0x000000FF
            self.Payload[4] = (Payload[1] & 0xFF00) >> 8
            self.Payload[5] = Payload[1] & 0x00FF
        elif self.DLC == 0x0C:  # Triple
            self.Payload[0] = (Payload[0] & 0xFF00) >> 8
            self.Payload[1] = Payload[0] & 0x00FF
            self.Payload[2] = (Payload[1] & 0xFF00) >> 8
            self.Payload[3] = Payload[1] & 0x00FF
            self.Payload[4] = (Payload[2] & 0xFF00) >> 8
            self.Payload[5] = Payload[2] & 0x00FF

        if self.Vbit:
            self.FrameEnd = self.rovlink_verify(
                [self.Opcode, self.Identify | self.DLC | self.Vbit | self.Sbit, Payload]
            )
        else:
            self.FrameEnd = 0x00

    def rovlink_generate(self):
        frame = []
        frame.append(self.FrameHead)
        frame.append(self.Opcode)
        frame.append(self.Identify | self.DLC | self.Vbit | self.Sbit)
        frame.append(self.Payload[0])
        frame.append(self.Payload[1])
        frame.append(self.Payload[2])
        frame.append(self.Payload[3])
        frame.append(self.Payload[4])
        frame.append(self.Payload[5])
        frame.append(self.FrameEnd)
        return frame

    def rovlink_parse(self):
        self.real_data = []
        if self.DLC == 0x00:  # Multi
            for d in self.Payload:
                self.real_data.append(d)
        elif self.DLC == 0x04:  # Single
            self.real_data.append(
                self.Payload[0] << 24
                | self.Payload[1] << 16
                | self.Payload[2] << 8
                | self.Payload[3]
            )
        elif self.DLC == 0x08:  # Double
            self.real_data.append(
                self.Payload[0] << 24
                | self.Payload[1] << 16
                | self.Payload[2] << 8
                | self.Payload[3]
            )
            self.real_data.append(self.Payload[4] << 8 | self.Payload[5])
        elif self.DLC == 0x0C:  # Triple
            self.real_data.append(self.Payload[0] << 8 | self.Payload[1])
            self.real_data.append(self.Payload[2] << 8 | self.Payload[3])
            self.real_data.append(self.Payload[4] << 8 | self.Payload[5])

    def rovlink_getOpcode(self):
        return self.opcdoe_lut[self.Opcode]

    def rovlink_generateOpcode(self, Opcode):
        self.Opcode = get_keys(self.opcdoe_lut, Opcode)

    def rovlink_getID(self):
        if self.Identify == 0x10:  # P
            return "Power"
        elif self.Identify == 0x20:  # S
            return "Ctrl"
        elif self.Identify == 0x30:  # C
            return "Comm"
        elif self.Identify == 0x40:  # M
            return "Main"
        elif self.Identify == 0x50:  # U
            return "Host"
        else:
            return "Error"

    def rovlink_generateID(self, Identify):
        if Identify == "Power":
            self.Identify = 0x10
        elif Identify == "Ctrl":
            self.Identify = 0x20
        elif Identify == "Comm":
            self.Identify = 0x30
        elif Identify == "Main":
            self.Identify = 0x40
        elif Identify == "Host":
            self.Identify = 0x50

    def rovlink_getDLC(self):
        if self.DLC == 0x00:
            return "Multi"
        elif self.DLC == 0x04:
            return "Single"
        elif self.DLC == 0x08:
            return "Double"
        elif self.DLC == 0x0C:
            return "Triple"
        else:
            return "Error"

    def rovlink_generateDLC(self, DLC):
        if DLC == "Multi":
            self.DLC = 0x00
        elif DLC == "Single":
            self.DLC = 0x04
        elif DLC == "Double":
            self.DLC = 0x08
        elif DLC == "Triple":
            self.DLC = 0x0C

    def rovlink_getVbit(self):
        if self.Vbit == 0x02:
            return "Verify: Enable"
        else:
            return "Verify: Disable"

    def rovlink_generateVbit(self, Vbit):
        if Vbit:
            self.Vbit = 0x02
        else:
            self.Vbit = 0x00

    def rovlink_getSbit(self):
        if self.Sbit == 0x01:
            return "Sensor: Enable"
        else:
            return "Sensor: Disable"

    def rovlink_generateSbit(self, Sbit):
        if Sbit:
            self.Sbit = 0x01
        else:
            self.Sbit = 0x00

    def rovlink_checkout(self):
        print("------ RovLink Data Frame Analyse ------")
        print(
            f"Opcode: {self.rovlink_getOpcode()}\t Identify: {self.rovlink_getID()}\t DLC: {self.rovlink_getDLC()}"
        )
        print(self.rovlink_getVbit())
        print(self.rovlink_getSbit())
        self.rovlink_parse()
        for d in self.real_data:
            print(f"int: {d:4d}\t hex: {hex(d)}")
        print("CRC32 Check: " + hex(self.FrameEnd))
        print("------ RovLink Data Frame Analyse ------")


if __name__ == "__main__":
    rovlink = RovLink()
    rovlink_temp = [0xFD, 0x11, 0x53, 0xDC, 0x37, 0x2F, 0x73, 0x00, 0x00, 0x55]
    rovlink_temp2 = [0xFD, 0x11, 0x53, 0xDC, 0x37, 0x2F, 0x74, 0x00, 0x00, 0xD0]
    rovlink_temp3 = [0xFD, 0x25, 0x5D, 0x04, 0xDC, 0x05, 0xDC, 0x03, 0xDC, 0xDB]
    rovlink_temp4 = [0xFD, 0x26, 0x5D, 0x06, 0xDC, 0x05, 0xDC, 0x03, 0xDC, 0x00]
    rovlink.rovlink_decode(rovlink_temp4)
    rovlink.rovlink_checkout()
    # data = [1500, 1600, 1400]
    # rovlink.rovlink_encode(
    #     Opcode="LightA", Identify="Host", DLC="Triple", Vbit=1, Sbit=1, Payload=data
    # )
    # rovlink.rovlink_checkout()
