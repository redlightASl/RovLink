FRAME_HEAD = 0xFD

OPCODE_PARSER = [
    ["NOP", "TEMP_HUMI", "WATER_STA", "ACC", "GYR", "EUL", "MAG", "SONAR_HIGHT", "SONAR_FRONT", "SONAR_CIRCLE", "SONAR_SIDE", "", "", "", "", ""],
    ["WATER_DET", "HEART_BEAT", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "THRUSTER_A", "THRUSTER_B", "THRUSTER_C", "THRUSTER_D", "LIGHT_A", "LIGHT_B", "PAN", "SERVO_A", "SERVO_B", "SERVO_C", "ATTITUDE", "", "", "", ""],
    ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "PID_KP", "PID_KI", "PID_KD", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "THRUSTER_STA", "LIGHT_STA", "PAN_STA", "SERVO_A_STA", "SERVO_B_STA", "", "", "", "", "", "", "", "", "", ""],
    ["", "PERIPHERAL_ENA", "MACHINE_ARM_A_ENA", "MACHINE_ARM_B_ENA", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "MODE_SWITCH_A", "MODE_SWITCH_B", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "MACHINE_ARM_1", "MACHINE_ARM_2", "MACHINE_ARM_3", "MACHINE_ARM_4", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["VOLTAGE", "CURRENT_GAIN", "CURRENT_BIAS", "CURRENT_OUTPUT", "RESERVE_CAP", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "CURRENT_THRUSTER_A", "CURRENT_THRUSTER_B", "CURRENT_THRUSTER_C", "CURRENT_THRUSTER_D", "CURRENT_LIGHT_A", "CURRENT_LIGHT_B", "CURRENT_PAN", "CURRENT_SERVO_A", "CURRENT_SERVO_B", "CURRENT_SERVO_C", "", "", "", "", ""],
    ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
    ["", "", "", "", "", "", "", "", "", "", "", "", "", "", "", ""],
]

class RovLink:
    def __init__(self):
        self._FrameHead = FRAME_HEAD
        self._Opcode = 0x00
        self._Device_ID = 0x00
        self._DLC = 0x00
        self._Valid = 0x00
        self._Sensor = 0x00
        self._Payload = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        self._FrameEnd = 0x00
        
    def Opcode_parse(self):
        return OPCODE_PARSER[self._Opcode & 0xF0][self._Opcode & 0x0F]
    
    def Opcode_encode(self, opcode):
        # DEBUG
        for i, lines in enumerate(OPCODE_PARSER):
            for j, o in enumerate(lines):
                if(opcode == o):
                    self._Opcode = (i << 4) & j
    
    def Characode_encode(self, Device_ID, DLC, Valid, Sensor):
        # TODO
        self._Device_ID = Device_ID
        self._DLC = DLC
        self._Valid = Valid
        self._Sensor = Sensor
    
    def FrameEnd_encode(self):
        # TODO
        self._FrameEnd = 0x00
        
    def Payload_encode(self, payload_data):
        if self._DLC == 0x00: # multiple data
            self._Payload[0] = payload_data[0]
            self._Payload[1] = payload_data[1]
            self._Payload[2] = payload_data[2]
            self._Payload[3] = payload_data[3]
            self._Payload[4] = payload_data[4]
            self._Payload[5] = payload_data[5]
        elif self._DLC == 0x01: # 1 data
            self._Payload[0] = payload_data & 0xFF000000 >> 24
            self._Payload[0] = payload_data & 0x00FF0000 >> 16
            self._Payload[0] = payload_data & 0x0000FF00 >> 8
            self._Payload[0] = payload_data & 0x000000FF
            self._Payload[0] = 0x00
            self._Payload[0] = 0x00
        elif self._DLC == 0x10: # 2 data
            self._Payload[0] = payload_data[0] & 0xFF000000 >> 24
            self._Payload[1] = payload_data[0] & 0x00FF0000 >> 16
            self._Payload[2] = payload_data[0] & 0x0000FF00 >> 8
            self._Payload[3] = payload_data[0] & 0x000000FF
            self._Payload[4] = payload_data[1] & 0xFF00 >> 8
            self._Payload[5] = payload_data[1] & 0x00FF
        elif self._DLC == 0x11: # 3 data
            self._Payload[0] = payload_data[0] & 0xFF00 >> 8
            self._Payload[1] = payload_data[0] & 0x00FF
            self._Payload[2] = payload_data[1] & 0xFF00 >> 8
            self._Payload[3] = payload_data[1] & 0x00FF
            self._Payload[4] = payload_data[2] & 0xFF00 >> 8
            self._Payload[5] = payload_data[2] & 0x00FF
            
    def Frame_trans(self):
        characode = (self._Device_ID << 4) & (self._DLC << 2) & (self._Valid << 1) & (self._Sensor)
        frame_data = [self._FrameHead, self._Opcode, characode, self._Payload[0], self._Payload[1], self._Payload[2], self._Payload[3], self._Payload[4], self._Payload[5],self._FrameEnd]
        return frame_data