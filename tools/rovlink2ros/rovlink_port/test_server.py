import asyncio
import websockets
import time
import sys
import os
import random
# sys.path.append("/home/pi/robot")
# sys.path.append(".")

from rovlink import *



rovlink_control_data = RovlinkFullFrameDebug()
rovlink_control_data.valid  = False

rovlink_sensor_data = RovlinkFullFrame()
rovlink_sensor_data.header = 0xfd
rovlink_sensor_data.device = RovDeviceType.COMM_CAB
rovlink_sensor_data.valid  = False
rovlink_sensor_data.sensor = True


propeller = RovControlPropellerAData()
ptz = RovControlPtzData()
servo = RovControlServoAData()
light = RovControlLightAData()





sht30 = RovSensorCabTempHumidPressData()
bno055 = RovSensorEulerAngleData()
m10 = RovSensorWaterTempDepthPressData()
ina226 = RovBatteryVoltageData()
p30 = RovSensorDistanceSonarData()


# from utils.util import BaseControl

# 初始化指令与控制类
# base_controller = BaseControl()

# 初始化PWM
# base_controller.set_init_pwm()


async def handle_client(reader, writer):
    async def recv(reader,writer):
        try:
            while True:
                message = await reader.read(100)
                rovlink_control_data.decode(message)

                if rovlink_control_data.opcode == RovlinkFrameType.CONTROL_POSTURE:
                    propeller.decode(rovlink_control_data.payload)
                    print(propeller.a1, propeller.a2, propeller.a3)
                    # base_controller.propeller_update(propeller.a1, propeller.a2, propeller.a3)
                
                elif rovlink_control_data.opcode == RovlinkFrameType.CONTROL_SERVO_A:
                    servo.decode(rovlink_control_data.payload)
                    print(servo.pwm1,servo.pwm2)
                    # base_controller.servo_update(servo.pwm1,servo.pwm2)

                elif rovlink_control_data.opcode == RovlinkFrameType.CONTROL_PTZ:
                    ptz.decode(rovlink_control_data.payload)
                    print(ptz.th1)
                    # base_controller.ptz_update(ptz.th1)

                elif rovlink_control_data.opcode == RovlinkFrameType.CONTROL_LIGHT_B:
                    light.decode(rovlink_control_data.payload)
                    print(light.l1)
                    # base_controller.light_update(light.l1)

        except asyncio.CancelledError:
            print("Receive task cancelled")


    async def send(reader,writer):
        try:
            while True:
                message_list = []
                # sensor_data = base_controller.get_sensor_data()

                # m10.depth, m10.temperature = sensor_data[0:2]
                # m10.depth = random.randint(1, 99)
                m10.depth = 50
                m10.temperature = 10
                rovlink_sensor_data.opcode = RovlinkFrameType.SENSOR_WATER_TEMP_DEPTH_PRESS
                rovlink_sensor_data.payload = m10.encode()
                m10_message = rovlink_sensor_data.encode()
                message_list.append(m10_message)

                # # sht30.temperature, sht30.humidity = sensor_data[2:4]
                # sht30.temperature = 20
                # sht30.humidity = 30
                # rovlink_sensor_data.opcode = RovlinkFrameType.SENSOR_CAB_TEMP_HUMID_PRESS
                # rovlink_sensor_data.payload = sht30.encode()
                # sht30_message = rovlink_sensor_data.encode()
                # message_list.append(sht30_message)

                
                # # ina226.voltage = sensor_data[4]
                # ina226.voltage = 40
                # rovlink_sensor_data.opcode = RovlinkFrameType.BETTERY_VOLTAGE
                # rovlink_sensor_data.payload = ina226.encode()
                # ina226_message = rovlink_sensor_data.encode()
                # message_list.append(ina226_message)

                
                # bno055.roll, bno055.pitch, bno055.yaw = sensor_data[5:8]
                bno055.roll = 0
                bno055.pitch = 1
                bno055.yaw = 2
                rovlink_sensor_data.opcode = RovlinkFrameType.SENSOR_EULER_ANGLE
                rovlink_sensor_data.payload = bno055.encode()
                bno055_message = rovlink_sensor_data.encode()
                message_list.append(bno055_message)

                

                # p30.distance, p30.confidence = sensor_data[8:10]
                # p30.confidence = 10000
                # rovlink.opcode = RovlinkFrameType.SENSOR_DISTANCE_SONAR
                # rovlink.payload = p30.encode()
                # p30_message = rovlink_sensor_data.encode()
                # message_list.append(p30_message)

                # print(message_list)
                print("send: {}".format(bno055_message.hex()))
                for message in message_list:
                #send_back_message = m10_message + sht30_message + ina226_message + bno055_message +p30_message
                    # print(message.hex())
                    writer.write(message)
                    await writer.drain()  # 确保数据已发送
                
                # writer.write(m10_message)
                # await writer.drain()  # 确保数据已发送
                await asyncio.sleep(0.05)  # 添加一个延迟以避免过快的循环

        except asyncio.CancelledError:
            print("Send task cancelled")
        finally:
            writer.close()
            await writer.wait_closed()

    task1 = asyncio.create_task(recv(reader,writer))
    task2 = asyncio.create_task(send(reader,writer))
    await asyncio.gather(task1, task2)


async def main():
    server = await asyncio.start_server(handle_client, "127.0.0.1", 9999)
    addr = server.sockets[0].getsockname()
    print(f"Serving on {addr}")

    async with server:
        await server.serve_forever()

if __name__ == "__main__":
    asyncio.run(main())