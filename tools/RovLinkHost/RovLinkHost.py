from Joystick import *

import threading
import cv2
import time
import socket
import numpy as np
import sys
import logging













# class Control:
#     def __init__(self):
#         self.url = "http://192.168.137.2:8081/"
#         self.servo_angle = 100
#         self.servo_claw = 90
#         logging.basicConfig(level=logging.DEBUG,
#                     format='%(asctime)s|%(levelname)s|%(message)s',
#                     datefmt='%d-%b-%H:%M:%S',
#                     filename='data.log',
#                     filemode='w')


#     def Open(self):
#         """接收视频流"""
#         self.cap = cv2.VideoCapture(self.url)
#         self.cap.set(3, 1280)
#         self.cap.set(4, 720)
#         self.frameRate = self.cap.get(cv2.CAP_PROP_FPS)
#         th = threading.Thread(target=self.Display)
#         th.setDaemon(True)
#         th.start()

#     def Display(self):
#         """显示界面"""
#         global recv_data
#         fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
#         sz = (int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
#         int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))

#         fps = 10
#         video_out = cv2.VideoWriter()
#         video_out.open('./output.mp4',fourcc,fps,sz,True)
#         cv2.namedWindow('image',cv2.WINDOW_FREERATIO)
#         cv2.resizeWindow("image", 1280, 720)


#         while self.cap.isOpened():
#             success, frame = self.cap.read()
            
#             if frame is None:
#                 tmp_img = np.zeros([480,640,3],np.uint8)
#                 cv2.putText(tmp_img,"NO VIDEO", (200, 300),cv2.FONT_HERSHEY_DUPLEX, 1, (255, 255, 255), 3)
#                 self.cap = cv2.VideoCapture(self.url)
#                 if self.cap.isOpened():
#                     success, frame = self.cap.read()
#                 else:
#                     frame = tmp_img
#             data= recv_data.split(":")
#             #logging.info("sensor_data: roll:{} pitch:{} yaw:{} depth:{} temprature:{}".format(str(data[0]),str(data[1]),str(data[2]),str(data[3]),str(data[4])))
#             if(len(data) > 1):
#                 cv2.putText(frame,"roll: "+str(data[0]),(10, 20), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)
#                 cv2.putText(frame,"pitch: "+str(data[1]),(10, 60), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)
#                 cv2.putText(frame,"yaw: "+str(data[2]),(10, 100), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)
#                 #1100
#                 cv2.putText(frame,"depth: "+str(data[3]),(600, 20), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)
#                 cv2.putText(frame,"temprature: "+str(data[4]),(600, 60), cv2.FONT_HERSHEY_DUPLEX, 0.5, (255, 255, 255), 1)

#             frame = cv2.resize(frame,(1280,720))
#             cv2.imshow('image', frame)
#             cv2.waitKey(33)
#             video_out.write(frame)
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 video_out.release()
#                 logging.info("stop recorder")
#                 print("结束录制")
#                 exit()
#             if cv2.waitKey(1) & 0xFF == ord('w'):
#                 self.cap = cv2.VideoCapture(self.url)
#                 logging.error("video offline")
#         self.cap.release()

#     def send_controldata(self):
#         """发指令"""
#         global recv_data
#         value = get_value()
#         countime = 0
#         while True:
#             temp_value = next(value)
#             countime = countime + 1
#             #机械臂舵机角度
#             """
#             if(self.servo_claw > 70 and self.servo_claw < 176):
#                 self.servo_claw += int(temp_value[15][0])*2
#             elif(self.servo_claw == 70):
#                 if(int(temp_value[15][0])<0):
#                     self.servo_claw = 70
#                 else:
#                     self.servo_claw += int(temp_value[15][0])*2
#             elif(self.servo_claw == 176):
#                 if(int(temp_value[15][0])>0):
#                     self.servo_claw = 176
#                 else:
#                     self.servo_claw += int(temp_value[15][0])*2
#             else:
#                 pass
#             """
#             if(self.servo_claw > 69 and self.servo_claw < 177):
#                 if(temp_value[2]<-0.4):
#                     self.servo_claw += 3
#                 elif(temp_value[2]>0.4):
#                     self.servo_claw -= 3
#                 else:
#                     pass
#             elif(self.servo_claw == 69):
#                 if(temp_value[2]>0.4):
#                     self.servo_claw = 69
#                 elif(temp_value[2]<-0.4):
#                     self.servo_claw += 3
#                 else:
#                     pass
#             elif(self.servo_claw == 177):
#                 if(temp_value[2]<-0.4):
#                     self.servo_claw = 177
#                 elif(temp_value[2]>0.4):
#                     self.servo_claw -= 3
#                 else:
#                     pass
#             else:
#                 pass
#             #机械臂舵机角度
#             if(self.servo_angle > 40 and self.servo_angle < 180):
#                 self.servo_angle -= int(temp_value[9])
#                 self.servo_angle += int(temp_value[10])
#             elif(self.servo_angle == 40):
#                 self.servo_angle += int(temp_value[10])
#             else:
#                 self.servo_angle -= int(temp_value[9])
#             #推进器PWM值
#             L = 0
#             R = 0
#             x = int(my_map(temp_value[0],-1,1,500,2500))
#             y = int(my_map(temp_value[1],1,-1,500,2500))
#             z = int(my_map(temp_value[3],1,-1,500,2500))
#             if x>=1500 and y>=1500 and x>=y :   #1
#                 L = x
#                 R = y-x+1500
#             elif x>=1500 and y>1500 and x<y :   #2
#                 L = y
#                 R = y-x+1500
#             elif x<1500 and y>1500 and x+y>=3000:   #3
#                 L = x+y-1500
#                 R = y
#             elif x<1500 and y>1500 and x+y<3000:   #4
#                 L = x+y-1500
#                 R = 3000-x
#             elif x<=1500 and y<=1500 and x<y :   #5
#                 L = x
#                 R = y-x+1500
#             elif x<=1500 and y<1500 and x>=y :   #6
#                 L = y
#                 R = y-x+1500
#             elif x>1500 and y<1500 and x+y<3000:   #7
#                 L = x+y-1500
#                 R = y
#             elif x>1500 and y<1500 and x+y>=3000:   #8
#                 L = x+y-1500
#                 R = 3000-x
#             else:
#                 L = 1500
#                 R = 1500
            
#             L1 = int(my_map(L,500,2500,76,110))
#             R1 = int(my_map(R,500,2500,110,76))
#             z1 = int(my_map(z,500,2500,76,110))
#             z2 = int(my_map(z,500,2500,110,76))
            

#             L1 = int(my_map(L,500,2500,108,84))
#             R1 = int(my_map(R,500,2500,81,105))
#             z1 = int(my_map(z,500,2500,105,81))
#             z2 = int(my_map(z,500,2500,105,81))


#             send_msg(udp_socket, HOST, PORT, L1 , R1, z1, z2, self.servo_angle,self.servo_claw)
#             print("send_data: L:{} R:{} z1:{} z2:{} Angle:{} claw:{}".format(L1,R1,z1,z2,self.servo_angle,self.servo_claw))
#             recv_data = recv_msg(udp_socket)
#             print(recv_data)
#             data= recv_data.split(":")
#             # if(countime == 30):
#             #     logging.info("send_data: L:{} R:{} z1:{} z2:{} Angle:{} claw:{}".format(L1,R1,z1,z2,self.servo_angle,self.servo_claw))
#             #     logging.info("sensor_data: roll:{} pitch:{} yaw:{} depth:{} temprature:{}".format(str(data[0]),str(data[1]),str(data[2]),str(data[3]),str(data[4])))
#             #     countime = 0























HOST = '192.168.137.2'
PORT = 8008


if __name__ =="__main__":
    rovlink = RovLink()
    control = Control()
    th = threading.Thread(target=control.send_controldata)
    th.start()
