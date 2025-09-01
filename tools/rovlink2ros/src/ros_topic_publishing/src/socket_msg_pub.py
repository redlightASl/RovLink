#!/usr/bin/python

import rospy
from ros_topic_publishing.msg import robot_data_message
import socket
import time


def read_socket_and_publish():
    rospy.init_node('robot_data', anonymous=True)
    pub = rospy.Publisher('robot_data', robot_data_message, queue_size=100)
    rate = rospy.Rate(10)
    ros_msg = robot_data_message()

    HOST = '127.0.0.1'  # 绑定到所有可用网络接口
    PORT = 9001  # Server端口
    # PORT = 9010

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    # server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 允许地址重用
    server_socket.bind((HOST, PORT))
    server_socket.listen()

    print('Waiting for a connection...')
    client_socket, client_address = server_socket.accept()

    while not rospy.is_shutdown():
        data = client_socket.recv(1024)
        if not data:
            break
        rov_msg = data.decode('utf-8').split(':')

        # [angle_x, angle_y, angle_z, deep, water_temp]
        try:
            ros_msg.roll = float(rov_msg[1])
            ros_msg.pitch = float(rov_msg[0]) 
            ros_msg.yaw = float(rov_msg[2])
            ros_msg.depth = float(rov_msg[3])
            ros_msg.cabinTemp = float(rov_msg[4])
        except:
            pass
        
        pub.publish(ros_msg)
        time.sleep(0.2)

    rate.sleep()

if __name__ == '__main__':
    try:
        read_socket_and_publish()
    except rospy.ROSInterruptException:
        pass
