#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
机器人串口数据模拟器
模拟ROV机器人发送串口数据，并转换成ROS消息格式
"""

import socket
import time
import random
import math
from threading import Thread
import rospy
from ros_topic_publishing.msg import robot_data_message

class RobotDataSimulator:
    def __init__(self, host='127.0.0.1', port=9001):
        """
        初始化机器人数据模拟器
        
        Args:
            host: 目标服务器IP地址
            port: 目标服务器端口
        """
        self.host = host
        self.port = port
        self.client_socket = None
        self.connected = False
        
        # 模拟数据参数
        self.angle_x = 0.0      # 翻滚角 (度)
        self.angle_y = 0.0      # 俯仰角 (度)
        self.angle_z = 0.0      # 偏航角 (度)
        self.depth = 0.0        # 深度 (米)
        self.water_temp = 20.0  # 水温 (摄氏度)
        
        # 运动参数
        self.angle_x_speed = 2.0    # 翻滚角变化速度 (度/秒)
        self.angle_y_speed = 1.5    # 俯仰角变化速度 (度/秒)
        self.angle_z_speed = 3.0    # 偏航角变化速度 (度/秒)
        self.depth_speed = 0.1      # 深度变化速度 (米/秒)
        self.temp_variation = 0.05  # 温度变化幅度 (度/秒)
        
        # 时间控制
        self.start_time = time.time()
        self.update_rate = 10  # 更新频率 (Hz)
        
    def connect(self):
        """连接到目标服务器"""
        try:
            self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.client_socket.connect((self.host, self.port))
            self.connected = True
            print(f"成功连接到 {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"连接失败: {e}")
            self.connected = False
            return False
    
    def disconnect(self):
        """断开连接"""
        if self.client_socket:
            self.client_socket.close()
        self.connected = False
        print("连接已断开")
    
    def generate_simulated_data(self):
        """生成模拟的传感器数据"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # 生成正弦波动的角度数据
        self.angle_x = 15.0 * math.sin(elapsed_time * self.angle_x_speed * 0.1)
        self.angle_y = 10.0 * math.sin(elapsed_time * self.angle_y_speed * 0.1)
        self.angle_z = (elapsed_time * self.angle_z_speed) % 360.0
        
        # 生成深度数据 (模拟上下浮动)
        self.depth = 5.0 + 2.0 * math.sin(elapsed_time * self.depth_speed * 0.5)
        
        # 生成水温数据 (模拟温度变化)
        self.water_temp = 20.0 + 2.0 * math.sin(elapsed_time * self.temp_variation)
        
        # 添加一些随机噪声，使数据更真实
        self.angle_x += random.uniform(-0.5, 0.5)
        self.angle_y += random.uniform(-0.3, 0.3)
        self.angle_z += random.uniform(-1.0, 1.0)
        self.depth += random.uniform(-0.1, 0.1)
        self.water_temp += random.uniform(-0.1, 0.1)
        
        # 限制数据范围
        self.angle_x = max(-180.0, min(180.0, self.angle_x))
        self.angle_y = max(-180.0, min(180.0, self.angle_y))
        self.angle_z = self.angle_z % 360.0
        self.depth = max(0.0, min(20.0, self.depth))
        self.water_temp = max(15.0, min(25.0, self.water_temp))
    
    def send_data(self):
        """发送数据到服务器"""
        if not self.connected:
            return False
        
        try:
            # 格式化数据字符串 (与原始代码格式一致)
            # [angle_x, angle_y, angle_z, deep, water_temp]
            send_data = f"{self.angle_x:.2f}:{self.angle_y:.2f}:{self.angle_z:.2f}:{self.depth:.2f}:{self.water_temp:.2f}"
            
            # 发送数据
            self.client_socket.send(send_data.encode('utf-8'))
            print(f"发送数据: {send_data}")
            return True
            
        except Exception as e:
            print(f"发送数据失败: {e}")
            self.connected = False
            return False
    
    def run_simulation(self):
        """运行模拟器主循环"""
        print("开始运行机器人数据模拟器...")
        print(f"目标服务器: {self.host}:{self.port}")
        print(f"更新频率: {self.update_rate} Hz")
        print("按 Ctrl+C 停止模拟器")
        
        try:
            while True:
                # 生成模拟数据
                self.generate_simulated_data()
                
                # 发送数据
                if self.connected:
                    self.send_data()
                else:
                    # 尝试重新连接
                    if not self.connect():
                        time.sleep(1)
                        continue
                
                # 控制更新频率
                time.sleep(1.0 / self.update_rate)
                
        except KeyboardInterrupt:
            print("\n收到停止信号，正在关闭模拟器...")
        finally:
            self.disconnect()

class ROSDataPublisher:
    """ROS数据发布器，用于测试ROS话题"""
    
    def __init__(self):
        """初始化ROS节点和发布器"""
        rospy.init_node('robot_data_simulator', anonymous=True)
        self.pub = rospy.Publisher('robot_data', robot_data_message, queue_size=100)
        self.rate = rospy.Rate(10)  # 10Hz
        
        print("ROS节点初始化完成")
        print("发布话题: /robot_data")
    
    def publish_simulated_data(self, angle_x, angle_y, angle_z, depth, water_temp):
        """发布模拟数据到ROS话题"""
        try:
            msg = robot_data_message()
            
            # 设置消息字段
            msg.roll = angle_x
            msg.pitch = angle_y
            msg.yaw = angle_z
            msg.depth = depth
            msg.cabinTemp = water_temp
            
            # 设置其他字段的默认值
            msg.cabinHumi = 60.0  # 湿度
            msg.longitude = 116.3  # 经度
            msg.latitude = 39.9    # 纬度
            msg.speed = 0.5        # 速度
            msg.times = int(time.time())  # 时间戳
            msg.magneticField = 50.0      # 磁场
            msg.acceleratedSpeed = 9.8    # 加速度
            msg.cabinPres = 101.3         # 气压
            msg.batteryVol = 24.0         # 电池电压
            msg.clawCur = 0.1             # 当前电流
            
            # 发布消息
            self.pub.publish(msg)
            print(f"ROS消息已发布: roll={angle_x:.2f}, pitch={angle_y:.2f}, yaw={angle_z:.2f}, depth={depth:.2f}, temp={water_temp:.2f}")
            
        except Exception as e:
            print(f"发布ROS消息失败: {e}")
    
    def run_ros_publisher(self):
        """运行ROS发布器"""
        print("开始运行ROS数据发布器...")
        
        try:
            while not rospy.is_shutdown():
                # 生成模拟数据
                current_time = time.time()
                elapsed_time = current_time - time.time()
                
                angle_x = 15.0 * math.sin(time.time() * 0.1)
                angle_y = 10.0 * math.sin(time.time() * 0.15)
                angle_z = (time.time() * 3.0) % 360.0
                depth = 5.0 + 2.0 * math.sin(time.time() * 0.05)
                water_temp = 20.0 + 2.0 * math.sin(time.time() * 0.02)
                
                # 发布数据
                self.publish_simulated_data(angle_x, angle_y, angle_z, depth, water_temp)
                
                # 控制发布频率
                self.rate.sleep()
                
        except KeyboardInterrupt:
            print("\n收到停止信号，正在关闭ROS发布器...")
        finally:
            print("ROS发布器已关闭")

def main():
    """主函数"""
    print("机器人串口数据模拟器")
    print("=" * 50)
    print("选择运行模式:")
    print("1. 网络数据模拟器 (发送到Socket服务器)")
    print("2. ROS数据发布器 (发布到ROS话题)")
    print("3. 同时运行两种模式")
    
    try:
        choice = input("请输入选择 (1/2/3): ").strip()
        
        if choice == "1":
            # 运行网络数据模拟器
            simulator = RobotDataSimulator()
            simulator.run_simulation()
            
        elif choice == "2":
            # 运行ROS数据发布器
            ros_publisher = ROSDataPublisher()
            ros_publisher.run_ros_publisher()
            
        elif choice == "3":
            # 同时运行两种模式
            print("同时运行网络模拟器和ROS发布器...")
            
            # 创建并启动网络模拟器线程
            simulator = RobotDataSimulator()
            network_thread = Thread(target=simulator.run_simulation)
            network_thread.daemon = True
            network_thread.start()
            
            # 运行ROS发布器
            ros_publisher = ROSDataPublisher()
            ros_publisher.run_ros_publisher()
            
        else:
            print("无效选择，退出程序")
            
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行出错: {e}")

if __name__ == "__main__":
    main()
