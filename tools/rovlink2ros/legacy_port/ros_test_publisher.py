#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS话题测试发布器
专门用于测试ROS话题，发布模拟的机器人数据
"""

import rospy
import time
import math
import random
from ros_topic_publishing.msg import robot_data_message

class ROSTestPublisher:
    def __init__(self):
        """初始化ROS节点和发布器"""
        # 初始化ROS节点
        rospy.init_node('ros_test_publisher', anonymous=True)
        
        # 创建发布器
        self.pub = rospy.Publisher('robot_data', robot_data_message, queue_size=100)
        
        # 设置发布频率
        self.rate = rospy.Rate(10)  # 10Hz
        
        # 模拟数据参数
        self.start_time = time.time()
        
        print("ROS测试发布器初始化完成")
        print("发布话题: /robot_data")
        print("发布频率: 10 Hz")
        print("按 Ctrl+C 停止发布器")
    
    def generate_test_data(self):
        """生成测试数据"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        # 生成正弦波动的角度数据
        angle_x = 15.0 * math.sin(elapsed_time * 0.2) + random.uniform(-0.5, 0.5)  # 翻滚角
        angle_y = 10.0 * math.sin(elapsed_time * 0.15) + random.uniform(-0.3, 0.3)  # 俯仰角
        angle_z = (elapsed_time * 30.0) % 360.0 + random.uniform(-1.0, 1.0)         # 偏航角
        
        # 生成深度数据 (模拟上下浮动)
        depth = 5.0 + 2.0 * math.sin(elapsed_time * 0.1) + random.uniform(-0.1, 0.1)
        
        # 生成水温数据 (模拟温度变化)
        water_temp = 20.0 + 2.0 * math.sin(elapsed_time * 0.05) + random.uniform(-0.1, 0.1)
        
        # 限制数据范围
        angle_x = max(-180.0, min(180.0, angle_x))
        angle_y = max(-180.0, min(180.0, angle_y))
        angle_z = angle_z % 360.0
        depth = max(0.0, min(20.0, depth))
        water_temp = max(15.0, min(25.0, water_temp))
        
        return angle_x, angle_y, angle_z, depth, water_temp
    
    def publish_test_message(self):
        """发布测试消息"""
        try:
            # 生成测试数据
            angle_x, angle_y, angle_z, depth, water_temp = self.generate_test_data()
            
            # 创建ROS消息
            msg = robot_data_message()
            
            # 设置主要数据字段
            msg.roll = angle_x
            msg.pitch = angle_y
            msg.yaw = angle_z
            msg.depth = depth
            msg.cabinTemp = water_temp
            
            # 设置其他字段的模拟值
            msg.cabinHumi = 60.0 + 5.0 * math.sin(time.time() * 0.1)  # 湿度变化
            msg.longitude = 116.3 + 0.001 * math.sin(time.time() * 0.05)  # 经度微调
            msg.latitude = 39.9 + 0.001 * math.cos(time.time() * 0.05)    # 纬度微调
            msg.speed = 0.5 + 0.2 * math.sin(time.time() * 0.2)           # 速度变化
            msg.times = int(time.time())                                   # 时间戳
            msg.magneticField = 50.0 + 2.0 * math.sin(time.time() * 0.15) # 磁场变化
            msg.acceleratedSpeed = 9.8 + 0.1 * math.sin(time.time() * 0.3) # 加速度变化
            msg.cabinPres = 101.3 + 0.5 * math.sin(time.time() * 0.08)    # 气压变化
            msg.batteryVol = 24.0 - 0.1 * (time.time() - self.start_time) / 3600  # 电池电压缓慢下降
            msg.clawCur = 0.1 + 0.05 * math.sin(time.time() * 0.5)        # 电流变化
            
            # 发布消息
            self.pub.publish(msg)
            
            # 打印发布信息
            print(f"[{time.strftime('%H:%M:%S')}] 发布数据: "
                  f"roll={angle_x:6.2f}°, pitch={angle_y:6.2f}°, yaw={angle_z:6.2f}°, "
                  f"depth={depth:5.2f}m, temp={water_temp:5.2f}°C")
            
            return True
            
        except Exception as e:
            print(f"发布消息失败: {e}")
            return False
    
    def run(self):
        """运行发布器主循环"""
        print("开始发布测试数据...")
        print("-" * 80)
        
        try:
            while not rospy.is_shutdown():
                # 发布测试消息
                self.publish_test_message()
                
                # 控制发布频率
                self.rate.sleep()
                
        except KeyboardInterrupt:
            print("\n收到停止信号，正在关闭发布器...")
        finally:
            print("ROS测试发布器已关闭")

def main():
    """主函数"""
    try:
        # 创建并运行发布器
        publisher = ROSTestPublisher()
        publisher.run()
        
    except Exception as e:
        print(f"程序运行出错: {e}")
        rospy.logerr(f"程序运行出错: {e}")

if __name__ == "__main__":
    main()
