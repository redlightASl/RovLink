注意：**当前内容为AI生成**

# 机器人串口数据模拟器

这个目录包含了用于测试机器人串口数据转ROS格式的模拟器和测试程序。

## 文件说明

### 1. `robot_simulator.py` - 完整模拟器
- **功能**：模拟ROV机器人发送串口数据，支持网络Socket和ROS话题两种模式
- **特点**：
  - 生成真实的传感器数据（角度、深度、温度等）
  - 支持网络数据发送到Socket服务器
  - 支持ROS话题发布
  - 可同时运行两种模式

### 2. `ros_test_publisher.py` - ROS测试发布器
- **功能**：专门用于测试ROS话题，发布模拟的机器人数据
- **特点**：
  - 轻量级，专注于ROS测试
  - 生成逼真的传感器数据
  - 实时显示发布的数据

## 使用方法

### 方法1：使用完整模拟器

```bash
python3 robot_simulator.py
```

选择运行模式：
- **模式1**：网络数据模拟器（发送到Socket服务器）
- **模式2**：ROS数据发布器（发布到ROS话题）
- **模式3**：同时运行两种模式

### 方法2：使用ROS测试发布器

```bash
python3 ros_test_publisher.py
```

### 方法3：使用rosrun运行

```bash
# 先编译工作空间
catkin_make
source devel/setup.bash

# 运行ROS测试发布器
rosrun ros_topic_publishing ros_test_publisher.py
```

## 数据格式

模拟器生成的数据格式兼容 `client.py` 的标准输出格式

```
angle_x:angle_y:angle_z:depth:water_temp
```

例如：
```
15.23:-8.45:180.12:5.67:20.34
```

## 模拟数据说明

### 角度数据
- **roll（翻滚角）**：-180° 到 +180°，正弦波动
- **pitch（俯仰角）**：-180° 到 +180°，正弦波动
- **yaw（偏航角）**：0° 到 360°，持续旋转

### 环境数据

- **depth（深度）**：0-20米，上下浮动
- **water_temp（水温）**：15-25°C，缓慢变化

### 其他传感器数据

- **湿度、GPS坐标、速度、磁场、加速度、气压、电池电压、电流**等

## 测试步骤

### 1. 测试ROS话题

```bash
# 终端1：运行ROS测试发布器
python3 ros_test_publisher.py

# 终端2：监听ROS话题
rostopic echo /robot_data

# 终端3：查看话题信息
rostopic list
rostopic info /robot_data
```

### 2. 测试网络连接

```bash
# 终端1：运行网络模拟器
python3 robot_simulator.py
# 选择模式1

# 终端2：运行测试服务器
python3 server_test.py
```

### 3. 同时测试

```bash
# 终端1：运行完整模拟器，选择模式3
python3 robot_simulator.py

# 终端2：监听ROS话题
rostopic echo /robot_data

# 终端3：运行测试服务器
python3 server_test.py
```

## 配置参数

可以在代码中调整以下参数：

- **更新频率**：默认10Hz
- **角度变化速度**：可调整运动幅度
- **深度范围**：默认0-20米
- **温度范围**：默认15-25°C
- **噪声水平**：可调整随机性

## 注意事项

1. **ROS环境**：确保ROS环境已正确设置
2. **网络配置**：网络模拟器默认连接到127.0.0.1:9001
3. **权限设置**：确保Python脚本有执行权限
4. **依赖包**：需要安装rospy、socket等Python包

## 故障排除

### 调试方法

1. 使用`rostopic list`检查话题是否存在
2. 使用`rostopic echo`监听数据
3. 检查ROS日志：`rosnode info /ros_test_publisher`
4. 使用`rqt_graph`查看节点连接图
