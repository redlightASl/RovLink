# ROS转发

支持 RovLink、RovLink-Lagency 转换至 URPC 指定的 ROS 格式，并推送到远端

## 使用说明

### 环境配置

需要使用x86_64平台的linux-ubuntu或windows-wsl2环境启动本代码。本代码在 `windows11-wsl2-docker-ros-noetic` 环境中进行了基本测试，可以保障在兼容环境下正确运行。

完成ros1 noetic配置后，将本代码库内的文件拷贝进用户新建的ROS工程中，即可开始使用

下文将以本代码的基本测试环境为例，介绍使用方法

### 启动ROS支持

首先在wsl2内启动docker环境

```shell
ros-noetic-docker
```

随后可使用指令启动ROS支持。注意后面所有与ROS相关的指令均要在docker环境内执行

```shell
roscore
```

## 启动TCP转ROS服务器

本脚本的方案为：将所有数据源（包括串口、TCP Socket、WebSocket）全部拆包并转换为标准TCP Socket数据，发送至转换服务器，在服务器端转换成ROS格式。可使用下列指令启动服务器。

```shell
source devel/setup.bash
rosrun ros_topic_publishing socket_msg_pub.py
```

## 启动串口接收客户端

注意：**如果您需要采用串口方式读取机器人数据才需要执行本步骤**

需要在windows宿主机中安装usbipd以让wsl2访问串口调试器硬件（将windows平台的COMx转移到wsl2的/dev/ttyx）

在windows下使用usbipd找到串口调试器

```cmd
usbipd list
```

得到如下所示的结果

```cmd
Connected:
BUSID  VID:PID    DEVICE                                                        STATE
2-1    1a86:7523  USB-SERIAL CH340 (COM6)                                       Shared
2-3    046d:c53f  USB 输入设备                                                  Not shared
2-4    27c6:659a  Goodix MOC Fingerprint                                        Not shared
2-5    0489:e111  MediaTek Bluetooth Adapter                                    Not shared
5-1    04f2:b7e8  Integrated Camera, Integrated IR Camera, Camera DFU Device    Shared

Persisted:
GUID                                  DEVICE

```

其中 `2-1    1a86:7523  USB-SERIAL CH340 (COM6)` 是串口调试器，其中 `2-1` 即为对应序号

首先将设备 `2-1` 选择为wsl和windows公用设备

```cmd
usbipd bind --busid 2-1
```

如果第一次插入串口调试器，可能在使用此指令之前，设备STATE为Not shared，使用之后变成Shared

随后将串口调试器重新挂载到wsl2的 `/dev/ttyUSB0`

```cmd
usbipd attach --wsl --busid 2-1
```

此后，可以在wsl2的shell界面输入指令查看是否挂载成功

```shell
>ls /dev/tty*
/dev/tty    /dev/tty15  /dev/tty22  /dev/tty3   /dev/tty37  /dev/tty44  /dev/tty51  /dev/tty59  /dev/tty9
/dev/tty0   /dev/tty16  /dev/tty23  /dev/tty30  /dev/tty38  /dev/tty45  /dev/tty52  /dev/tty6   /dev/ttyS0
/dev/tty1   /dev/tty17  /dev/tty24  /dev/tty31  /dev/tty39  /dev/tty46  /dev/tty53  /dev/tty60  /dev/ttyS1
/dev/tty10  /dev/tty18  /dev/tty25  /dev/tty32  /dev/tty4   /dev/tty47  /dev/tty54  /dev/tty61  /dev/ttyS2
/dev/tty11  /dev/tty19  /dev/tty26  /dev/tty33  /dev/tty40  /dev/tty48  /dev/tty55  /dev/tty62  /dev/ttyS3
/dev/tty12  /dev/tty2   /dev/tty27  /dev/tty34  /dev/tty41  /dev/tty49  /dev/tty56  /dev/tty63  /dev/ttyUSB0
/dev/tty13  /dev/tty20  /dev/tty28  /dev/tty35  /dev/tty42  /dev/tty5   /dev/tty57  /dev/tty7
/dev/tty14  /dev/tty21  /dev/tty29  /dev/tty36  /dev/tty43  /dev/tty50  /dev/tty58  /dev/tty8
```

可以发现存在 `/dev/ttyUSB0`

随后可以正常使用串口脚本读写USB串口调试器了

对于RovLink-Legacy，可以使用以下脚本

```shell
python legacy_port/client.py
```

对于标准RovLink，可以使用以下脚本

```shell
python rovlink_port/serial_client.py
```

### 启动TCP接收客户端

对于采用TCP Socket进行连接的标准RovLink设备，使用以下脚本

```shell
python rovlink_port/client.py
```

### 启动CoralReef接收客户端

当前大部分标准RovLink设备都支持了CoralReef生态。CoralReef采用WebSocket传输编码为protobuf二进制数据格式的RovLink帧数据，同时使用WebSocket传输MJPEG编码的图像。因此，对于CoralReef上位机，需要使用下列脚本完成协议转换

```shell
python rovlink_port/coralreef_client.py
```

## 启动ROS转发

如果您启动转发服务器和客户端，并在客户端调试输出中看到符合期望的数据，表明服务器已经能够执行ROS转发。

完成上述操作后，可执行下列指令，使用URPC官方提供的脚本来实现ROS数据推送

```shell
source devel/setup.bash
roslaunch ros_topic_forwarding run.launch
```

