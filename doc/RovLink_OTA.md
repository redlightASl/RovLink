# RovLink OTA标准

RovLink提供了一个标准的串口OTA Bootloader，可以通过用户选定的 `LinkCOM` 与上位机连接实现OTA升级。

> [!NOTE]
>
> 由FinNAV支持的Bootloader程序被称为**Caviar 鱼子**，下面以Caviar为受控端实例讲解具体实现

在多设备组成的RovLink连接中，在目标设备之前的所有RovLink中间设备都会接收来自上位机（源设备）的OTA升级数据，数据不会被加密，也不应当被拦截

## OTA基本实现

RovLink提供的OTA功能将设备ROM划分为三个区域：

* **Caviar Bootloader**：基于FinNAV OTA组件的嵌入式Bootloader
* **Golden Image**：RovLink设备的主程序，能通过RovLink控制整个机器人
* **MultiBoot Image**：通过OTA烧录到设备的新程序，会在下次设备重启后被Caviar加载到 `Golden Image`

### OTA协议

**RovLink的OTA功能基于RovLink突发传输特性**。

OTA请求只能由上位机作为源设备发起，通过发送一帧OTA起始帧，主设备向目标设备请求进行OTA烧录，并随后利用突发传输的方式向目标设备连续发送烧录数据直到OTA文件发送完毕，最后主设备向目标设备发送一帧结束帧

在OTA协议中，**握手协议作为必选项**。也就是说目标设备需要对每一帧的数据进行校验，如果数据正确则向上位机回传空白的确认帧，当且仅当上位机收到一帧确认帧后才可以继续下一帧数据的发送

### OTA数据帧

OTA起始帧、OTA结束帧、OTA数据帧的Payload格式都保持一致，且按照`DLC=2'b01`的方式处理

OTA起始帧和OTA结束帧的特殊点在于`Sbit=1`，待升级设备在接收到`Opcode=0x1F`的帧后，根据Sbit判断这是起始帧、结束帧还是数据帧。若设备处于非OTA模式，第一个被接收到的Sbit=1的OTA帧会被接受，该帧被视为起始帧，设备转入OTA模式并进行烧录；随后的所有数据帧都应保证`Sbit=0`，否则OTA将立即停止；正常情况下，最后发送的数据帧必须保证`Sbit=1`，让OTA正常停止，设备自动重启完成OTA流程

OTA数据帧的Payload格式请参考[RovLink标准ISA文档](./RovLink_ISA.md)

### OTA流程

按照上电顺序开始的OTA流程如下：

1. 设备执行厂商ROM Bootloader和基础的堆栈初始化Bootloader，跳转到Caviar
2. Caviar对基础的机器人外设初始化，防止设备出现失控情况
3. Caviar检查MultiBoot Image是否存在；如果不存在，跳转到步骤5
5. 如果存在，Caviar将MultiBoot Image复制到Golden Image
6. 跳转到Golden Image
7. Golden Image修改设备的中断向量表基地址
8. 执行Golden Image主程序

在主程序运行后，如果出现OTA请求，处理流程如下：

1. RovLink设备接收到OTA请求
1. Golden Image进入休眠模式，只对上位机发送的心跳包进行应答
1. Golden Image连续OTA数据并存储到MultiBoot Image区域
1. 继续执行Golden Image，直到外部复位或重新启动设备

可以发现，MultiBoot Image区域的程序将永远不会被主动启动，这有助于避免传输过程中出现的安全问题

## Caviar功能实现

Caviar作为裸金属设备的主Bootloader会被存储到默认上电执行Flash地址，上电后首先执行。虽然它的功能简单，但是应当包含FinNAV的必要组件，实现基本设备控制和调试信息输出

Caviar可以不使用RovLink协议栈，但推荐支持部分RovLink协议来让设备在启动阶段暂停方便调试和紧急控制

### MultiBoot Image检查

**Caviar通过检查MultiBoot Image区域中最后一个字是否等于0x12345678来确定MultiBoot Image是否存在**

### MultiBoot Image复制

Caviar首先对Golden Image区域进行擦除，完成擦除后，分批将MultiBoot Image区域的数据分批写入到Golden Image区域

在该阶段中，Caviar需要通过`LinkCOM`输出调试信息和进度
