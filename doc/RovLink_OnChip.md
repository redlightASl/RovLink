# RovLink 片上互联

RovLink提供了片上总线的支持，允许的片上总线包括：

* AMBA4 AXI4-Full/Lite
* AHB4-Full/Lite

RovLink基于上述总线的突发传输功能在片上设备间交换内部帧。

* APB4

此外，RovLink允许用户通过APB4总线分两次以大端传输一个内部帧的高字和低字

推荐用户在片上使用RovLink的32位压缩指令帧，从而在一个总线时钟内完成片上设备的数据交互

> TODO