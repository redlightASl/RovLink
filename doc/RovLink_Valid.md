# RovLink 校验算法标准

RovLink标准帧采用CRC32算法计算帧校验位前所有数据，截取低8位作为校验位。采用的算法参数规定如下

* 基础算法：CRC32
* 算法位宽：`[31: 0]`，截取 `[7: 0]`
* CRC多项式：`0x04C11DB7`
* 初值：`0xFFFFFFFF`
* 结果异或值：`0xFFFFFFFF`

协议标准规定的算法如下所示，这里给出C语言实例

## C代码实现

```c
uint32_t crc32_check(const uint8_t* buffer, uint32_t len, uint32_t init_value)
{
    const uint32_t CRC32_POLY = 0xEDB88320; // Inversion bit sequence of 0x04C11DB7
    uint32_t crc_res = init_value;

    for (uint32_t num = 0; num < len; num++)
    {
        crc_res ^= buffer[num];
        for (int8_t bits = 8; bits > 0; bits--)
        {
            crc_res = (crc_res >> 1) ^ ((crc_res & 1) ? CRC32_POLY : 0);
        }
    }
    return crc_res ^ 0xFFFFFFFF;
}
```

## FinNAV实现

FinNAV为用户提供了通用的软件CRC校验算法的实现，更多内容可以参考 `tools/crc` 目录给出的示例。用户可以使用 `crc_generic` 调用软件CRC算法
