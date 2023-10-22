import math as m
import numpy as np


def get_keys(d, value):
    for key, val in d.items():
         if val == value:
             return key

def sumcheck(rovlink_frame):
    # 加和校验
    sum_val = 0
    for data in rovlink_frame[1:-1]:
        sum_val += data
    return sum_val


def crc32_lut_init():
    # CRC32查找表初始化
    CRC32_LUT = []
    crc32_poly = 0xEDB88320
    for i in range(256):
        crc_l = i
        for _ in range(8):
            if crc_l & 1:
                crc_l = (crc_l >> 1) ^ crc32_poly
            else:
                crc_l >>= 1
        CRC32_LUT.append(crc_l)
    return CRC32_LUT


def crc32_check_lut(LUT, rovlink_frame):
    # 查表计算CRC32
    crc_val = 0xFFFFFFFF
    for data in rovlink_frame[1:-1]:
        crc_val = (crc_val >> 8) ^ LUT[(crc_val ^ data) & 0xFF]
    crc_val = crc_val ^ 0xFFFFFFFF  # 取反
    return crc_val


def crc32_check(rovlink_frame):
    # 动态计算CRC32
    crc32_poly = 0xEDB88320
    crc_val = 0xFFFFFFFF
    for data in rovlink_frame[1:-1]:
        crc_val ^= data
        for _ in range(8):
            if crc_val & 1:
                crc_val = (crc_val >> 1) ^ crc32_poly
            else:
                crc_val >>= 1
    crc_val = crc_val ^ 0xFFFFFFFF  # 取反
    return crc_val


if __name__ == "__main__":
    lut = crc32_lut_init()
    rovlink_temp = [0xFD, 0x11, 0x53, 0xDC, 0x37, 0x2F, 0x73, 0x00, 0x00, 0x55]
    rovlink_temp2 = [0xFD, 0x11, 0x53, 0xDC, 0x37, 0x2F, 0x74, 0x00, 0x00, 0xD0]
    rovlink_temp3 = [0xFD, 0x11, 0x53, 0xDC, 0x37, 0x38, 0x6A, 0x00, 0x00, 0x8C]
    rovlink_temp4 = [0xFD, 0x26, 0x5D, 0x04, 0xDC, 0x05, 0xDC, 0x03, 0xDC, 0xF8]
    # crc1 = crc32_check(rovlink_temp)
    # crc2 = crc32_check(rovlink_temp2)
    # crc3 = crc32_check(rovlink_temp3)
    crc1 = crc32_check_lut(lut, rovlink_temp)
    crc2 = crc32_check_lut(lut, rovlink_temp2)
    crc3 = crc32_check_lut(lut, rovlink_temp4)
    print(hex(crc1 & 0xFF))
    print(hex(crc2 & 0xFF))
    print(hex(crc3 & 0xFF))
