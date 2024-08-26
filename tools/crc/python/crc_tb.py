def invert_u(bit_width, data):
    rev = 0
    for i in range(bit_width):
        if data & (1 << i):
            rev |= 1 << ((bit_width - 1) - i)
    return rev


def crc32_check(buffer, length, init_value):
    CRC32_POLY = 0xEDB88320
    crc_res = init_value

    for num in range(length):
        crc_res ^= buffer[num]
        for _ in range(8):
            crc_res = (crc_res >> 1) ^ ((crc_res & 1) * CRC32_POLY)

    return crc_res ^ 0xFFFFFFFF


class Crc:
    def __init__(
        self,
        bit_width: int,
        poly: int,
        xor: int,
        init_value: int,
        is_refin: bool,
        is_refout: bool,
    ):
        self._bit_width = bit_width
        self._poly = poly
        self._xor = xor
        self._init_value = init_value
        self._is_refin = is_refin
        self._is_refout = is_refout

    def crc_generic(self, cac, size):
        crc_res = self._init_value
        for num in range(size):
            if self._is_refin:
                crc_res ^= invert_u(8, cac[num]) << (self._bit_width - 8)
            else:
                crc_res ^= cac[num] << (self._bit_width - 8)

            for _ in range(8):
                if crc_res & (1 << (self._bit_width - 1)):
                    crc_res <<= 1
                    crc_res ^= self._poly
                else:
                    crc_res <<= 1

        if self._is_refout:
            crc_res = invert_u(self._bit_width, crc_res)

        return crc_res ^ self._xor


def main():
    RAW_DATA = [0xFD, 0xEA, 0x52, 0x03, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x7C]
    TEST_DATA = RAW_DATA[1:9]

    print("Input data: ", " ".join(f"{byte:02x}" for byte in TEST_DATA))
    crc = Crc(
        bit_width=32,
        poly=0x04C11DB7,
        xor=0xFFFFFFFF,
        init_value=0xFFFFFFFF,
        is_refin=True,
        is_refout=True,
    )

    result = crc32_check(TEST_DATA, len(TEST_DATA), 0xFFFFFFFF)
    test_result = crc.crc_generic(TEST_DATA, len(TEST_DATA))
    print("Output:     ", f"{result:08X}")
    print("Test Output:", f"{test_result:08X}")

    if result != test_result:
        print("Testbench FAIL!")
    else:
        print("Testbench PASS!")


if __name__ == "__main__":
    main()
