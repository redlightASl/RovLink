#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>


struct crc_cfg
{
    uint8_t bit_width;
    uint32_t poly;
    uint32_t xor;
    uint32_t init_value;
    bool is_refin;
    bool is_refout;
};
typedef struct crc_cfg crc_cfg_t;

uint8_t RAW_DATA[10] = { 0xfd, 0xea, 0x52, 0x03, 0xff, 0x00, 0x00, 0x00, 0x00, 0x7c };
uint8_t TEST_DATA[8] = { 0 };

uint32_t crc32_check(const uint8_t* buffer, uint32_t len, uint32_t init_value);
uint32_t crc_generic(crc_cfg_t* crc_cfg, const uint8_t* cac, uint32_t size);

static uint32_t invert_u(const uint8_t bit_width, const uint32_t data)
{
    uint32_t rev = 0;
    for (uint8_t i = 0; i < bit_width; i++)
    {
        if (data & (1 << i))
        {
            rev |= 1 << ((bit_width - 1) - i);
        }
    }
    return rev;
}

int main(char* argv[], char* envp[])
{
    uint32_t result = 0;
    uint32_t test_result = 0;
    for (int i = 1; i < 9; i++)
    {
        TEST_DATA[i - 1] = RAW_DATA[i];
    }
    printf("Input data: ");
    for (int i = 0; i < sizeof(TEST_DATA); i++)
    {
        printf("%.2x ", TEST_DATA[i]);
    }
    printf("\n");
    crc_cfg_t cfg = {
        .bit_width = 32,
        .poly = 0x04C11DB7,
        .xor = 0xFFFFFFFF,
        .init_value = 0xFFFFFFFF,
        .is_refin = true,
        .is_refout = true
    };
    result = crc32_check(TEST_DATA, sizeof(TEST_DATA), 0xFFFFFFFF);
    test_result = crc_generic(&cfg, TEST_DATA, sizeof(TEST_DATA));
    printf("Output: %.8X\n", result);
    printf("Test Output: %.8X\n", test_result);
    if (result != test_result)
    {
        printf("Testbench FAIL!\n");
        return 1;
    }
    printf("Testbench PASS!\n");
    return 0;
}

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

uint32_t crc_generic(crc_cfg_t* crc_cfg, const uint8_t* cac, uint32_t size)
{
    uint32_t crc_res = crc_cfg->init_value;
    for (uint32_t num = 0; num < size; num++)
    {
        if (crc_cfg->is_refin)
        {
            crc_res ^= invert_u(8, cac[num]) << (crc_cfg->bit_width - 8);
        }
        else
        {
            crc_res ^= cac[num] << (crc_cfg->bit_width - 8);
        }

        for (uint8_t i = 0; i < 8; i++)
        {
            if (crc_res & (1 << (crc_cfg->bit_width - 1)))
            {
                crc_res <<= 1;
                crc_res ^= crc_cfg->poly;
            }
            else
            {
                crc_res <<= 1;
            }
        }
    }
    if (crc_cfg->is_refout)
    {
        crc_res = invert_u(crc_cfg->bit_width, crc_res);
    }
    return crc_res ^ crc_cfg->xor;
}
