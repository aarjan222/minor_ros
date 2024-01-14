#include "crc.hpp"

CRC_Hash::CRC_Hash(uint8_t poly)
{
        uint8_t remainder;
        int dividend = 0;

        for(; dividend < 256; ++dividend) {
                remainder = dividend << (WIDTH - 8);

                for(uint8_t b = 8; b > 0; --b) {
                        if(remainder & TOP_BIT) {
                                remainder = (remainder << 1) ^ poly;
                        }
                        else {
                                remainder = (remainder << 1);
                        }
                }
                table_[dividend] = remainder;
        }
}

uint8_t CRC_Hash::get_Hash(uint8_t *buf, uint16_t len)
{
        uint8_t data;
        uint8_t remainder = 0;

        for(uint16_t index = 0; index < len; ++index) {
                data = buf[index] ^ (remainder >> (WIDTH - 8));
                remainder = table_[data] ^ (remainder << 8);
        }

        return (remainder);
}
