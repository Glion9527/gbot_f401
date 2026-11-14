/* 用 C11 编译共享契约头文件，防止它悄悄变成“只能 C++ 用”。 */

#include <stdint.h>

#include "link_v2.h"

int main(void)
{
    static const uint8_t text[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    if (link_v2_crc16(text, sizeof(text)) != 0x29B1u) {
        return 1;
    }
    if (LINK_V2_CMD_SIZE != 18u || LINK_V2_TELEM_SIZE != 30u) {
        return 1;
    }
    if (LINK_V2_CMD_OFF_CRC + 2u != LINK_V2_CMD_SIZE ||
        LINK_V2_TELEM_OFF_CRC + 2u != LINK_V2_TELEM_SIZE) {
        return 1;
    }
    return 0;
}
