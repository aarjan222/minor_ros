#include "xbee.hpp"
#include "memory.h"
#include "stdio.h"

Xbee::Xbee(UART_HandleTypeDef *_huart, DMA_HandleTypeDef *_hdma)
    : huart(_huart), hdma_usart_rx(_hdma), oldPos(0), newPos(0), readPos(0) {}

uint8_t Xbee::XBeeDecodePacket()
{
    uint16_t checksum = 0x0000;
    uint8_t data[24] = {0};
    bool _escape = false;
    int i, j;
    for (i = 0, j = 0; j < 4; ++i)
    {
        if (i > 0 && uartRxBuffer[i] == XBeeDataEscape)
        {
            _escape = true;
            continue;
        }
        if (_escape)
        {
            data[j] = uartRxBuffer[i] ^ 0x20;
            _escape = false;
        }
        else
            data[j] = uartRxBuffer[i];
        if (j == 3)
            checksum += data[j];
        ++j;
    }

    uint16_t length;
    rpac.delim = data[0];
    rpac.lsblength = data[1];
    rpac.msblength = data[2];
    rpac.frameID = data[3];
    length = (data[1] << 8 | data[2]) + 4;

    switch (rpac.frameID)
    {
    case Zigbee_Receive_Packet:
        length = (data[1] << 8 | data[2]) + 4;
        for (; j < length; ++i)
        {
            if (uartRxBuffer[i] == XBeeDataEscape)
            {
                _escape = true;
                continue;
            }
            if (_escape)
            {
                data[j] = uartRxBuffer[i] ^ 0x20;
                _escape = false;
            }
            else
                data[j] = uartRxBuffer[i];
            if (j != 23)
                checksum += data[j];
            ++j;
        }
        checksum = 0xFF - (0x00FF & checksum);
        if (checksum != data[length - 1])
            return Packet_Checksum_Error;

        last_updated_tick = HAL_GetTick();

        rpac.receiveraddressUP = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];
        rpac.receiveraddressLW = (data[8] << 24) | (data[9] << 16) | (data[10] << 8) | data[11];
        rpac.srcaddress = (data[12] << 8) | data[13];
        rpac.recoption = data[14];
        for (j = 0; j < Payload_Size; ++j)
        {
            rpac.payload[j] = data[15 + j];
        }
        rpac.checksum = checksum;
        return Packet_Success;
        break;

    case Modem_Status:
        length = (data[1] << 8 | data[2]) + 4;
        for (; j < length; ++i)
        {
            if (uartRxBuffer[i] == XBeeDataEscape)
            {
                _escape = true;
                continue;
            }
            if (_escape)
            {
                data[j] = uartRxBuffer[i] ^ 0x20;
                _escape = false;
            }
            else
                data[j] = uartRxBuffer[i];
            if (j != 5)
                checksum += data[j];
            ++j;
        }
        checksum = 0xFF - (0x00FF & checksum);
        if (checksum != data[5])
            return Packet_Checksum_Error;
        rpac.recoption = data[4];
        break;

    default:
        break;
    }

    return 1;
}

void Xbee::uartCallBack(uint16_t size)
{

    if (XBeeReadPacket())
    {
        oldPos = newPos; // Update the last position before copying new data

        if (oldPos + Payload_Size >
            UART_INPUT_BUFFER_SIZE) // If the current position + new data size is
                                    // greater than the main buffer
        {
            uint16_t datatocopy =
                UART_INPUT_BUFFER_SIZE -
                oldPos; // find out how much space is left in the main buffer
            memcpy((uint8_t *)uartBuffer + oldPos, rpac.payload,
                   datatocopy); // copy data in that remaining space

            oldPos = 0; // point to the start of the buffer
            memcpy((uint8_t *)uartBuffer,
                   (uint8_t *)rpac.payload + datatocopy,
                   (Payload_Size - datatocopy));  // copy the remaining data
            newPos = (Payload_Size - datatocopy); // update the position
        }
        else
        {
            memcpy((uint8_t *)uartBuffer + oldPos, rpac.payload,
                   Payload_Size);
            newPos = Payload_Size + oldPos;
        }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(huart, (uint8_t *)uartRxBuffer, UART_RECEIVE_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT);
}

bool Xbee::XBeeReadPacket()
{
    uint8_t test = XBeeDecodePacket();
    HAL_UARTEx_ReceiveToIdle_DMA(this->huart, this->uartRxBuffer, UART_RECEIVE_BUFFER_SIZE);
    return test == 0;
}

void Xbee::init(void)
{
    printf("trying to dma init \n");
    HAL_UARTEx_ReceiveToIdle_DMA(this->huart, this->uartRxBuffer, UART_RECEIVE_BUFFER_SIZE);
    __HAL_DMA_DISABLE_IT(hdma_usart_rx, DMA_IT_HT);
    printf("dma init done\n");
}

void Xbee::get_data(uint8_t *received_data)
{
    memcpy(received_data, rpac.payload, 8);
}
