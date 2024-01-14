#include "uart.h"
#include "robotlib/maths/math.hpp"
#include <stdio.h>

UART::UART(UART_HandleTypeDef *_huart, uint8_t size, UARTMode _mode = RECEIVING)
    : huart(_huart), last_updated_tick(0), mode(_mode)
{
    if (mode == RECEIVING)
    {
        r_size = size;
        receive_state = WAITING_FOR_START_BYTE;
        status = DISCONNECTED;
    }
    else if (mode == TRANSMITTING)
    {
        t_size = size;
    }
}

UART::UART(UART_HandleTypeDef *_huart, uint8_t _r_size, uint8_t _t_size, UARTMode = BOTH)
    : huart(_huart), r_size(_r_size), t_size(_t_size), mode(BOTH)
{
    receive_state = WAITING_FOR_START_BYTE;
    status = DISCONNECTED;
}

void UART::init()
{
    if (mode == RECEIVING || mode == BOTH)
    {
        HAL_UART_Receive_DMA(huart, &first_byte, 1);
    }

    last_updated_tick = HAL_GetTick();
}

UARTStatus UART::receive()
{
    status = HASH_DIDNT_MATCH;
    if (receive_state == WAITING_FOR_START_BYTE)
    {
        if (first_byte == UART_START_BYTE)
        {
            // printf("first byte=%d\n", first_byte);
            HAL_UART_Receive_DMA(huart, receiving_data_dma, r_size);
            receive_state = RECEIVING_DATA;
            first_byte = 0x00;
        }
        else
        {
            // printf("resend again\n");
            HAL_UART_Receive_DMA(huart, &first_byte, 1);
        }

        return status;
    }

    if (receive_state == RECEIVING_DATA)
    {
        HAL_UART_Receive_DMA(huart, &rem_byte, 1);
        receive_state = WAITING_FOR_REM;
        return status;
    }

    if (receive_state == WAITING_FOR_REM)
    {
        receive_state = WAITING_FOR_START_BYTE;
        uint8_t hash = crc.get_Hash(receiving_data_dma, r_size);
        HAL_UART_Receive_DMA(huart, &first_byte, 1);
        // display(receiving_data_dma, 9);
        if (hash == rem_byte)
        {
            // printf("Hash match\n");
            status = OK;
            return status;
        }
        else
        {
            // printf("Hash fail\n");
            status = HASH_DIDNT_MATCH;
        }
    }
    return status;
}

void UART::transmit(uint8_t *t_data)
{
    transmission_data[0] = UART_START_BYTE;
    memcpy(&transmission_data[1], t_data, t_size);
    transmission_data[t_size + 1] = crc.get_Hash(transmission_data + 1, t_size);
    transmission_data[t_size + 2] = '\n';
    HAL_UART_Transmit_DMA(huart, transmission_data, t_size + 3);
}

void UART::get_received_data(uint8_t *r_data)
{
    if (this->receive() == OK)
    {
        // printf("status ok\n");
        memcpy(r_data, receiving_data_dma, r_size);
        this->last_updated_tick = HAL_GetTick();
        // display(r_data, 9);
    }
}
void UART::display(uint8_t *data, uint8_t size)
{
    // printf("display datas:\t");
    for (int i = 0; i < size; i++)
    {
        printf("%d\t", data[i]);
    }
    printf("\n");
}