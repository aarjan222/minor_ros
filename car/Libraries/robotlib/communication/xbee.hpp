#ifndef _XBEE_H_
#define _XBEE_H_

#include "stdint.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stm32f4xx.h"
#include <stdio.h>

// #include "robotlib/joystick/joystick.hpp"

#define XBeeDataDelim 0x7e
#define XBeeDataEscape 0x7d
#define Zigbee_Transmit_Request 0x10
#define Explicit_Addressing_Zigbee_Command_Frame 0x11
#define Remote_Command_Request 0x17
#define Create_Source_Route 0x21
#define AT_Command_Response 0x88
#define Modem_Status 0x8A
#define Zigbee_Transmit_Status 0x8B
#define Zigbee_Receive_Packet 0x90
#define Zigbee_Explicit_Rx_Indicator 0x91
#define Zigbee_IO_Data_Sample_Rx_Indicator 0x92
#define XBee_Sensor_Read_Indicator 0x94
#define Node_Identification_Indicator 0x95
#define Remote_Command_Response 0x97
#define Extended_Modem_Status 0x98

#define Packet_Misaligned 0x11
#define Packet_Error 0x12
#define Packet_Checksum_Error 0x12
#define Packet_Success 0x00

#define Payload_Size 0x08

#define UART_RECEIVE_BUFFER_SIZE 64
#define UART_INPUT_BUFFER_SIZE 512

// static XBeeUartHandle hndl;
// static uint16_t oldPos = 0;
// static uint16_t newPos = 0;
// static uint16_t readPos = 0;
// static XBeeReceivePacket rpac;

// extern UART_HandleTypeDef huart2;
// extern DMA_HandleTypeDef hdma_usart2_rx;

// static uint8_t uartRxBuffer[UART_RECEIVE_BUFFER_SIZE];
// static uint8_t uartBuffer[UART_INPUT_BUFFER_SIZE];

typedef struct XBeeRecData
{
    uint8_t delim;
    uint8_t msblength;
    uint8_t lsblength;
    uint8_t frameID;
    uint32_t receiveraddressUP;
    uint32_t receiveraddressLW;
    uint16_t srcaddress;
    uint8_t recoption;
    uint8_t payload[8];
    uint8_t checksum;
} XBeeReceivePacket;

typedef struct XBeeTranData
{
    uint8_t delim;
    uint8_t msblength;
    uint8_t lsblength;
    uint8_t frametype;
    uint8_t frameid;
    uint32_t receiveraddressUP;
    uint32_t receiveraddressLW;
    uint16_t networkdestaddress;
    uint8_t srcendpoint;
    uint8_t destendpoint;
    uint16_t cid;
    uint16_t pfid;

} XBeeTransmitPacket;

class Xbee
{
public:
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef *hdma_usart_rx;

    uint16_t oldPos;
    uint16_t newPos;
    uint16_t readPos;

    XBeeReceivePacket rpac;
    uint32_t last_updated_tick = 0;

    uint8_t uartRxBuffer[UART_RECEIVE_BUFFER_SIZE];
    uint8_t uartBuffer[UART_INPUT_BUFFER_SIZE];
    Xbee(UART_HandleTypeDef *, DMA_HandleTypeDef *);
    void XBeeSetRxBuffer(uint8_t *);
    uint8_t XBeeDecodePacket();
    bool XBeeReadPacket();
    void uartCallBack(uint16_t);
    void init(void);
    void get_data(uint8_t *);
};

#endif