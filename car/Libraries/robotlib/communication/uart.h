#ifndef _UART_H_
#define _UART_H_

#include "robotlib/crypto/crc.hpp"
#include "usart.h"
#include <memory.h>
#include <stdio.h>

#define UART_START_BYTE (0xA5)
enum UARTReceiveByteStatus
{
  WAITING_FOR_START_BYTE,
  RECEIVING_DATA,
  WAITING_FOR_REM
};
enum UARTStatus
{
  DISCONNECTED,
  HASH_DIDNT_MATCH,
  OK
};
enum UARTMode
{
  RECEIVING,
  TRANSMITTING,
  BOTH
};

class UART
{
public:
  UART() = delete; // try to prevent _huart=null
  UART(UART_HandleTypeDef *, uint8_t, UARTMode);
  UART(UART_HandleTypeDef *, uint8_t, uint8_t, UARTMode);
  UART(UART &&) = default;
  UART(const UART &) = default;
  UART &operator=(UART &&) = default;
  UART &operator=(const UART &) = default;
  ~UART() {}

  UART_HandleTypeDef *huart;

  CRC_Hash crc{7};

  // milliseconds since last update
  uint32_t last_updated_tick = 0;

  // store first and last bytes from the UART
  uint8_t first_byte = 0, rem_byte = 0;

  // store UART data from dma
  uint8_t receiving_data_dma[40];

  // array of sending bytes to send data to upper part
  uint8_t transmission_data[40];

  uint8_t r_size, t_size;

  UARTReceiveByteStatus receive_state = WAITING_FOR_START_BYTE;
  UARTStatus status = DISCONNECTED;
  UARTMode mode = RECEIVING;

  void get_received_data(uint8_t *);
  UARTStatus receive();
  void transmit(uint8_t *);
  void init();
  void display(uint8_t *, uint8_t);
};

#endif // !_UART_H_
