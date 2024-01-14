#include "robotlib/joystick/joystick.hpp"
#include "robotlib/maths/math.hpp"
#include <stdio.h>

void display(JoystickData &);
Joystick::Joystick(UART_HandleTypeDef *_r_huart, UART_HandleTypeDef *_t_huart)
    : r_huart(_r_huart), t_huart(_t_huart), last_updated_tick(0)
{
  receive_state = WAITING_FOR_START_BYTE;
  status = DISCONNECTED;
}

void Joystick::init()
{
  HAL_UART_Receive_DMA(r_huart, &first_byte, 1);
}

/*
 * INCOMMING Data:
 * Total Bytes = 10 bytes;
 * Data Format={
 *    Start Byte,
 *    [all the button state - 8 bytes],
 *    CRC Byte
 * }
 */
void Joystick::parse()
{

  HAL_GPIO_TogglePin(ST2_GPIO_Port, ST2_Pin);

  if (receive_state == WAITING_FOR_START_BYTE)
  {
    if (first_byte == JOYSTICK_START_BYTE)
    {
      HAL_UART_Receive_DMA(r_huart, joystick_data_dma, NUM_JOYSTICK_BYTES);
      receive_state = RECEIVING_DATA;
    }
    else
    {
      HAL_UART_Receive_DMA(r_huart, &first_byte, 1);
    }
    // printf("first byte %d\n", first_byte);

    return;
  }

  if (receive_state == RECEIVING_DATA)
  {
    HAL_UART_Receive_DMA(r_huart, &rem_byte, 1);
    receive_state = WAITING_FOR_REM;
    return;
  }

  if (receive_state == WAITING_FOR_REM)
  {
    HAL_UART_Receive_DMA(r_huart, &first_byte, 1);
    receive_state = WAITING_FOR_START_BYTE;
    uint8_t hash = crc.get_Hash(joystick_data_dma, NUM_JOYSTICK_BYTES);
    // printf("hash %d\n", hash);
    if (hash == rem_byte)
    {
      status = OK;
      update();
    }
    else
    {
      status = HASH_DIDNT_MATCH;
    }
  }
}

void Joystick::update()
{
  joystick_data.button1 = joystick_data_dma[0];
  joystick_data.button2 = joystick_data_dma[1];
  joystick_data.lt = joystick_data_dma[2];
  joystick_data.rt = joystick_data_dma[3];
  joystick_data.l_hatx = joystick_data_dma[4];
  joystick_data.l_haty = joystick_data_dma[5];
  joystick_data.r_hatx = joystick_data_dma[6];
  joystick_data.r_haty = joystick_data_dma[7];

  display(joystick_data);

  transmission_data[0] = START_BYTE;
  memcpy(&transmission_data[1], joystick_data_dma, 8);
  transmission_data[17] = crc.get_Hash(transmission_data + 1, 16);
  HAL_UART_Transmit_DMA(&huart2, transmission_data, 18);

  last_updated_tick = HAL_GetTick();
}

JoystickData Joystick::get_data()
{
  return joystick_data;
}
void display(JoystickData &jdata)
{
  printf("joy_data :: %d\t", jdata.button1);
  printf("%d\t", jdata.button2);
  printf("%d\t", jdata.lt);
  printf("%d\t", jdata.rt);
  printf("%d\t", jdata.l_hatx);
  printf("%d\t", jdata.l_haty);
  printf("%d\t", jdata.r_hatx);
  printf("%d\n", jdata.r_haty);
}