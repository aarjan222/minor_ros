///////////////////////////////////////////////////////////////////////////////
//     THis is the library for using TFmini PLUS
//////////////////////////////////////////////////////////////////////////////
 /*

    @ General Information:
    -communication protocal: UART
    -DMA: Rx pin intiallized with DMA
    -baud rate: 115200 {default}
    -Number of bytes in each frame: 9
    -data bits: 8
    -stop bit: 1
    -parity bit: none

    @ Frame Format:
      byte 0: 0x59  {tfmini head}
      byte 1: 0x59  {tfmini head}
      byte 2: lower 8 bits of distance
      byte 3: upper 8 bits of distance
      byte 4: lower 8 bits of intensity
      byte 5: upper 8 bits of intensity
      byte 6: lower 8 bits of temperature
      byte 7: upper 8 bits of temperature
      byte 8: checksum

*/

#ifndef TFMINI_HPP_
#define TFMINI_HPP_

#include "stm32f4xx.h"
#include "dma.h"
#include "usart.h"
#include "gpio.h"
#include "main.h"
#include<stdio.h>

#define TF_HEAD 0x59
#define NO_OF_BYTES 9

class tfmini
{ 
  public:
    uint16_t Distance_tfmini;
    uint16_t Intensity_tfmini;
    uint16_t Temperature_tfmini;

    UART_HandleTypeDef *huart;
    uint32_t hash;
    uint8_t Recieve_UART[NO_OF_BYTES];

    tfmini();
    /*
      @ construtor of class tfmini
      @ param- address of data variable of type _UART_Handletype
      @ Returns None
    */

     tfmini(UART_HandleTypeDef *huartC);

      /*
         @ Give distance 
         @ param: buffer to store distance
         @ Return None
      */
     void startRecieving();
     void parseDistance();
     void ParseIntesity();
     void parseTemp();
     uint16_t getDistance();
     uint16_t getTemperature();
     uint16_t getIntensity();

};
#endif