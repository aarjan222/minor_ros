#include "tfmini.hpp"
#include "main.h"

///////////////////////////////////////////////////////////////////////////////////////
// Function definitions of tfmini class
///////////////////////////////////////////////////////////////////////////////////////

// Empty constructor

tfmini::tfmini()
{
    printf("object created \n");
}

/*
    @ construtor of class tfmini
    @ param- address of data variable of type _UART_Handletype
    @ general information:
        -communication protocal: UART
        -DMA: Rx pin intiallized with DMA
        -baud rate: 115200 {default}
        -Number of bytes in each frame: 9
        -data bits : 8
        -stop bit: 1
        -parity bit: none
    @ Returns None
*/
tfmini::tfmini(UART_HandleTypeDef *huartC)
{
    huart = huartC;
}
/*
    @ Starts UART recieving process
    @ Return None
*/
void tfmini::startRecieving()
{
    HAL_UART_Receive_DMA(huart, Recieve_UART, NO_OF_BYTES);
}

/*
    @ Function to extract distance value from 9 bytes recieved data
    @ Must be called in the callback function
*/
void tfmini::parseDistance()
{
     HAL_UART_Receive_DMA(huart,Recieve_UART, NO_OF_BYTES);
        if ((Recieve_UART[0] == TF_HEAD) && (Recieve_UART[1] == TF_HEAD))
        {        
            hash = (Recieve_UART[0] + Recieve_UART[1] + Recieve_UART[2] + Recieve_UART[3] + Recieve_UART[4] + Recieve_UART[5] + Recieve_UART[6] + Recieve_UART[7]);
            if (Recieve_UART[8] == (uint8_t)(hash))
            {
                Distance_tfmini = (((uint16_t)Recieve_UART[3] << 8) + Recieve_UART[2]);  
            }    
        }

}

/*
    @ Function to extract intensity value from 9 bytes recieved data
    @ Must be called in the callback function
*/
void tfmini::parseTemp(){
        HAL_UART_Receive_DMA(huart,Recieve_UART, NO_OF_BYTES);
        if ((Recieve_UART[0] == TF_HEAD) && (Recieve_UART[1] == TF_HEAD))
        {
            hash = (Recieve_UART[0] + Recieve_UART[1] + Recieve_UART[2] + Recieve_UART[3] + Recieve_UART[4] + Recieve_UART[5] + Recieve_UART[6] + Recieve_UART[7]);
            if (Recieve_UART[8] == (uint8_t)(hash))
            {
                Temperature_tfmini = (((uint16_t)Recieve_UART[7] << 8) + Recieve_UART[6]);      
            }
        }
}

/*
    @ Function to extract temperature value from 9 bytes recieved data
    @ Must be called in the callback function
*/
void tfmini::ParseIntesity(){
        HAL_UART_Receive_DMA(huart,Recieve_UART, NO_OF_BYTES);
        if ((Recieve_UART[0] == TF_HEAD) && (Recieve_UART[1] == TF_HEAD))
        {
            hash = (Recieve_UART[0] + Recieve_UART[1] + Recieve_UART[2] + Recieve_UART[3] + Recieve_UART[4] + Recieve_UART[5] + Recieve_UART[6] + Recieve_UART[7]);
            if (Recieve_UART[8] == (uint8_t)(hash))
            {
                Intensity_tfmini = (((uint16_t)Recieve_UART[5] << 8) + Recieve_UART[4]);      
            }
        }
}

uint16_t tfmini::getDistance() // Returns distance
{
    return Distance_tfmini;
}
uint16_t tfmini::getTemperature() // Returns temperature
{
    return Temperature_tfmini;
}
uint16_t tfmini::getIntensity() // Returns intesity
{
    return Intensity_tfmini;
}