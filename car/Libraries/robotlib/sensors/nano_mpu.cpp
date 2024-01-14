#include "nano_mpu.h"
#include <stdint.h>
#include <stdio.h>

uint8_t nanoDataBuffer;               // Stores incoming byte
bool nanoStartReading = false;        // Check if data is being read or not
uint8_t nanoStartByte = START_BUFFER; // Stores the start byte to check incomming data
uint8_t dataIndex = 0;                // Index of incoming data
bool nanoBufferFull = false;          // Check if the buffer array is full
uint8_t buffer[BUFFER_SIZE];          // Buffer array to store incoming data

// Initializes the nano object with given uart handle
void Nano::init(UART_HandleTypeDef *_huart)
{
    huart = _huart;                                  // Define the uart being used
    HAL_UART_Receive_DMA(huart, &nanoDataBuffer, 1); // Start communication in the given uart
}

// Function to get euler angle obtained from imu
void Nano::getImuAngles(float *roll, float *pitch, float *yaw)
{
    // printf("Angels\n");
    copyDataToStruct();
    *roll = _imu.roll;
    *pitch = _imu.pitch;
    *yaw = _imu.yaw;
}

// Function to get acceleration values from imu
void Nano::getImuAccel(float *ax, float *ay, float *az)
{
    // printf("Accel\n");
    copyDataToStruct();
    *ax = _imu.ax;
    *ay = _imu.ay;
    *az = _imu.az;
}

// Function to group related data from buffer in an array
void Nano::setData(uint8_t *var, uint8_t *storage, uint8_t size, uint8_t start)
{
    // printf("Data set\n");
    for (uint8_t i = 0; i < size; i++)
    {
        *(var + i) = *(storage + (start + i));
    }
}

// Function to return float from array of 4 uint8_t members
float Nano::typeConversion(uint8_t *var)
{
    void *temp = (void *)var;
    return *(float *)temp;
}

// Function to copy data from buffer to IMU structure
void Nano::copyDataToStruct()
{
    // printf("Copy\n");
    uint8_t yawdata[4], pitchdata[4], rolldata[4], ax[4], ay[4], az[4];
    if (nanoBufferFull)
    {
        // gyroscope
        setData(yawdata, buffer, 4, 0);
        setData(pitchdata, buffer, 4, 4);
        setData(rolldata, buffer, 4, 8);

        // acceleration
        setData(ax, buffer, 4, 12);
        setData(ay, buffer, 4, 16);
        setData(az, buffer, 4, 20);

        // convert buffer group of 4 bytes to float
        _imu.yaw = typeConversion(yawdata);
        _imu.pitch = typeConversion(pitchdata);
        _imu.roll = typeConversion(rolldata);
        _imu.ax = typeConversion(ax);
        _imu.ay = typeConversion(ay);
        _imu.az = typeConversion(az);
    }
}

/* This function is called in the callback function
     Sets the incomming data to buffer array
*/
void setNanoBuffer(UART_HandleTypeDef *huart)
{
    if (!nanoStartReading)
    {
        if (nanoDataBuffer == nanoStartByte) // incomming byte is the start byte
        {
            nanoStartReading = true; // will add next data in the buffer
            dataIndex = 0;
            nanoBufferFull = false;
        }
    }
    else
    {
        buffer[dataIndex] = nanoDataBuffer; // store the incoming data in the buffer with dataindex
        dataIndex++;
        if (dataIndex == BUFFER_SIZE) // next data index is buffer size which is greater than buffer can hold and data is sent
        {
            dataIndex = 0;            // reset index
            nanoStartReading = false; // stop adding data to buffer
            nanoBufferFull = true;    // tell buffer is full
        }
    }
    HAL_UART_Receive_DMA(huart, &nanoDataBuffer, 1); // enable the uart dma once again
}