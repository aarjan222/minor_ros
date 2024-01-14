#ifndef _NANO_SENSOR_
#define _NANO_SENSOR_
#include "usart.h"
#define BUFFER_SIZE 24    // size of the data bytes that is sent from the arduino
#define START_BUFFER 0x94 // byte to check if the data incomming from the nano
// stores the IMU data comming from the
struct IMU
{
    float roll, pitch, yaw;
    float ax, ay, az;
};
class Nano
{
private:
    UART_HandleTypeDef *huart; // uart handle type
    IMU _imu;                  // structure to store data of imu

public:
    Nano() {}
    void init(UART_HandleTypeDef *_huart);
    void getImuAngles(float *roll, float *pitch, float *yaw);
    void getImuAccel(float *ax, float *ay, float *az);
    void setData(uint8_t *var, uint8_t *storage, uint8_t size, uint8_t start);
    float typeConversion(uint8_t *var);
    void copyDataToStruct();
};
void setNanoBuffer(UART_HandleTypeDef *);

#endif