#ifndef _JOYSTICK_H_
#define _JOYSTICK_H_

#include "robotlib/crypto/crc.hpp"
#include "usart.h"

#define START_BYTE (0xA5)

// Button's bit position in the byte
#define B_X (7)
#define B_Y (6)
#define B_A (5)
#define B_B (4)
#define B_UP (3)
#define B_DOWN (2)
#define B_LB (1)
#define B_RB (0)

#define B_START (7)
#define B_BACK (6)
#define B_XBOX (5)
#define B_LEFT (4)
#define B_RIGHT (3)
#define B_L3 (2)
#define B_R3 (1)

#ifndef _BV
#define _BV(x) (1 << x)
#endif

#define NUM_JOYSTICK_BYTES (8)
#define JOYSTICK_START_BYTE (START_BYTE)

struct JoystickData {
  uint8_t button1=0;
  uint8_t button2=0;
  uint8_t lt=0;
  uint8_t rt=0;
  int8_t l_hatx=0;
  int8_t l_haty=0;
  int8_t r_hatx=0;
  int8_t r_haty=0;
};

enum JoystickStartByteStatus {
  WAITING_FOR_START_BYTE,
  RECEIVING_DATA,
  WAITING_FOR_REM
};
enum JoystickStatus { DISCONNECTED, HASH_DIDNT_MATCH, OK };

class Joystick {
public:
  Joystick() = delete; // try to prevent _huart=null
  Joystick(UART_HandleTypeDef *_r_huart, UART_HandleTypeDef *_t_huart);
  Joystick(Joystick &&) = default;
  Joystick(const Joystick &) = default;
  Joystick &operator=(Joystick &&) = default;
  Joystick &operator=(const Joystick &) = default;
  ~Joystick() {}

  UART_HandleTypeDef *r_huart, *t_huart;
  CRC_Hash crc{7};

  // milliseconds since last update
  uint32_t last_updated_tick = 0;

  // store first and last bytes from the joystick
  uint8_t first_byte = 0, rem_byte = 0;

  // store joystick data from dma
  uint8_t joystick_data_dma[NUM_JOYSTICK_BYTES] = {0};

  //array of sending bytes to send data to upper part
  uint8_t transmission_data[18] = {0};

  JoystickData joystick_data;

  JoystickStartByteStatus receive_state = WAITING_FOR_START_BYTE;
  JoystickStatus status = DISCONNECTED;

  JoystickData get_data();
  void parse();
  void update();
  void init();
};

#endif // !_JOYSTICK_H_
