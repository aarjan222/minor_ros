#include "motor.hpp"
#include "stm32f4xx.h"

Direction reverse_direction(Direction dir) {
  if (dir == CLOCKWISE)
    return ANTI_CLOCKWISE;
  return CLOCKWISE;
}

void Motor::set_direction(Direction _direction) {
  direction = _direction;
  update();
}

void Motor::set_speed(float speed) {

  if (speed < 0) {
    set_direction(ANTI_CLOCKWISE);
    speed = -1 * speed;
  } else {
    set_direction(CLOCKWISE);
  }

  if (speed > 1.0) {
    speed = 1.0;
  }

  pwm_signal = pwm_full_signal * speed;
  update();
}

void Motor::init() {
  HAL_TIM_PWM_Start(pwm_timer, pwm_timer_channel);
  set_direction(CLOCKWISE);
  set_speed(0);
}

void Motor::update() {
  GPIO_PinState direction_pin_state =
      (direction == ANTI_CLOCKWISE) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(direction_port, direction_pin, direction_pin_state);
  __HAL_TIM_SET_COMPARE(pwm_timer, pwm_timer_channel, pwm_signal);
}
