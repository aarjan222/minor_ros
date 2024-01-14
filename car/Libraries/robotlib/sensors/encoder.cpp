#include "encoder.hpp"
#include "stm32f4xx.h"
#include "stdio.h"

int16_t Encoder::get_count(void)
{
  int32_t count = henc->Instance->CNT;
  if (count > 32768)
    count = count - 65535;
  return count;
}

int64_t Encoder::get_count_aggregate(void)
{
  return count_aggregate + get_count();
}

float Encoder::get_omega(void)
{
  int16_t count = get_count() - prevCount;
  int32_t sample_time = HAL_GetTick() - last_reset_time;
  omega = (2.0 * 3.14) * count / (ppr * ((float)sample_time / 1000.0));
  prevCount = get_count();
  return omega;
}

void Encoder::init(void)
{
  HAL_TIM_Encoder_Start(henc, TIM_CHANNEL_ALL);
  henc->Instance->CNT = 0;
}

void Encoder::reset_encoder_count(void)
{
  prevCount = 0;
  count_aggregate += get_count();
  henc->Instance->CNT = 0;
  last_reset_time = HAL_GetTick();
}
