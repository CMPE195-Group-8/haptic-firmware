#pragma once
#include "main.h"

typedef struct MG90S_t{
	volatile TIM_TypeDef *timer;
	volatile uint32_t *channel;
}MG90S_t;

MG90S_t MG90S_init(volatile TIM_TypeDef* timer, volatile uint32_t *channel_addr);
void MG90S_position(MG90S_t servo, float angle);
uint32_t angle_to_pwm(MG90S_t servo, float angle);
