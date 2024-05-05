#include "MG90S.h"
#include "main.h"

float clamp(float val, float min, float max){
	if(val > max){
		return max;
	}else if(val < min){
		return min;
	}
	return val;
}

uint32_t angle_to_pwm(MG90S_t servo, float angle){
	float clamped_angle = clamp(angle, -90.0f, 90.0f) + 90.0f;
	float ratio = clamped_angle/ 180.0f;
	const uint32_t max_duty = ((servo.timer->ARR + 1) / 10); //2ms
	const uint32_t min_duty = ((servo.timer->ARR + 1) / 20); //1ms
	const uint32_t diff = max_duty - min_duty;
	ratio = ratio * (float)diff + (float)min_duty;
	return (uint32_t)ratio;

}

MG90S_t MG90S_init(volatile TIM_TypeDef* timer, volatile uint32_t *channel_addr){
	static MG90S_t servo;
	servo.timer = timer;
	servo.channel = channel_addr;
	return servo;
}

void MG90S_position(MG90S_t servo, float angle){
	*servo.channel = angle_to_pwm(servo, angle);
}
