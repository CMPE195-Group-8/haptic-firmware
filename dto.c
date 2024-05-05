#include "dto.h"

uint32_t clamp2(uint32_t val, uint32_t min, uint32_t max){
	if(val > max){
		return max;
	}else if(val < min){
		return min;
	}
	return val;
}

hand_t hand_init(finger_t thumb,
				 finger_t index,
				 finger_t middle,
				 finger_t ring,
				 finger_t pinky){
	static hand_t hand;
	hand.thumb = thumb;
	hand.index = index;
	hand.middle = middle;
	hand.ring = ring;
	hand.pinky = pinky;
	hand.buzz_cycles = 0;
	hand.end_of_buzz = 0;
	finger_set_servo_inverted(&hand.thumb, 0);
	finger_set_servo_inverted(&hand.index, 0);
	finger_set_servo_inverted(&hand.middle, 0);
	finger_set_servo_inverted(&hand.ring, 0);
	finger_set_servo_inverted(&hand.pinky, 0);
	return hand;
}

finger_t finger_init(LIS3DH_t imu_l1,
					 LIS3DH_t imu_l2,
					 MG90S_t servo,
					 float min_angle,
					 float max_angle,
					 uint16_t buzzer_pin,
					 GPIO_TypeDef* buzzer_port){
	finger_t finger;
	finger.imu_l1 = imu_l1;
	finger.imu_l2 = imu_l2;
	finger.servo = servo;
	if(min_angle > 90.0f){
		finger.min_angle = 90.0f;
	}else if(min_angle < -90.0f){
		finger.min_angle = -90;
	}else{
		finger.min_angle = min_angle;
	}

	if(max_angle > 90.0f){
		finger.max_angle = 90.0f;
	}else if(max_angle < -90.0f){
		finger.max_angle = -90.0f;
	}else{
		finger.max_angle = max_angle;
	}

	finger.buzzer_pin = buzzer_pin;
	finger.buzzer_port = buzzer_port;
	finger.buzzer_duration = 0;
	return finger;
}

uint32_t hand_set_buzz(hand_t *hand, uint32_t whole, uint32_t decimal){
	hand->buzz_cycles = whole * 60;
	hand->buzz_cycles += ((decimal % 10)/6 + (decimal/10) * 6);
	return hand->buzz_cycles;
}

void hand_buzz(hand_t *hand){
	if(hand->buzz_cycles){
		finger_buzzer_on(hand->thumb);
		finger_buzzer_on(hand->index);
		finger_buzzer_on(hand->middle);
		finger_buzzer_on(hand->ring);
		finger_buzzer_on(hand->pinky);
		hand->buzz_cycles--;
		if(!hand->buzz_cycles){
			hand->end_of_buzz = 1;
		}
	}
	if(hand->end_of_buzz){
		finger_buzzer_off(hand->thumb);
		finger_buzzer_off(hand->index);
		finger_buzzer_off(hand->middle);
		finger_buzzer_off(hand->ring);
		finger_buzzer_off(hand->pinky);
		hand->end_of_buzz = 0;
	}
}

uint32_t finger_get_curl(finger_t finger){
	uint32_t tmp;
	tmp = (uint32_t)(LIS3DH_get_angle(finger.imu_l1) - LIS3DH_get_angle(finger.imu_l2));
	tmp = clamp2(1000 * tmp/90, 0, 1000);
	if (tmp > finger.curr_percentage && tmp > 900){
		finger_buzzer_on(finger);
	}else{
		finger_buzzer_off(finger);
	}
	return tmp;
}

float finger_set_servo_inverted(finger_t* finger, uint32_t percentage){
	float scale = finger->max_angle - finger->min_angle;
	float percent = (float)(percentage)/1000.0f;
	float angle = finger->max_angle - scale * percent;

	MG90S_position(finger->servo, angle);
	finger->curr_percentage = 1000 - percentage;
	return angle;
}

float finger_set_servo(finger_t finger, uint32_t percentage){
	float scale = finger.max_angle - finger.min_angle;
	float percent = (float)(percentage)/1000.0f;
	float angle = finger.min_angle + scale * percent;

	MG90S_position(finger.servo, angle);
	return angle;
}

void finger_buzzer_on(finger_t finger){
	HAL_GPIO_WritePin(finger.buzzer_port, finger.buzzer_pin, GPIO_PIN_SET);
	return;
}
void finger_buzzer_off(finger_t finger){
	HAL_GPIO_WritePin(finger.buzzer_port, finger.buzzer_pin, GPIO_PIN_RESET);
}
void set_heat_pad(hand_t hand, uint32_t percentage);

