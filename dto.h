/*
 * dto.h
 *
 *  Created on: Feb 8, 2024
 *      Author: luonggene
 */

#ifndef SRC_DTO_H_
#define SRC_DTO_H_
#include "LIS3DH/LIS3DH.h"
#include "MG90S/MG90S.h"

#define BUFFER_SIZE 16
/// 1 servo 1 buzzer
typedef struct finger_t{
	LIS3DH_t imu_l1, imu_l2;
	MG90S_t servo;
	uint32_t curr_percentage;
	float min_angle, max_angle;

	uint32_t curl_buffer[BUFFER_SIZE];
	uint8_t buffer_index;

	uint16_t buzzer_pin;
	GPIO_TypeDef* buzzer_port;
	int16_t buzzer_duration;

}finger_t;

/// 5 fingers, 1 heatpack
typedef struct hand_t{
	finger_t thumb, index, middle, ring, pinky;
	uint32_t buzz_cycles;
	volatile TIM_TypeDef *heat_pad_timer;
	volatile uint32_t *heat_pad_channel;
	uint8_t end_of_buzz;
}hand_t;

typedef struct vrstruct_t{
	int16_t thumb; 	//A
	int16_t index; 	//B
	int16_t middle;	//C
	int16_t ring; 	//D
	int16_t pinky;	//E

}vrstruct_T;
hand_t hand_init(finger_t thumb,
				 finger_t index,
				 finger_t middle,
				 finger_t ring,
				 finger_t pinky);

finger_t finger_init(LIS3DH_t imu_l1,
					 LIS3DH_t imu_l2,
					 MG90S_t servo,
					 float min_angle,
					 float max_angle,
					 uint16_t buzzer_pin,
					 GPIO_TypeDef* buzzer_port);

uint32_t hand_set_buzz(hand_t *hand, uint32_t whole, uint32_t decimal);
void hand_buzz(hand_t *hand);
uint32_t finger_get_curl(finger_t* finger);
float finger_set_servo(finger_t finger, uint32_t percentage);
float finger_set_servo_inverted(finger_t* finger, uint32_t percentage);
void finger_buzzer_on(finger_t finger);
void finger_buzzer_off(finger_t finger);
void set_heat_pad(hand_t hand, uint32_t percentage);

#endif /* SRC_DTO_H_ */
