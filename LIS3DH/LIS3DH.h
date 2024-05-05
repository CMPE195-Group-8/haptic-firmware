/*
 * LIS3DH.h
 *
 *  Created on: Feb 2, 2024
 *      Author: luonggene
 */
#include "../../Inc/main.h"

#include "LIS3DH_consts.h"
#ifndef SRC_LIS3DH_LIS3DH_H_
#define SRC_LIS3DH_LIS3DH_H_

typedef struct LIS3DH_t{
	SPI_HandleTypeDef* spi_channel;
	uint16_t gpio_pin;
	GPIO_TypeDef* gpio_port;
}LIS3DH_t;

typedef struct acceleration_t{
	float x;
	float y;
	float z;
}acceleration_t;

LIS3DH_t LIS3DH_init(SPI_HandleTypeDef* spi_channel, uint16_t gpio_pin, GPIO_TypeDef* gpio_port);
acceleration_t LIS3DH_read_acc(LIS3DH_t LIS3DH);
float LIS3DH_get_angle(LIS3DH_t LIS3DH);
void wr_reg(uint8_t addr, uint8_t data, LIS3DH_t LIS3DH);
uint8_t rd_reg(uint8_t addr, LIS3DH_t LIS3DH);

#endif /* SRC_LIS3DH_LIS3DH_H_ */
