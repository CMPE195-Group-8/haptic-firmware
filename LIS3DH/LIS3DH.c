/*
 * LIS3DH.cpp
 *
 *  Created on: Feb 2, 2024
 *      Author: luonggene
 */

#include "LIS3DH.h"
#include "math.h"
#include <stdio.h>

LIS3DH_t LIS3DH_init(
				 SPI_HandleTypeDef* spi_channel,
				 uint16_t gpio_pin,
				 GPIO_TypeDef* gpio_port)
{
	static LIS3DH_t LIS3DH;
	LIS3DH.spi_channel = spi_channel;
	LIS3DH.gpio_pin = gpio_pin;
	LIS3DH.gpio_port = gpio_port;

	wr_reg(ctrl5, 0x80, LIS3DH);
	HAL_Delay(50);
	wr_reg(ctrl1, 0x77, LIS3DH);
	HAL_Delay(50);
	wr_reg(ctrl3, 0x00, LIS3DH);
	HAL_Delay(50);
	wr_reg(ctrl4, 0x00, LIS3DH);
	HAL_Delay(50);

	return LIS3DH;
}

acceleration_t LIS3DH_read_acc(LIS3DH_t LIS3DH)
{
	acceleration_t read;
	uint8_t lo, hi;
	int16_t total;

	lo = rd_reg(x_l, LIS3DH);
	hi = rd_reg(x_h, LIS3DH);
	total = lo | (hi << 8);
	if(total >= 0){
		read.x = (float)total * 2.0 /(0x7fff);
	}else{
		read.x = (float)total * 2.0 /(0x8000);
	}

	lo = rd_reg(y_l, LIS3DH);
	hi = rd_reg(y_h, LIS3DH);
	total = lo | (hi << 8);
	if(total >= 0){
		read.y = (float)total * 2.0 /(0x7fff);
	}else{
		read.y = (float)total * 2.0 /(0x8000);
	}

	lo = rd_reg(z_l, LIS3DH);
	hi = rd_reg(z_h, LIS3DH);
	total = lo | (hi << 8);
	if(total >= 0){
		read.z = (float)total * 2.0 /(0x7fff);
	}else{
		read.z = (float)total * 2.0 /(0x8000);
	}


	return read;
}

float atan2_to_degrees(float y, float x)
{
  return (float)(atan2(y, x) * 180.0f / M_PI);
}


float LIS3DH_get_angle(LIS3DH_t LIS3DH){
	acceleration_t tmp;
	tmp = LIS3DH_read_acc(LIS3DH);
	return atan2_to_degrees(tmp.x, tmp.z);
}

void wr_reg(uint8_t addr,
			uint8_t data,
			LIS3DH_t LIS3DH)
{
	uint8_t TxData[2];
	HAL_GPIO_WritePin(LIS3DH.gpio_port, LIS3DH.gpio_pin, GPIO_PIN_RESET);
	TxData[0] = addr;
	TxData[1] = data;
	HAL_SPI_Transmit(LIS3DH.spi_channel, TxData, sizeof(TxData), 100);
	HAL_GPIO_WritePin(LIS3DH.gpio_port, LIS3DH.gpio_pin, GPIO_PIN_SET);
}

uint8_t rd_reg(uint8_t addr,
			   LIS3DH_t LIS3DH)
{
	uint8_t TxData[2];
	uint8_t RxData[2];
	HAL_GPIO_WritePin(LIS3DH.gpio_port, LIS3DH.gpio_pin, GPIO_PIN_RESET);
	TxData[0] = addr | 0x80;
	TxData[1] = 0x0;
	HAL_SPI_TransmitReceive(LIS3DH.spi_channel, TxData, RxData, sizeof(TxData), 100);
	HAL_GPIO_WritePin(LIS3DH.gpio_port, LIS3DH.gpio_pin, GPIO_PIN_SET);
	return RxData[1];
}



