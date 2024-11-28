/*
 * shiftreg.h
 *
 *  Created on: 22 февр. 2023 г.
 *      Author: ilya0
 */

#ifndef INC_SHIFTREG_H_
#define INC_SHIFTREG_H_

#include "main.h"
#include "stm32f4xx_hal.h"


void ShiftRegs_init(void);
void ShiftRegs(void);
void blue_led(uint8_t);
void red_led(uint8_t);
void yellow_led(uint8_t);
void green_led(uint8_t);
void white_led(uint8_t);
void bmp_cs(uint8_t);
void lis3mdl_cs(uint8_t);
void lsm6ds_cs(uint8_t);
void RF_ShiftRegs(void);
void NRF_cs(uint8_t);
void NRF_ce(uint8_t);
void rf_led(uint8_t);


	extern SPI_HandleTypeDef hspi2;  //используемый spi для управления сдвиговыми регистрами

#endif /* INC_SHIFTREG_H_ */
