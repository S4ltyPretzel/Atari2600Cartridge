/*
 * sd_spi.h
 *
 *  Created on: 26.11.2017
 *      Author: jaras
 */

#ifndef SD_SPI_H_
#define SD_SPI_H_

#include "stm32f4xx_hal.h"

#define SDSPI_CSPORT GPIOA
#define SDSPI_CSPIN GPIO_PIN_15

uint8_t SDSPI_Init(SPI_HandleTypeDef *phandle);
uint8_t SDSPI_ReadInfo(SPI_HandleTypeDef *phandle, uint16_t *sector, uint32_t *capacity);
uint8_t SDSPI_ReadBlock(SPI_HandleTypeDef *phandle, uint32_t lba, uint8_t *buf, uint16_t size);
uint8_t SDSPI_WriteBlock(SPI_HandleTypeDef *phandle, uint32_t lba, uint8_t *buf, uint16_t size);


#endif /* SD_SPI_H_ */
