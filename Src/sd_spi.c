/*
 * sd_spi.c
 *
 *  Created on: 26.11.2017
 *      Author: jaras
 */

#include "sd_spi.h"
#include <string.h>

uint8_t SDSPI_SendCMD(SPI_HandleTypeDef *phandle, uint8_t cmd, uint32_t arg, uint8_t crc) {
	uint8_t buf[6];
	buf[0] = cmd | 0x40;
	buf[1] = (arg >> 24) & 0xff;
	buf[2] = (arg >> 16) & 0xff;
	buf[3] = (arg >> 8) & 0xff;
	buf[4] = arg & 0xff;
	buf[5] = crc;

	if(HAL_SPI_Transmit(phandle, buf, 6, 1000) != HAL_OK) {
		return 1;
	}

	return 0;
}

uint8_t SDSPI_Response(SPI_HandleTypeDef *phandle, uint8_t *buf, uint16_t size) {
	uint8_t tx = 0xff;
	uint8_t rx = 0xff;
	uint8_t i = 0;

	while(rx == 0xff) {
		if(HAL_SPI_TransmitReceive(phandle, &tx, &rx, 1, 1000) != HAL_OK) {
			return 1;
		}
		i++;
		if(i > 8) {
			return 2;
		}
	}

	*buf = rx;

	for(uint16_t k = 1; k < size; k++) {
		if(HAL_SPI_TransmitReceive(phandle, &tx, &rx, 1, 1000) != HAL_OK) {
			return 1;
		}
		*(buf + k) = rx;
	}

	return 0;
}

uint8_t SDSPI_CMD(SPI_HandleTypeDef *phandle, uint8_t cmd, uint32_t arg, uint8_t crc,
					uint8_t *response, uint8_t size) {

	HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_RESET);

	uint8_t res = SDSPI_SendCMD(phandle, cmd, arg, crc);
	if(res > 0) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 1;
	}

	res = SDSPI_Response(phandle, response, size);
	if(res > 0) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 2;
	}

	HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
	uint8_t tx = 0xff;
	if(HAL_SPI_Transmit(phandle, &tx, 1, 1000) != HAL_OK) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 3;
	}

	return 0;
}

uint8_t SDSPI_ACMD(SPI_HandleTypeDef *phandle, uint8_t cmd, uint32_t arg, uint8_t crc,
					uint8_t *response, uint8_t size) {
	uint8_t rx = 0;

	uint8_t res = SDSPI_CMD(phandle, 55, 0, 0x65, &rx, 1);
	if(res > 0) {
		return 1;
	}
	if((rx & 0xf4) > 0) {
		return 2;
	}

	res = SDSPI_CMD(phandle, cmd, arg, crc, response, size);
	if(res > 0) {
		return 3;
	}

	return 0;
}

uint8_t SDSPI_Init(SPI_HandleTypeDef *phandle) {
	HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
	HAL_Delay(10);
	uint8_t buf[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
	if(HAL_SPI_Transmit(phandle, buf, 10, 1000) != HAL_OK) {
		return 1; //spi error
	}

	uint8_t res = SDSPI_CMD(phandle, 0, 0, 0x95, buf, 1);
	if(res > 0) {
		return 1; //command error
	}
	if(buf[0] != 1) {
		return 2; //not initialized
	}

	uint8_t type = 0;
	uint8_t block = 0;

	res = SDSPI_CMD(phandle, 8, 0x01aa, 0x87, buf, 5);
	if(res > 0) {
		type = 1;
	}
	if(buf[0] != 1) {
		type = 1;
	}
	if((buf[3] & 0x0f) != 1 || buf[4] != 0xaa) {
		return 3; //initialization error
	}

	uint8_t stat = 0;
	uint32_t tickstart = 0;

	if(type == 0) {
		stat = 1;
		tickstart = HAL_GetTick();
		while(stat > 0) {
			if((HAL_GetTick()-tickstart) >= 1000) {
				HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
				return 4; //timeout
			}

			res = SDSPI_ACMD(phandle, 41, 0x40000000, 0x77, &stat, 1);
			if(res > 0) {
				return 5; //not supported
			}
		}

		res = SDSPI_CMD(phandle, 58, 0, 0x75, buf, 5);
		if(res > 0) {
			return 6; //not supported
		}
		if(buf[0] > 0) {
			return 7;
		}
		if((buf[1] & 0x04) > 0) {
			block = 1;
		}

	}
	if(type == 1) {
		stat = 1;
		tickstart = HAL_GetTick();
		while(stat > 0) {
			if((HAL_GetTick()-tickstart) >= 1000) {
				HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
				stat = 0;
				type = 2;
			}

			res = SDSPI_ACMD(phandle, 41, 0, 0xff, &stat, 1);
			if(res > 0) {
				stat = 0;
				type = 2;
			}
		}
	}
	if(type == 2) {
		stat = 1;
		tickstart = HAL_GetTick();
		while(stat > 0) {
			if((HAL_GetTick()-tickstart) >= 1000) {
				HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
				return 8; //timeout
			}

			res = SDSPI_CMD(phandle, 1, 0, 0xff, &stat, 1);
			if(res > 0) {
				return 9; //error
			}
		}
	}
	if(block == 0) {
		res = SDSPI_CMD(phandle, 16, 512, 0xff, buf, 1);
		if(res > 0) {
			return 10; //not supported
		}
		if(buf[0] > 0) {
			return 11; //error
		}
	}
	return 0;
}

uint8_t SDSPI_ReadCommand(SPI_HandleTypeDef *phandle, uint8_t cmd, uint32_t arg, uint8_t *buf, uint16_t size) {
	HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_RESET);

	uint8_t stat = 0;
	uint8_t res = SDSPI_SendCMD(phandle, cmd, arg, 0xff);
	if(res > 0) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 1; //error
	}

	res = SDSPI_Response(phandle, &stat, 1);
	if(res > 0) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 2; //error
	}
	if(stat > 0) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 3; //error result
	}

	//wait for data token
	uint32_t tickstart = HAL_GetTick();
	uint8_t tx = 0xff;
	stat = 0xff;
	while(stat == 0xff) {
		if((HAL_GetTick()-tickstart) >= 1000) {
			HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
			return 4; //timeout
		}
		if(HAL_SPI_TransmitReceive(phandle, &tx, &stat, 1, 1000) != HAL_OK) {
			HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
			return 5;
		}
	}

	if(stat != 0xfe) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 6; //error token
	}

	//read sector
	for(uint16_t i = 0; i < size; i++) {
		if(HAL_SPI_TransmitReceive(phandle, &tx, buf +i, 1, 1000) != HAL_OK) {
			HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
			return 7;
		}
	}

	//read 2 byte crc
	for(uint16_t i = 0; i < 2; i++) {
		if(HAL_SPI_TransmitReceive(phandle, &tx, &stat, 1, 1000) != HAL_OK) {
			HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
			return 7;
		}
	}

	//end
	HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
	if(HAL_SPI_Transmit(phandle, &tx, 1, 1000) != HAL_OK) {
		return 9;
	}

	return 0;
}

uint8_t SDSPI_WriteBlock(SPI_HandleTypeDef *phandle, uint32_t lba, uint8_t *buf, uint16_t size) {
	HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_RESET);

	uint8_t stat = 0;
	uint8_t res = SDSPI_SendCMD(phandle, 24, lba, 0xff);
	if(res > 0) {
		return 1; //error
	}

	res = SDSPI_Response(phandle, &stat, 1);
	if(res > 0) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 2; //error
	}
	if(stat > 0) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 3; //error result
	}

	//8 tics of CLK
	stat = 0xff;
	if(HAL_SPI_Transmit(phandle, &stat, 1, 1000) != HAL_OK) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 4;
	}

	//data packet
	//data token
	stat = 0xfe;
	if(HAL_SPI_Transmit(phandle, &stat, 1, 1000) != HAL_OK) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 5;
	}
	//data block
	if(HAL_SPI_Transmit(phandle, buf, size, 1000) != HAL_OK) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 6;
	}
	//crc
	uint8_t crc[] = {0xff, 0xff};
	if(HAL_SPI_Transmit(phandle, crc, 2, 1000) != HAL_OK) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 7;
	}

	//data response
	res = SDSPI_Response(phandle, &stat, 1);
	if(res > 0) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 8; //error
	}
	if((stat & 0x1f) != 0x05) {
		HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
		return 9; //error result
	}

	//wait
	uint32_t tickstart = HAL_GetTick();
	uint8_t tx = 0xff;
	stat = 0;
	while(stat == 0) {
		if((HAL_GetTick()-tickstart) >= 1000) {
			HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
			return 10; //timeout
		}
		if(HAL_SPI_TransmitReceive(phandle, &tx, &stat, 1, 1000) != HAL_OK) {
			HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
			return 11;
		}
	}

	HAL_GPIO_WritePin(SDSPI_CSPORT, SDSPI_CSPIN, GPIO_PIN_SET);
	return 0;
}

uint8_t SDSPI_ReadBlock(SPI_HandleTypeDef *phandle, uint32_t lba, uint8_t *buf, uint16_t size) {
	return SDSPI_ReadCommand(phandle, 17, lba, buf, size);
}

uint8_t SDSPI_ReadInfo(SPI_HandleTypeDef *phandle, uint16_t *sector, uint32_t *capacity) {
	uint8_t buf[16];
	uint8_t res = SDSPI_ReadCommand(phandle, 9, 0, buf, 16);
	if(res > 0) {
		return res;
	}

	uint8_t version = buf[0] >> 6;
	uint32_t c_size = 0;
	uint8_t c_size_mult = 0;
	uint8_t read_bl_len = 0;
	if(version == 0) {
		read_bl_len = buf[5] & 0x0f;
		c_size = (((buf[6] & 3) << 16) | (buf[7] << 8) | buf[8]) >> 6;
		c_size_mult = (((buf[9] & 3) << 8) | buf[10]) >> 7;
		*sector = 1 << read_bl_len;
		*capacity = (c_size +1) * (1 << (c_size_mult +2)) * (*sector);
	}
	else {
		c_size = ((buf[7] & 0x3f) << 16) | (buf[8] << 8) | buf[9];
		*sector = 512;
		*capacity = (c_size +1) * 512 * 1024;
	}
	return 0;
}

