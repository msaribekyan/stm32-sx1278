/*
 * sx1278.h
 *
 * Created on: Jun 13, 2025
 * Author: Mher Saribekyan
 */
#include <sx1278.h>

void SS_Select(SX1278* sx1278){
	HAL_GPIO_WritePin(sx1278->nss_port, sx1278->nss_pin, GPIO_PIN_RESET);
}

void SS_UnSelect(SX1278* sx1278){
	HAL_GPIO_WritePin(sx1278->nss_port, sx1278->nss_pin, GPIO_PIN_SET);
}

uint8_t ReadReg_Single(SX1278* sx1278, uint8_t Reg){
	uint8_t data=0;
	SS_Select(sx1278);
	HAL_Delay(1);
	HAL_SPI_Transmit(sx1278->hspi, &Reg, 1, 100);
	HAL_SPI_Receive(sx1278->hspi, &data, 1, 100);
	SS_UnSelect(sx1278);
	return data;
}

void WriteReg_Single(SX1278* sx1278, uint8_t Reg, uint8_t Data){
	uint8_t buf[2];
	buf[0] = Reg|(1<<7);
	buf[1] = Data;
	SS_Select(sx1278);
	HAL_Delay(1);
	HAL_SPI_Transmit(sx1278->hspi, buf, 2, 100);
	SS_UnSelect(sx1278);
}

/**
 * @brief Creates a new SX1278 instance.
 *
 * @param hspi SX1278 spi handler
 * @param nss_port SX1278 NSS GPIO port
 * @param nss_pin SX1278 NSS GPIO pin
 *
 * @return SX1278 instance
 *
 */
SX1278* SX1278_setup(SPI_HandleTypeDef* hspi, GPIO_TypeDef* nss_port, uint16_t nss_pin){
	SX1278* sx1278 = (SX1278*) malloc(sizeof(SX1278));

	sx1278->hspi = hspi;
	sx1278->nss_port = nss_port;
	sx1278->nss_pin = nss_pin;

	return sx1278;
}

/**
 * @brief Destroys an SX1278 instance.
 *
 * @param sx1278 SX1278 instance pointer
 */
void SX1278_destroy(SX1278* sx1278){
	free(sx1278);
}

/**
 * @brief Sets up SX1278 in FSK mode
 *
 * @param sx1278 SX1278 instance pointer
 */
void SX1278_setup_FSK(SX1278* sx1278){
	WriteReg_Single(sx1278, RegOpMode, 0x00); // FSK Sleep mode

	WriteReg_Single(sx1278, RegPacketConfig1, 0x00); // Fixed length, no CRC, no Address
	WriteReg_Single(sx1278, RegFifoThresh, 0x80); // condition to start packet tx
	WriteReg_Single(sx1278, RegSyncConfig, 0x00); // No sync word
	WriteReg_Single(sx1278, RegBitrateMsb, 0x00); // BitRate
	WriteReg_Single(sx1278, RegBitrateLsb, 0x6B); // BitRate 300kbps
	WriteReg_Single(sx1278, RegPreambleDetect, 0x8A); // 1 byte preamble
	WriteReg_Single(sx1278, RegPreambleLsb, 0xAB); // Preamble

	WriteReg_Single(sx1278, RegOpMode, 0x01); // FSK Standby mode
}

/**
 * @brief Sends payload in FSK mode
 *
 * @param sx1278 SX1278 instance pointer
 * @param payload uint8_t payload pointer
 * @param size uint16_t payload size
 */
void SX1278_transmit_FSK(SX1278* sx1278, uint8_t* payload, uint16_t size){
	WriteReg_Single(sx1278, RegOpMode, 0x01); // FSK Standby mode

	for(uint16_t i = 0;i < size;i++){
		WriteReg_Single(sx1278, RegFifo, *(payload+i));
	}

	WriteReg_Single(sx1278, RegOpMode, 0x03); // FSK Tx mode
	uint8_t st0 = ReadReg_Single(sx1278, RegIrqFlags2);
	while(((st0 >> 3) & 0x01) == 0){
		st0 = ReadReg_Single(sx1278, RegIrqFlags2);
	}
	WriteReg_Single(sx1278, RegOpMode, 0x01); // FSK Standby mode
}

