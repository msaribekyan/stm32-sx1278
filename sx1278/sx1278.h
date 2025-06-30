/*
 * sx1278.h
 *
 * Created on: Jun 13, 2025
 * Author: Mher Saribekyan
 */
#include <stdlib.h>
#include <stm32f1xx_hal.h>
#ifndef INC_SX1278_H_
#define INC_SX1278_H_

/*
 * Registers
 */
#define RegFifo 0x00
#define RegOpMode 0x01
#define RegBitrateMsb 0x02
#define RegBitrateLsb 0x03
#define RegPaConfig 0x09
#define RegPaRamp 0x0A
#define RegOcp 0x0B
#define RegLna 0x0C
#define RegRxConfig 0x0D
#define RegPreambleDetect 0x1f
#define RegOsc 0x24
#define RegPreambleMsb 0x25
#define RegPreambleLsb 0x26
#define RegSyncConfig 0x27
#define RegSyncValue1 0x28
#define RegSyncValue2 0x29
#define RegPacketConfig1 0x30
#define RegPacketConfig2 0x31
#define RegPayloadLength 0x32
#define RegNodeAdrs 0x33
#define RegBroadcastAdrs 0x34
#define RegFifoThresh 0x35
#define RegIrqFlags1 0x3e
#define RegIrqFlags2 0x3f

/**
 * @brief Represents SX1278 module hardware connections
 */
typedef struct {
	SPI_HandleTypeDef* hspi;
	GPIO_TypeDef* nss_port;
	uint16_t nss_pin;
} SX1278;

SX1278* SX1278_setup(SPI_HandleTypeDef* hspi, GPIO_TypeDef* nss_port, uint16_t nss_pin);
void SX1278_destroy(SX1278* sx1278);
void SX1278_setup_FSK(SX1278* sx1278);
void SX1278_transmit_FSK(SX1278* sx1278, uint8_t* payload, uint16_t size);

#endif /* INC_SX1278_H_ */
