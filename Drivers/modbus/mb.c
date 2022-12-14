/*
 * mb.c
 *
 *  Created on: Oct 1, 2022
 *      Author: Peter
 */

#include "mb.h"
#include "main.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "tmc/ic/TMC4671/TMC4671.h"

extern uint32_t Raw_ADC_temp[5];
extern uint32_t HS1temp_raw;
extern int32_t HS1temp;

ModBus_State mb_state;
uint8_t rx_buffer[256] = { 0 };
uint8_t tx_buffer[256] = { 0 };
uint8_t NewFrameRecv = 0;
uint16_t NewFrameSize = 0;
static uint16_t wpos = 0;
static uint16_t rpos = 0;
/* Table of CRC values for high-order byte */
static const uint8_t table_crc_hi[] = { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1,
		0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
		0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };

/* Table of CRC values for low-order byte */
static const uint8_t table_crc_lo[] = { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03,
		0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C,
		0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
		0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE,
		0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17,
		0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30,
		0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35,
		0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
		0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B,
		0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24,
		0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21,
		0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6,
		0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
		0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8,
		0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD,
		0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2,
		0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53,
		0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
		0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59,
		0x58, 0x98, 0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E,
		0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47,
		0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };

uint16_t crc16(uint8_t *buffer, uint16_t buffer_length) {
	uint8_t crc_hi = 0xFF; /* high CRC byte initialized */
	uint8_t crc_lo = 0xFF; /* low CRC byte initialized */
	unsigned int i; /* will index into CRC lookup */

	/* pass through message buffer */
	while (buffer_length--) {
		i = crc_lo ^ *buffer++; /* calculate the CRC  */
		crc_lo = crc_hi ^ table_crc_hi[i];
		crc_hi = table_crc_lo[i];
	}

	return (crc_hi << 8 | crc_lo);
}

bool IsValidCRC() {
	uint16_t calcCRC = crc16(rx_buffer, NewFrameSize - 2);

	uint16_t recvCRC = (rx_buffer[NewFrameSize - 1] << 8)
			+ rx_buffer[NewFrameSize - 2];

	return calcCRC == recvCRC;
}

int32_t getChipInfo() {
	int32_t chipInfo = 0;

	tmc4671_writeInt(0, TMC4671_CHIPINFO_ADDR, 0);
	chipInfo = tmc4671_readInt(0, TMC4671_CHIPINFO_DATA);

	return chipInfo;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

//	HAL_UART_Transmit(&huart5, rx_buffer, Size, 100);
//	HAL_UARTEx_ReceiveToIdle_IT(&huart5, rx_buffer, sizeof(rx_buffer));
	NewFrameSize = Size;
	mb_state = NewFrame;
}

void ModBus_ParseNewFrame();
void ModBus_CustomFunctions();


void ModBus_Init(void)
{
	HAL_UARTEx_ReceiveToIdle_IT(&huart5, rx_buffer, sizeof(rx_buffer));
}
void ModBus_Perform(void) {

	switch (mb_state) {

	case NewFrame: {
		ModBus_ParseNewFrame();
	}
		break;

	case Idle:
		break;

	default:
		break;

	}
}

static inline void frame_read_ui32(uint32_t *value) {
	*value = *((uint32_t*) (rx_buffer + rpos));
	rpos += 4;
}

static inline void frame_write_ui32(const uint32_t value) {
	*((uint32_t*) (tx_buffer + wpos)) = value;
	wpos += 4;
}

static inline void frame_read_ui16(uint16_t *value) {
	*value = *((uint16_t*) (rx_buffer + rpos));
	rpos += 2;
}

static inline void frame_write_ui16(const uint16_t value) {
	*((uint16_t*) (tx_buffer + wpos)) = value;
	wpos += 2;
}

static inline void frame_read_ui8(uint8_t *value) {
	*value = *((uint8_t*) (rx_buffer + rpos));
	rpos += 1;
}

static inline void frame_write_ui8(const uint8_t value) {
	*((uint8_t*) (tx_buffer + wpos)) = value;
	wpos += 1;
}

void ModBus_CustomFunction();

void ModBus_ParseNewFrame() {

	wpos = 0;
	rpos = 0;
	uint8_t slaveID;
	ModBus_Funtions functionID;
	uint16_t frameLenPos = 0;

	// if minimum min_data_len>=1B (min_frame_len>=5B) and CRC is valid
	if (NewFrameSize>=5 && IsValidCRC())
	{
		frame_read_ui8(&slaveID);
		frame_read_ui8(&functionID);

		frame_write_ui8(slaveID);
		frame_write_ui8(functionID);

		frameLenPos = wpos++;
	}
	else
	{
		functionID = Error;
	}

	switch (functionID) {
	case Custom_Function: {
		ModBus_CustomFunction();
	}
		break;
	case Error: {
		//ModBus_CustomFunction();
	}
		break;

	default:
		break;
	}

	tx_buffer[frameLenPos] = wpos - frameLenPos - 1;
	frame_write_ui16(crc16(tx_buffer, wpos));

	NewFrameSize = 0;
	HAL_UART_Transmit(&huart5, tx_buffer, wpos, 100);
	HAL_UARTEx_ReceiveToIdle_IT(&huart5, rx_buffer, sizeof(rx_buffer));
	mb_state = Idle;
}

void ModBus_CustomFunction() {

	ModBus_Custom_Functions customFunctionID;

	frame_read_ui8(&customFunctionID);

	switch (customFunctionID) {

	case readInt: {
		uint8_t motor_id;
		uint8_t read_address;
		int32_t result;

		frame_read_ui8(&motor_id);
		frame_read_ui8(&read_address);

		result = tmc4671_readInt(0, read_address);

		frame_write_ui32(result);

	}
		break;

	case writeInt: {
		uint8_t motor_id;
		uint8_t write_address;
		uint32_t write_value;

		frame_read_ui8(&motor_id);
		frame_read_ui8(&write_address);
		frame_read_ui32(&write_value);

		tmc4671_writeInt(motor_id, write_address, write_value);
		frame_write_ui8(0);
	}
	break;
	case enablePWM: {
		uint8_t motor_id;

		frame_read_ui8(&motor_id);
		tmc4671_writeInt(0, TMC4671_PWM_SV_CHOP, 0x00000007);
		frame_write_ui8(0);
	}
	break;
	case disablePWM: {
		uint8_t motor_id;

		frame_read_ui8(&motor_id);
		tmc4671_writeInt(0, TMC4671_PWM_SV_CHOP, 0x00000000);
		frame_write_ui8(0);
	}
	break;
	case setTargetTorque: {
			uint8_t motor_id;
			uint32_t write_value;

			frame_read_ui8(&motor_id);
			frame_read_ui32(&write_value);

			tmc4671_setTargetTorque_mA(motor_id, 256, write_value);
			frame_write_ui8(0);
		}
	break;
	case getTargetTorque: {
		uint8_t motor_id;
		int32_t result;

		frame_read_ui8(&motor_id);

		result = tmc4671_getTargetTorque_mA(motor_id, 256);
		frame_write_ui32(result);
	}
	break;
	case getActualTorque: {
		uint8_t motor_id;
		uint32_t result;

		frame_read_ui8(&motor_id);

		result = tmc4671_getActualTorque_mA(motor_id, 256);
		frame_write_ui32(result);
	}
	break;
	case setTargetVelocity: {
		uint8_t motor_id;
		uint32_t write_value;

		frame_read_ui8(&motor_id);
		frame_read_ui32(&write_value);

		tmc4671_setTargetVelocity(motor_id, write_value);
		frame_write_ui8(0);
	}
	break;
	case getTargetVelocity: {
		uint8_t motor_id;
		int32_t result;

		frame_read_ui8(&motor_id);

		result = tmc4671_getTargetVelocity(motor_id);
		frame_write_ui32(result);
	}
	break;
	case getActualVelocity: {
		uint8_t motor_id;
		uint32_t result;

		frame_read_ui8(&motor_id);

		result = tmc4671_getActualVelocity(motor_id);
		frame_write_ui32(result);
	}
	break;
	case setAbsoluteTargetPosition: {
		uint8_t motor_id;
		uint32_t write_value;

		frame_read_ui8(&motor_id);
		frame_read_ui32(&write_value);

		tmc4671_setAbsoluteTargetPosition(motor_id, write_value);
		frame_write_ui8(0);
	}
	break;
	case incrementTargetPosition: {
		uint8_t motor_id;
		uint32_t write_value;

		frame_read_ui8(&motor_id);
		frame_read_ui32(&write_value);

		tmc4671_setAbsoluteTargetPosition(motor_id, tmc4671_getTargetPosition(motor_id) + write_value);
		//tmc4671_setRelativeTargetPosition(motor_id, write_value); // broken?
		frame_write_ui8(0);
	}
	break;
	case decrementTargetPosition: {
		uint8_t motor_id;
		uint32_t write_value;

		frame_read_ui8(&motor_id);
		frame_read_ui32(&write_value);

		tmc4671_setAbsoluteTargetPosition(motor_id, tmc4671_getTargetPosition(motor_id) - write_value);
		frame_write_ui8(0);
	}
	break;
	case getActualPosition: {
		uint8_t motor_id;
		uint32_t result;

		frame_read_ui8(&motor_id);

		result = tmc4671_getActualPosition(motor_id);
		frame_write_ui32(result);
	}
	break;
	case getTargetPosition: {
		uint8_t motor_id;
		uint32_t result;

		frame_read_ui8(&motor_id);

		result = tmc4671_getTargetPosition(motor_id);
		frame_write_ui32(result);
	}
	break;
	case setDebugLedState: {
		uint8_t state;

		frame_read_ui8(&state);
		HAL_GPIO_WritePin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin, state);
		frame_write_ui8(0);
	}
	break;
	case getDebugLedState: {
		uint8_t result;
		result = HAL_GPIO_ReadPin(DEBUG_LED_GPIO_Port, DEBUG_LED_Pin);
		frame_write_ui8(result);
	}
		break;
	case getRawADC2Measurements: {
		for (uint8_t i=0; i<5; i++)
			frame_write_ui32(Raw_ADC_temp[i]);
	}
		break;
	case getHSTemp: {
		uint8_t sensor_id;
		uint32_t result;

		frame_read_ui8(&sensor_id);
		//result = HS1temp_raw;
		result = HS1temp;
		frame_write_ui32(result);
	}
		break;
	case performEncoderInitUD: {
		uint8_t motor_id;
		uint32_t write_value;

		frame_read_ui8(&motor_id);
		frame_read_ui32(&write_value);

		//openloop_test_drive(write_value); //disabled temporarily
		ABN_encoder_test_drive(write_value);
		frame_write_ui8(0);
	}
		break;
	default:
		break;
	}

}

