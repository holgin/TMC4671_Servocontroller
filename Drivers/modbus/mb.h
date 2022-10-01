/*
 * mb.h
 *
 *  Created on: Oct 1, 2022
 *      Author: Peter
 */

#ifndef MODBUS_MB_H_
#define MODBUS_MB_H_

#include <stdint.h>

void ModBus_Perform(void);

typedef enum {
	Idle = 0,
	NewFrame = 1
} ModBus_State;

typedef enum {
	Read_Coils = 1,
	Read_Discrete_Inputs = 2,
	Read_Holding_Registers = 3,
	Read_Input_Registers = 4,
	Write_Single_Coil = 5,
	Write_Single_Register = 6,
	Write_Multiple_Registers = 16,
	Custom_Function = 0x64,
	Error = 0xFF
} ModBus_Funtions;


typedef enum {
	readInt = 1,
	writeInt = 2,
	setTargetVelocity = 3,
	getTargetVelocity = 4


} ModBus_Custom_Functions;

typedef union {
	uint32_t ui32;
	uint8_t ui8[4];
} ui32_to_ui8;

extern ModBus_State mb_state;
extern uint8_t rx_buffer[256];
extern uint8_t tx_buffer[256];
extern uint8_t NewFrameRecv;
extern uint16_t NewFrameSize;

#endif /* MODBUS_MB_H_ */
