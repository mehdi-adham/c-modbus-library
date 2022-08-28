/**
 * @file modbus_rtu.c
 * @author Mehdi Adham (mehdi.adham@yahoo.com)
 * @brief This library implements the Modbus RTU protocol.
 * @version 0.1
 * @date 2022-07-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "modbus.h"
#include "modbus_rtu.h"
#include "CRC.h"
#include <stdint.h>

unsigned char SLAVE_ADDRESS;

extern unsigned char auchCRCHi[];
extern unsigned char auchCRCLo[];

unsigned char frame_buffer[256];
unsigned char response_buffer[256];

/**
 * @brief
 * NOTE : This function should not be modified, when the callback is needed,
 the uart_receive Handler could be implemented in the user file
 * @return Modbus Status
 */
__attribute__((weak))	 ModbusStatus_t modbus_uart_receive_Handler(uint8_t *Data) {
	return 0;
}

/**
 * @brief
 * NOTE : This function should not be modified, when the callback is needed,
 the uart transmit Handler could be implemented in the user file
 * @return None
 */
__attribute__((weak)) 	void modbus_uart_transmit_Handler(uint8_t *Data,
		uint16_t length) {

}

/**
 * @brief
 *
 * @param slave_ID
 */
void set_slave_ID(unsigned char slave_ID){
	SLAVE_ADDRESS = slave_ID;
}

/**
 * @brief This function receives the frame related to this device from serial, 
 * processes it and applies commands (reading/writing to registers or coils) and 
 * prepares the response or exception response for the master and sends it via serial.
 * 
 * @note Template Frame For Test:
 * {0x11, 0x01, 0x00, 0x13, 0x00, 0x25, 0x0E, 0x84}
 * {0x11, 0x02, 0x00, 0xC4, 0x00, 0x16, 0xBA, 0xA9}
 * {0x11, 0x03, 0x00, 0x6B, 0x00, 0x03 ,0x76 ,0x87}
 * {0x11, 0x04, 0x00, 0x08, 0x00, 0x01 ,0xB2 ,0x98}
 * {0x11, 0x05, 0x00, 0xAC, 0xFF, 0x00 ,0x4E ,0x8B}
 * {0x11, 0x06, 0x00, 0x01, 0x00, 0x03 ,0x9A ,0x9B}
 * {0x11, 0x07, 0x4C, 0x22}
 * {0x11, 0x0B, 0x4C, 0x27}
 * {0x11, 0x0C, 0x0D, 0xE5}
 * {0x11, 0x0F, 0x00, 0x13, 0x00, 0x0A ,0x02 ,0xCD, 0x01, 0xBF, 0x0B}
 * {0x11, 0x10, 0x00, 0x01, 0x00, 0x02 ,0x04 ,0x00, 0x0A, 0x01, 0x02, 0xC6, 0xF0}
 * {0x11, 0x11, 0xCD, 0xEC}
 * {0x11, 0x14, 0x0E, 0x06, 0x00, 0x04 ,0x00 ,0x01, 0x00, 0x02, 0x06, 0x00, 0x03, 0x00, 0x09, 0x00, 0x02, 0xF9, 0x38}
 * {0x11, 0x15, 0x0D, 0x06, 0x00, 0x04 ,0x00 ,0x07, 0x00, 0x03, 0x06, 0xAF, 0x04, 0xBE, 0x10, 0x0D, 0xDB, 0xC7}
 * {0x11, 0x16, 0x00, 0x04, 0x00, 0xF2 ,0x00 ,0x25, 0x66, 0xE2}
 * {0x11, 0x17, 0x00, 0x04, 0x00, 0x06 ,0x00 ,0x0F, 0x00, 0x03, 0x06, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x1C, 0x56}
 * {0x11, 0x18, 0x04, 0xDE, 0x07, 0x87}
 *
 * @param mbus_frame_buffer Saves the frame in the user buffer
 * @param Tick Get a pointer to tick value in millisecond. For 
 * setting the timeout to exit the function if the master device 
 * does not send the frame related to this device
 * @param Mode 1. Normal mode (by responding to the master and applying commands) 
 * 2. only listening mode (without responding to the master and without applying commands)
 */
ModbusStatus_t MODBUS_RTU_MONITOR(unsigned char *mbus_frame_buffer,
		int monitor_fun_timeout, volatile uint32_t *Tick, ModbusMonitorMode_t Mode) {
	unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
	unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */
	unsigned char uIndex; /* will index into CRC lookup table */
	unsigned short calculate_crc;
	unsigned char rec_byte;
	ModbusStatus_t res;

	uint32_t tickstart_for_monitor_timeout;
	uint32_t currenttick_for_monitor_timeout;

	/* The starting address of the buffer */
	unsigned char *starting_address_of_buffer = mbus_frame_buffer;

	unsigned char (*receive_uart_fun)() = modbus_uart_receive_Handler;
	void (*transmit_uart_fun)(uint8_t *Data,
			uint16_t length) = modbus_uart_transmit_Handler;
	uint16_t counter = 0;

	/* Initial tick start for monitor timeout management */
	tickstart_for_monitor_timeout = *Tick;
	while (1) {
		/* 0. Check for monitor function timeout */
		currenttick_for_monitor_timeout = *Tick;
		if (currenttick_for_monitor_timeout - tickstart_for_monitor_timeout
				> monitor_fun_timeout)
			return MODBUS_MONITOR_TIMEOUT;

		/* 1. frame timeout management */
		do {
			res = (*receive_uart_fun)(&rec_byte);
		} while (res == MODBUS_OK);

		/* 2. Ready for receive of first Byte (Address Field) */
		do {
			res = (*receive_uart_fun)(&rec_byte);
			/* Check for monitor function timeout */
			currenttick_for_monitor_timeout = *Tick;
			if (currenttick_for_monitor_timeout - tickstart_for_monitor_timeout
					> monitor_fun_timeout)
				return MODBUS_MONITOR_TIMEOUT;
		} while (res != MODBUS_OK);


		/* 3. if out of range allowed address OR Address field not match with slave ID AND not broadcast */
		if (rec_byte > MAX_SLAVE_ADDRESS || (rec_byte != SLAVE_ADDRESS && rec_byte != Broadcast))
			/* return to 0. */
			continue;

		uchCRCHi = 0xFF;
		uchCRCLo = 0xFF;

		/* 4. Be assigned to the buffer and Calculate the CRC of this field */
		frame_buffer[counter++] = *mbus_frame_buffer++ = rec_byte;
		uIndex = uchCRCHi ^ rec_byte;
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];

		/* 5. Get second field (Function Field) */
		res = (*receive_uart_fun)(&rec_byte);
		if (res != MODBUS_OK)
			return res;

		/* 6. Be assigned to the fun/buffer value and Calculate the CRC of this field */
		unsigned char fun = frame_buffer[counter++] = *mbus_frame_buffer++ =
				rec_byte;
		uIndex = uchCRCHi ^ rec_byte;
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];


		/* 7. */
		/* Extracting length (register or coil ...) from frame data */
		int len = 0;
		if (fun == Read_Coil_Status || fun == Read_Input_Status
				|| fun == Read_Holding_Registers || fun == Read_Input_Registers
				|| fun == Force_Single_Coil || fun == Preset_Single_Register) {
			len = 4;
		} else if (fun == Read_Exception_Status
				|| fun == Fetch_Comm_Event_Counter
				|| fun == Fetch_Comm_Event_Log || fun == Report_Slave_ID) {
			len = 0;
		} else if (fun == Force_Multiple_Coils
				|| fun == Preset_Multiple_Registers) {
			int l = 5;
			while (l--) {
				res = (*receive_uart_fun)(&rec_byte);
				if (res != MODBUS_OK)
					return res;
				frame_buffer[counter++] = *mbus_frame_buffer++ = rec_byte;
				/* Calculate the CRC of this field */
				uIndex = uchCRCHi ^ rec_byte;
				uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
				uchCRCLo = auchCRCLo[uIndex];
			}

			len = rec_byte;
		} else if (fun == Read_General_Reference
				|| fun == Write_General_Reference) {
			res = (*receive_uart_fun)(&rec_byte);
			if (res != MODBUS_OK)
				return res;
			frame_buffer[counter++] = *mbus_frame_buffer++ = rec_byte;
			/* Calculate the CRC of this field */
			uIndex = uchCRCHi ^ rec_byte;
			uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
			uchCRCLo = auchCRCLo[uIndex];

			len = rec_byte;
		} else if (fun == Mask_Write_4X_Register) {
			len = 6;
		} else if (fun == Read_Write_4X_Registers) {
			int l = 9;
			while (l--) {
				res = (*receive_uart_fun)(&rec_byte);
				if (res != MODBUS_OK)
					return res;
				frame_buffer[counter++] = *mbus_frame_buffer++ = rec_byte;
				/* Calculate the CRC of this field */
				uIndex = uchCRCHi ^ rec_byte;
				uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
				uchCRCLo = auchCRCLo[uIndex];
			}

			len = rec_byte;
		} else if (fun == Read_FIFO_Queue) {
			len = 2;
		}


		/* 8. Remain byte */
		while (len) {
			/* check overflow */
			if(counter > MAX_BUFFER){
				/* clear buffer */
				while (starting_address_of_buffer < mbus_frame_buffer) {
					frame_buffer[counter--] = *mbus_frame_buffer-- = 0x00;
				}
				/* return to 0. */
				continue;
			}

			/* 8.1 */
			res = (*receive_uart_fun)(&rec_byte);
			if (res != MODBUS_OK)
				return res;
			/* 8.2 */
			frame_buffer[counter++] = *mbus_frame_buffer++ = rec_byte;
			/* Calculate the CRC of this field */
			uIndex = uchCRCHi ^ rec_byte;
			uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
			uchCRCLo = auchCRCLo[uIndex];

			/* 8.3 */
			len--;
		}

		/* 9. Get CRC */
		res = (*receive_uart_fun)(&rec_byte);
		if (res != MODBUS_OK)
			return res;
		frame_buffer[counter++] = *mbus_frame_buffer++ = rec_byte;

		res = (*receive_uart_fun)(&rec_byte);
		if (res != MODBUS_OK)
			return res;
		frame_buffer[counter++] = *mbus_frame_buffer++ = rec_byte;

		/* 10. Check CRC calculated, that is equal with CRC frame */
		calculate_crc = (uchCRCHi << 8 | uchCRCLo);
		unsigned short frameCRC = (unsigned short) ((*(mbus_frame_buffer - 2)
				<< 8) | *(mbus_frame_buffer - 1));

		if (calculate_crc != frameCRC) {
			/* clear buffer */
			while (starting_address_of_buffer < mbus_frame_buffer) {
				frame_buffer[counter--] = *mbus_frame_buffer-- = 0x00;
			}
			/* return to 0. */
			continue;
		}

		break;
	}/*< End while() for frame time out */

	if(Mode == Normal){
		/* 10. MODBUS PROCESS for Constructed Response Frame */
		uint16_t len = MODBUS_FARME_PROCESS(frame_buffer, response_buffer);

		/* 11. Transmit frame */
		(*transmit_uart_fun)(response_buffer, len);
	}

	return MODBUS_OK;
}
