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
#include "modbus_handler.h"

//#define speed_test

unsigned char frame_buffer[MAX_BUFFER];
unsigned char response_buffer[MAX_BUFFER];

/**
 * @brief Table of CRC values for high–order byte
 *
 */
static const unsigned char auchCRCHi[] = { 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
		0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80,
		0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 };

/**
 * @brief Table of CRC values for low–order byte
 *
 */
static const unsigned char auchCRCLo[] = { 0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2,
		0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD,
		0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
		0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F,
		0xDD, 0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6,
		0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1,
		0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
		0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB,
		0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA,
		0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5,
		0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
		0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67,
		0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE,
		0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79,
		0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C,
		0xB4, 0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73,
		0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92,
		0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D,
		0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
		0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F,
		0x8D, 0x4D, 0x4C, 0x8C, 0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86,
		0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40 };

/**
 * @brief This function calculate CRC for RTU Modbus Protocol.
 * @note This function performs the swapping of the high/low CRC bytes internally.
 * The bytes are already swapped in the CRC value that is returned from the function.
 * Therefore the CRC value returned from the function can be directly placed into
 * the message for transmission.
 * @param puchMsg A pointer to the message buffer containing
 * binary data to be used for generating the CRC.
 * @param usDataLen The quantity of bytes in the message buffer.
 * @retval returns the CRC as a type unsigned short.
 */
unsigned short CRC16(const unsigned char *puchMsg, unsigned short usDataLen) {
	unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
	unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */
	unsigned char uIndex; /* will index into CRC lookup table */
	while (usDataLen--) /* pass through message buffer */
	{
		uIndex = uchCRCHi ^ *puchMsg++; /* calculate the CRC */
		uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo);
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

#ifdef speed_test
	uint32_t tickstart_for_tr_time;
	uint32_t currenttick_for_tr_time;
	unsigned int tr_time ;

	tickstart_for_tr_time = *Tick;
#endif

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

	unsigned char SLAVE_ADDRESS = get_slave_ID();

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

#ifdef speed_test
		currenttick_for_tr_time = *Tick;
				tr_time = currenttick_for_tr_time - tickstart_for_tr_time;
				Set_holding_register(4, tr_time);

#endif

	if(Mode == Normal){

#ifdef speed_test
	tickstart_for_tr_time = *Tick;
#endif
		/* 10. MODBUS PROCESS for Constructed Response Frame */
		uint8_t len = MODBUS_FARME_PROCESS(frame_buffer, response_buffer);


		/* Add CRC to response frame */
	    unsigned short crc = CRC16(response_buffer, len);
		response_buffer[len] = crc >> 8; /* CRC Lo */
		response_buffer[len + 1] = crc; /* CRC Hi */

		len += 2;

#ifdef speed_test
		currenttick_for_tr_time = *Tick;
				tr_time = currenttick_for_tr_time - tickstart_for_tr_time;
				Set_holding_register(5, tr_time);


	tickstart_for_tr_time = *Tick;
#endif

		/* 11. Transmit frame */
		(*transmit_uart_fun)(response_buffer, len);

#ifdef speed_test
		currenttick_for_tr_time = *Tick;
						tr_time = currenttick_for_tr_time - tickstart_for_tr_time;
						Set_holding_register(6, tr_time - RS485_Delay);
#endif
	}

	return MODBUS_OK;
}
