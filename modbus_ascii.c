/**
 * @file modbus_ascii.c
 * @author Mehdi Adham (mehdi.adham@yahoo.com)
 * @brief This library implements the Modbus ASCII protocol.
 * @version 0.1
 * @date 2022-08-29
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "modbus.h"
#include "modbus_ascii.h"
#include "modbus_handler.h"

/* Private define */
#define __HexASCII_Convert(__HexASCII__) (__HexASCII__ < 58 ? (__HexASCII__ - 48) : (__HexASCII__ - 55))
#define __TO_HexASCII_(__Hex__) (__Hex__ < 10 ? (__Hex__ + 48) : (__Hex__ + 55))

extern unsigned char SLAVE_ADDRESS;

unsigned char frame_buffer[MAX_BUFFER];
unsigned char response_buffer[MAX_BUFFER];
unsigned char Hex_ASCII_response_buffer[MAX_BUFFER];

// START   0x3A        ':'
// END     0x0D 0x0A   'CR' 'LF'  '\r' '\n'

/**
 * @brief This function calculate LRC for ASCII Modbus Protocol.
 *
 * @param auchMsg A pointer to the message buffer containing binary data to be used for generating the LRC
 * @param usDataLen The quantity of bytes in the message buffer
 * @return LRC
 */
static unsigned char LRC(unsigned char *auchMsg, unsigned short usDataLen) {
	unsigned char uchLRC = 0; /* LRC char initialized */
	while (usDataLen--) /* pass through message buffer */
		uchLRC += *auchMsg++; /* add buffer byte without carry */
	return ((unsigned char) (-((char) uchLRC))); /* return twos complement */
}

/**
 * @brief
 *
 * @param mbus_frame_buffer
 * @param monitor_fun_timeout
 * @param Tick
 * @param Mode
 * @return ModbusStatus_t
 */
ModbusStatus_t MODBUS_ASCII_MONITOR(unsigned char *mbus_frame_buffer,
		int monitor_fun_timeout, volatile uint32_t *Tick,
		ModbusMonitorMode_t Mode) {
	ModbusStatus_t res;
	unsigned char uchLRC = 0;
	unsigned char rec_char;

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

		/* 1. wait for start character ':' */
		do {
			res = (*receive_uart_fun)(&rec_char);
			/* Check for monitor function timeout */
			currenttick_for_monitor_timeout = *Tick;
			if (currenttick_for_monitor_timeout - tickstart_for_monitor_timeout
					> monitor_fun_timeout)
				return MODBUS_MONITOR_TIMEOUT;
		} while (rec_char != 0x3A);

		unsigned char add = 0;
		res = (*receive_uart_fun)(&rec_char);
		add = __HexASCII_Convert(rec_char) * 16;
		res = (*receive_uart_fun)(&rec_char);
		add += __HexASCII_Convert(rec_char);

		/* 2. if out of range allowed address OR Address field not match with slave ID AND not broadcast */
		if (add > MAX_SLAVE_ADDRESS
				|| (add != SLAVE_ADDRESS && add != Broadcast))
			/* return to 0. */
			continue;

		/* Be assigned to the buffer */
		frame_buffer[counter++] = *mbus_frame_buffer++ = add;
		/* Calculate the LRC of this field */
		uchLRC += add;

		/* 3. Be assigned to the fun/buffer value and Calculate the LRC of this field */
		unsigned char fun = 0;
		res = (*receive_uart_fun)(&rec_char);
		if (res != MODBUS_OK)
			return res;
		fun = __HexASCII_Convert(rec_char) * 16;
		res = (*receive_uart_fun)(&rec_char);
		if (res != MODBUS_OK)
			return res;
		fun += __HexASCII_Convert(rec_char);

		/* Be assigned to the buffer */
		frame_buffer[counter++] = *mbus_frame_buffer++ = fun;
		/* Calculate the LRC of this field */
		uchLRC += fun;

		/* 3.1 Extracting length (register or coil ...) from frame data */
		int len = 0;
		unsigned char temp;
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
				res = (*receive_uart_fun)(&rec_char);
				if (res != MODBUS_OK)
					return res;
				temp = __HexASCII_Convert(rec_char) * 16;
				res = (*receive_uart_fun)(&rec_char);
				if (res != MODBUS_OK)
					return res;
				temp += __HexASCII_Convert(rec_char);

				frame_buffer[counter++] = *mbus_frame_buffer++ = temp;
				/* Calculate the LRC of this field */
				uchLRC += temp;
			}

			len = temp;
		} else if (fun == Read_General_Reference
				|| fun == Write_General_Reference) {
			res = (*receive_uart_fun)(&rec_char);
			if (res != MODBUS_OK)
				return res;
			temp = __HexASCII_Convert(rec_char) * 16;
			res = (*receive_uart_fun)(&rec_char);
			if (res != MODBUS_OK)
				return res;
			temp += __HexASCII_Convert(rec_char);

			frame_buffer[counter++] = *mbus_frame_buffer++ = temp;
			/* Calculate the LRC of this field */
			uchLRC += temp;

			len = temp;
		} else if (fun == Mask_Write_4X_Register) {
			len = 6;
		} else if (fun == Read_Write_4X_Registers) {
			int l = 9;
			while (l--) {
				res = (*receive_uart_fun)(&rec_char);
				if (res != MODBUS_OK)
					return res;
				temp = __HexASCII_Convert(rec_char) * 16;
				res = (*receive_uart_fun)(&rec_char);
				if (res != MODBUS_OK)
					return res;
				temp += __HexASCII_Convert(rec_char);

				frame_buffer[counter++] = *mbus_frame_buffer++ = temp;
				/* Calculate the LRC of this field */
				uchLRC += temp;
			}

			len = temp;
		} else if (fun == Read_FIFO_Queue) {
			len = 2;
		}

		unsigned char first_char;
		unsigned char second_char;
		/* 3.2 Remain byte */
		while (len) {
			/* check overflow */
			if (counter > MAX_BUFFER) {
				/* clear buffer */
				while (starting_address_of_buffer < mbus_frame_buffer) {
					frame_buffer[counter--] = *mbus_frame_buffer-- = 0x00;
				}
				/* return to 0. */
				continue;
			}

			res = (*receive_uart_fun)(&first_char);
			if (res != MODBUS_OK)
				return res;
			first_char = __HexASCII_Convert(first_char) * 16;

			res = (*receive_uart_fun)(&second_char);
			if (res != MODBUS_OK)
				return res;
			second_char = __HexASCII_Convert(second_char);

			frame_buffer[counter++] = *mbus_frame_buffer++ = (first_char
					+ second_char);
			/* Calculate the LRC of this field */
			uchLRC += (first_char + second_char);

			len--;
		}

		/* 4. Check LRC calculated, that is equal with LRC frame */
		/* Get LRC */
		unsigned char Frame_LRC_Hi;
		res = (*receive_uart_fun)(&Frame_LRC_Hi);
		if (res != MODBUS_OK)
			return res;
		unsigned char Fram_LRC_Lo;
		res = (*receive_uart_fun)(&Fram_LRC_Lo);
		if (res != MODBUS_OK)
			return res;

		unsigned char LRC_calculated = (unsigned char) (-((char) uchLRC));
		unsigned char Calculated_LRC_Hi = __TO_HexASCII_(LRC_calculated / 16);
		unsigned char Calculated_LRC_Lo = __TO_HexASCII_(LRC_calculated % 16);

		if (Frame_LRC_Hi != Calculated_LRC_Hi
				|| Fram_LRC_Lo != Calculated_LRC_Lo) {
			/* clear buffer */
			while (starting_address_of_buffer < mbus_frame_buffer) {
				frame_buffer[counter--] = *mbus_frame_buffer-- = 0x00;
			}
			/* return to 0. */
			continue;
		}

		break;
	} /*< End while() for frame time out */

	if (Mode == Normal) {
		/* 5. MODBUS PROCESS for Constructed Response Frame */
		uint16_t len = MODBUS_FARME_PROCESS(frame_buffer, response_buffer);

		/* 6.Convert response_buffer to ascii response */
		/* 6.1 Apend Satrt char */
		Hex_ASCII_response_buffer[0] = ':';

		/* 6.2 convert buffer To Hex ASCII  */
		uint16_t _len = 1;
		for (int char_n = 0; char_n < len; char_n++) {
			Hex_ASCII_response_buffer[_len++] = __TO_HexASCII_(
					response_buffer[char_n] / 16);
			Hex_ASCII_response_buffer[_len++] = __TO_HexASCII_(
					response_buffer[char_n] % 16);
		}

		/* 6.3 Apend Calculated LRC into Hex_ASCII response frame */
		unsigned char LRC_calculated = LRC(response_buffer, len);
		unsigned char Apend_LRC_Hi = __TO_HexASCII_(LRC_calculated / 16);
		unsigned char Apend_LRC_Lo = __TO_HexASCII_(LRC_calculated % 16);

		Hex_ASCII_response_buffer[_len++] = Apend_LRC_Hi;
		Hex_ASCII_response_buffer[_len++] = Apend_LRC_Lo;

		/* 6.4 Apend End char */
		Hex_ASCII_response_buffer[_len++] = 0x0D;
		Hex_ASCII_response_buffer[_len++] = 0x0A;

		/* 7. Transmit ASCII frame */
		(*transmit_uart_fun)(Hex_ASCII_response_buffer, _len);
	}

	return MODBUS_OK;
}
