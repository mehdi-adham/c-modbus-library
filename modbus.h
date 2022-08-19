/**
 * @file modbus.h
 * @author Mehdi Adham (mehdi.adham@yahoo.com)
 * @brief This library implements the Modbus protocols.
 * @version 0.1
 * @date 2022-07-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __MODBUS_H
#define __MODBUS_H

#ifndef __STDINT_H
#include <stdint.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* 184/384:[800]   484:[512]     584/984/884:[2000]    M84:[64] */
#define MAX_COIL      	2000
#define MAX_INPUT      	2000

/* 184/384:[100]   484:[254]     584/984/884:[125]    M84:[64] */
#define MAX_HOLDING_REGISTERS       254

/* 184/384:[100]   484:[32]     584/984/884:[125]    M84:[4] */
#define MAX_INPUT_REGISTERS         254

#define Broadcast	0

/* Modbus function codes. */
enum Modbus_Function_Name{
    Read_Coil_Status = 1,
    Read_Input_Status = 2,
    Read_Holding_Registers = 3,
    Read_Input_Registers = 4,
    Force_Single_Coil = 5,
    Preset_Single_Register = 6,
    Read_Exception_Status = 7,
    Fetch_Comm_Event_Counter = 11,
    Fetch_Comm_Event_Log = 12,
    Force_Multiple_Coils = 15,
    Preset_Multiple_Registers = 16,
    Report_Slave_ID = 17,
    Read_General_Reference = 20,
    Write_General_Reference = 21,
    Mask_Write_4X_Register = 22,
    Read_Write_4X_Registers = 23,
    Read_FIFO_Queue =24
};

typedef enum
{
  MODBUS_OK       = 0x00U,
  MODBUS_REC_BYTE_ERROR    = 0x01U,
  MODBUS_REC_BYTE_BUSY     = 0x02U,
  MODBUS_REC_BYTE_TIMEOUT  = 0x03U,
  MODBUS_MONITOR_TIMEOUT  = 0x04U,
  MODBUS_ERROR    = 0x05U,
  MODBUS_MONITOR_BUSY     = 0x06U,
  MODBUS_BROADCAST = 0X07U
} ModbusStatus_t;

typedef enum  {
	Listen_Only = 0,
	Normal = 1
} ModbusMonitorMode_t;

typedef enum Serial_Transmission_Modes {
	RTU,
	ASCII
} Serial_Transmission_Modes_t;

typedef enum Parity{
	EVEN_PARITY,
	ODD_PARITY,
	NONE_PARITY
} Parity_t;

typedef enum Stop_Bit{
	StopBit_1,
	StopBit_2
} Stop_Bit_t;

typedef struct Serial {
	uint32_t *UART;
	int BaudRate;
	Parity_t Parity;
	Stop_Bit_t StopBit;
} Serial_t;

/* Modbus Protocol Exceptions. */

void modbus_serial_init(Serial_t  *serial);
unsigned char Get_coil_status(int coli);
unsigned char MODBUS_FARME_PROCESS(unsigned char *RequestFrame, unsigned char *ResponseFrame);


#ifdef __cplusplus
}
#endif

#endif
