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

/* Modbus memory map for COIL, INPUT, HOLDING_REGISTERS, INPUT_REGISTERS */
#define MAX_COIL      				8 	/*< 184/384:[800]   484:[512]     584/984/884:[2000]    M84:[64] */
#define MAX_INPUT      				0 		/*< 184/384:[800]   484:[512]     584/984/884:[2000]    M84:[64] */
#define MAX_HOLDING_REGISTERS       8 	/*< 184/384:[100]   484:[254]     584/984/884:[125]    M84:[64] */
#define MAX_INPUT_REGISTERS         0 		/*< 184/384:[100]   484:[32]     584/984/884:[125]    M84:[4] */

#define Broadcast	0

#define RS485_Delay							5  		/*< ms */
#define Modbus_Communication_timeout		10 		/*< ms */

#define MAX_BUFFER          255
#define MAX_SLAVE_ADDRESS   247

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
	uint32_t BaudRate;
	Parity_t Parity;
	Stop_Bit_t StopBit;
} Serial_t;


/* Modbus Exceptions. */
typedef enum Modbus_Exception_Code{
	ILLEGAL_FUNCTION		= 1, /*< The function code received in the query is not an allowable action for the slave.
										If a Poll Program Complete command was issued, this code indicates that no program
										function preceded it.*/

	ILLEGAL_DATA_ADDRESS 	= 2, /*< The data address received in the query is not an allowable address for the slave.*/

	ILLEGAL_DATA_VALUE		= 3, /*< A value contained in the query data field is not an allowable value for the slave.*/

	SLAVE_DEVICE_FAILURE	= 4, /*< An unrecoverable error occurred while the slave was attempting to perform the requested action.*/

	ACKNOWLEDGE				= 5, /*< The slave has accepted the request and is processing it, but a long duration of time will be
										required to do so. This response is returned to prevent a timeout error from occurring in the master.
										The master can next issue a Poll Program Complete message to determine if processing is completed.*/

	SLAVE_DEVICE_BUSY		= 6, /*< The slave is engaged in processing a long–duration program command. The master should retransmit
										the message later when the slave is free.*/

	NEGATIVE_ACKNOWLEDGE	= 7, /*< The slave cannot perform the program function received in the query. This code is returned for an
											unsuccessful programming request using function code 13 or 14 decimal.
											The master should request diagnostic or error information from the slave.*/

	MEMORY_PARITY_ERROR		= 8 /*< The slave attempted to read extended memory, but detected a parity error in the memory.
										The master can retry the request, but service may be required on the slave device.*/
}Modbus_Exception_Code_t;

typedef struct frame_parameter{
	unsigned char slave_ID;
	unsigned char function;
	unsigned int start_address;
	unsigned char quantity;
} frame_parameter_t;

typedef struct communication_parameter {
	unsigned char RS_485_Delay;
	unsigned char communication_timeout;
	unsigned char communication_retry_Times;
}communication_parameter_t;


void modbus_serial_init(Serial_t  *serial);

void set_slave_ID(unsigned char slave_ID);
unsigned char get_slave_ID(void);

unsigned char Get_coil_status(int coli);
void Set_coil_status(int coil, unsigned int status);
unsigned int Get_holding_register(int Holding_Register_Address);
void Set_holding_register(int Holding_Register_Address, unsigned int value);

unsigned char MODBUS_FARME_PROCESS(unsigned char *RequestFrame, unsigned char *ResponseFrame);
unsigned char Modbus_Exception(Modbus_Exception_Code_t Modbus_Exception_Code, unsigned char *ResponseFrame);
ModbusStatus_t MODBUS_MASTER_PROCESS(frame_parameter_t *frame_parameter, 
				communication_parameter_t *communication_parameter, 
				Serial_Transmission_Modes_t transmission_mode, volatile uint32_t *Tick);

#ifdef __cplusplus
}
#endif

#endif
