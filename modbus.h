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

#ifdef __cplusplus
extern "C" {
#endif

/* 184/384:[800]   484:[512]     584/984/884:[2000]    M84:[64] */
#define MAX_COIL      2000 
#define MAX_INPUT      2000
/* 184/384:[100]   484:[254]     584/984/884:[125]    M84:[64] */
#define MAX_HOLDING_REGISTERS       254

unsigned char COIL_MEM[MAX_COIL/8];
unsigned char INPUT_MEM[MAX_INPUT/8];
uint16_t HOLDING_REGISTERS_MEM[MAX_HOLDING_REGISTERS];


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

/* Modbus BROADCAST Address */


/* Modbus Protocol Exceptions. */





#ifdef __cplusplus
}
#endif

#endif