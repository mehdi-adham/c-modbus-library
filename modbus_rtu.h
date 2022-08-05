/**
 * @file modbus_rtu.h
 * @author Mehdi Adham (mehdi.adham@domain.com)
 * @brief This library implements the Modbus RTU protocol.
 * @version 0.1
 * @date 2022-07-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>

#define SLAVE_ADDRESS       0x11
#define MAX_BUFFER          256
#define MAX_SLAVE_ADDRESS   247

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

int request_timeout = 500;
int request_timer = 0; 
int RX_PIN ;
int DIR_PIN;
long long int *timer;

unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen);
void MODBUS_RTU_MONITOR(unsigned char *mbus_frame_buffer, int monitor_fun_timeout,
                        unsigned char (*receive_uart_fun)());
#ifdef __cplusplus
}
#endif
#endif
