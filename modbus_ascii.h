/**
 * @file modbus_ascii.h
 * @author Mehdi Adham (mehdi.adham@yahoo.com)
 * @brief This library implements the Modbus ASCII protocol.
 * @version 0.1
 * @date 2022-08-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __MODBUS_ASCII_H
#define __MODBUS_ASCII_H

#define MAX_BUFFER          256
#define MAX_SLAVE_ADDRESS   247
#define timeout_1C_ascii     1000 /*< (ms) */

ModbusStatus_t MODBUS_ASCII_MONITOR(unsigned char *mbus_frame_buffer,
		int monitor_fun_timeout, volatile uint32_t *Tick,
		ModbusMonitorMode_t Mode);

#endif
