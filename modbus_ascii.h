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

#ifdef __cplusplus
extern "C"
{
#endif

ModbusStatus_t MODBUS_ASCII_MONITOR(unsigned char *mbus_frame_buffer,
		int monitor_fun_timeout, volatile uint32_t *Tick,
		ModbusMonitorMode_t Mode);

#ifdef __cplusplus
}
#endif
#endif
