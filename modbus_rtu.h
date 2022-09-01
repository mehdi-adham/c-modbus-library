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


void set_slave_ID(unsigned char slave_ID);
ModbusStatus_t MODBUS_RTU_MONITOR(unsigned char *mbus_frame_buffer,
		int monitor_fun_timeout, volatile uint32_t *Tick, ModbusMonitorMode_t Mode);

#ifdef __cplusplus
}
#endif
#endif
