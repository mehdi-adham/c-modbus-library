/**
 * @file modbus_handler.h
 * @author Mehdi Adham (mehdi.adham@yahoo.com)
 * @brief This library Handle the Modbus Communication.
 * @version 0.1
 * @date 2022-08-29
 *
 * @copyright Copyright (c) 2022
 *
 */


#ifndef __MODBUS_HANDLER_H
#define __MODBUS_HANDLER_H

/**
 * @brief
 * NOTE : This function should not be modified, when the callback is needed,
 the uart_receive Handler could be implemented in the user file
 * @return Modbus Status
 */
__attribute__((weak))	  ModbusStatus_t modbus_uart_receive_Handler(uint8_t *Data) {
	return 0;
}

/**
 * @brief
 * NOTE : This function should not be modified, when the callback is needed,
 the uart transmit Handler could be implemented in the user file
 * @return None
 */
__attribute__((weak)) void modbus_uart_transmit_Handler(uint8_t *Data,
		uint16_t length) {

}

/**
 * @brief
 * NOTE : This function should not be modified, when the callback is needed,
 the uart_init Handler could be implemented in the user file
 * @return Modbus Status
 */
__attribute__((weak)) void modbus_uart_init_Handler(Serial_t *Serial) {

}


#endif
