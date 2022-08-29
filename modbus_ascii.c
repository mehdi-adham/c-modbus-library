/**
 * @file modbus_ascii.c
 * @author Mehdi Adham (mehdi.adham@domain.com)
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

// START    0x3A        ':'
// END      0x0A 0x0D   'CR' 'LF'

ModbusStatus_t MODBUS_ASCII_MONITOR(unsigned char *mbus_frame_buffer,
                                    int monitor_fun_timeout, volatile uint32_t *Tick, ModbusMonitorMode_t Mode)
{
    ModbusStatus_t res;

    unsigned char rec_char;

    uint32_t tickstart_for_monitor_timeout;
    uint32_t currenttick_for_monitor_timeout;

    /* The starting address of the buffer */
    unsigned char *starting_address_of_buffer = mbus_frame_buffer;

    unsigned char (*receive_uart_fun)() = modbus_uart_receive_Handler;
    void (*transmit_uart_fun)(uint8_t * Data,
                              uint16_t length) = modbus_uart_transmit_Handler;
    uint16_t counter = 0;

    /* Initial tick start for monitor timeout management */
    tickstart_for_monitor_timeout = *Tick;
    while (1)
    {
        /* 0. Check for monitor function timeout */
        currenttick_for_monitor_timeout = *Tick;
        if (currenttick_for_monitor_timeout - tickstart_for_monitor_timeout > monitor_fun_timeout)
            return MODBUS_MONITOR_TIMEOUT;

        /* 1. wait for start character ':' */
        do
        {
            res = (*receive_uart_fun)(&rec_char);
            /* Check for monitor function timeout */
            currenttick_for_monitor_timeout = *Tick;
            if (currenttick_for_monitor_timeout - tickstart_for_monitor_timeout > monitor_fun_timeout)
                return MODBUS_MONITOR_TIMEOUT;
        } while (res != ':');

        res = (*receive_uart_fun)(&rec_char);
        add = __HexASCII_Convert(rec_char) * 16;
        res = (*receive_uart_fun)(&rec_char);
        add += __HexASCII_Convert(rec_char);

        unsigned char add = 0;
        /* 2. if out of range allowed address OR Address field not match with slave ID AND not broadcast */
        if (add > MAX_SLAVE_ADDRESS || (add != SLAVE_ADDRESS && add != Broadcast))
            /* return to 0. */
            continue;

        /* Be assigned to the buffer */
        frame_buffer[counter++] = *mbus_frame_buffer++ = add;

        /* 3. receive from uart while CR LF received */
        unsigned char first_char;
        unsigned char second_char;
        do
        {
            res = (*receive_uart_fun)(&first_char);
            first_char = __HexASCII_Convert(first_char) * 16;

            res = (*receive_uart_fun)(&second_char);
            second_char = __HexASCII_Convert(second_char);

            frame_buffer[counter++] = *mbus_frame_buffer++ = first_char + second_char;
        } while (first_char != 0x0D /*'CR'*/ && second_char != 0x0A /*'LF'*/);

        /* 4. Check LRC calculated, that is equal with LRC frame */

        break;
    } /*< End while() for frame time out */

    if (Mode == Normal)
    {
        /* 5. MODBUS PROCESS for Constructed Response Frame */
        uint16_t len = MODBUS_FARME_PROCESS(frame_buffer, response_buffer);

        /* 6. Hex ASCII convert*/

        /* 7. Add LRC to response frame */

        /* 8. Transmit frame */
        (*transmit_uart_fun)(response_buffer, len);
    }

    return MODBUS_OK;
}