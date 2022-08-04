/**
 * @file modbus_rtu.c
 * @author Mehdi Adham (mehdi.adham@yahoo.com)
 * @brief This library implements the Modbus RTU protocol.
 * @version 0.1
 * @date 2022-07-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "modbus_rtu.h"
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Table of CRC values for high–order byte
 *
 */
static unsigned char auchCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40};

/**
 * @brief Table of CRC values for low–order byte
 *
 */
static char auchCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40};

/**
 * @brief This function calculate CRC for RTU Modbus Protocol.
 * @note This function performs the swapping of the high/low CRC bytes internally.
 * The bytes are already swapped in the CRC value that is returned from the function.
 * Therefore the CRC value returned from the function can be directly placed into
 * the message for transmission.
 * @param puchMsg A pointer to the message buffer containing
 * binary data to be used for generating the CRC.
 * @param usDataLen The quantity of bytes in the message buffer.
 * @retval returns the CRC as a type unsigned short.
 */
unsigned short CRC16(unsigned char *puchMsg, unsigned short usDataLen)
{
    unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
    unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */
    unsigned char uIndex;          /* will index into CRC lookup table */
    while (usDataLen--)            /* pass through message buffer */
    {
        uIndex = uchCRCHi ^ *puchMsg++; /* calculate the CRC */
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}

/**
 * @brief
 *
 * @param mbus_frame_buffer
 * @param receive_uart_fun
 * @return true or false
 */
bool Receive_byte_to_byte(unsigned char *mbus_frame_buffer, unsigned char (*receive_uart_fun)())
{
    static unsigned char uchCRCHi = 0xFF; /* high byte of CRC initialized */
    static unsigned char uchCRCLo = 0xFF; /* low byte of CRC initialized */
    static unsigned char uIndex;          /* will index into CRC lookup table */
    unsigned short calculate_crc;
    unsigned char rec_byte;

    /* 1. **********************************************************/
    /* Get First field (Address Field)*/
    rec_byte = (*receive_uart_fun)();

    /* if out of range allowed address  */
    if (rec_byte > MAX_SLAVE_ADDRESS)
        return false;

    /* if Address field not match with slave ID return false */
    if (rec_byte != SLAVE_ADDRESS)
        return false;

    /* Be assigned to the buffer value */
    *mbus_frame_buffer++ = rec_byte;

    /* Calculate the CRC of this field */
    uIndex = uchCRCHi ^ rec_byte;
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
    uchCRCLo = auchCRCLo[uIndex];

    /* 2. **********************************************************/
    /* Get second field (Function Field)*/
    /* check for Byte timeout 1.5C. if timeout 1.5C return false */
    // timeout_1.5C;
    unsigned char fun = rec_byte = (*receive_uart_fun)();
    /* if(timeout_1.5C)
        return false; */
    /* Be assigned to the buffer value */
    *mbus_frame_buffer++ = rec_byte;
    /* Calculate the CRC of this field */
    uIndex = uchCRCHi ^ rec_byte;
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
    uchCRCLo = auchCRCLo[uIndex];

    /* 3. **********************************************************/
    /* Extracting frame length from frame data */
    int len = 0;
    /* Calculate len (byte count) for 1,2,3,4 function */
    if (fun == 1 || fun == 2 || fun == 3 || fun == 4)
    {
        /* 3 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 4 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 5 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        unsigned char Hi = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 6 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        unsigned char Lo = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        if (fun == 1 || fun == 2)
        {
            len = (Hi << 8 | Lo) / 8;
        }
        else
        {
            len = (Hi << 8 | Lo) * 2;
        }
    }
    else if (fun == 5 || fun == 6)
    {
        len = 3;
    }
    else if (fun == 7 || fun == 11 || fun == 12 || fun == 17)
    {
        len = 0;
    }
    else if (fun == 15 || fun == 16)
    {
        /* 3 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 4 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 5 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 6 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 7 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        len = rec_byte;
    }
    else if (fun == 20 || fun == 21)
    {
        /* 3 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        len = rec_byte;
    }
    else if (fun == 22)
    {
        len = 6;
    }
    else if (fun == 23)
    {
        /* 3 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 4 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 5 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 6 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 7 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 8 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 9 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        /* 10 */
        rec_byte = (*receive_uart_fun)();
        *mbus_frame_buffer++ = rec_byte;
        /* Calculate the CRC of this field */
        uIndex = uchCRCHi ^ rec_byte;
        uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
        uchCRCLo = auchCRCLo[uIndex];

        len = rec_byte;
    }
    else if (fun == 24)
    {
        len = 2;
    }

    /* 4. **********************************************************/
    while (len)
    {
        /* check for Byte timeout 1.5C. if timeout 1.5C return false */
        // timeout_1.5C;
        rec_byte = (*receive_uart_fun)();
        /* if(timeout_1.5C)
            return false; */

        /* Be assigned to the buffer value */
        *mbus_frame_buffer++ = rec_byte;
    }

    /* 5. **********************************************************/
    rec_byte = (*receive_uart_fun)();
    /* Be assigned to the buffer value */
    *mbus_frame_buffer++ = rec_byte;
    /* Calculate the CRC of this field */
    uIndex = uchCRCHi ^ rec_byte;
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
    uchCRCLo = auchCRCLo[uIndex];

    rec_byte = (*receive_uart_fun)();
    /* Be assigned to the buffer value */
    *mbus_frame_buffer++ = rec_byte;
    /* Calculate the CRC of this field */
    uIndex = uchCRCHi ^ rec_byte;
    uchCRCHi = uchCRCLo ^ auchCRCHi[uIndex];
    uchCRCLo = auchCRCLo[uIndex];

    /* The calculated CRC is assigned in the value of calcul_crc */
    calculate_crc = (uchCRCHi << 8 | uchCRCLo);

    /* Check CRC calculated, that is equal with CRC frame */
    if (calculate_crc != (unsigned short)((*(mbus_frame_buffer - 2) << 8) | *(mbus_frame_buffer - 1)))
        return false;

    return true;
}

void MODBUS_RTU_MONITOR(unsigned char (*receive_uart_fun)())
{
    unsigned char fram_buf[MAX_BUFFER];
    /* Wait while be silent bus for least 3.5 character. */

    /* Receive byte to byte from serial.
     * if address field not match with slave ID return false
     * if timeout 1.5C return false
     */
    bool ResultOfreceive = Receive_byte_to_byte(fram_buf, receive_uart_fun);
    if (ResultOfreceive != true)
    {
        MODBUS_MONITOR();
    }
    else
    {
        MODBUS_FARME_PROCESS(fram_buf);
    }
}
