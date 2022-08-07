/**
 * @file modbus.c
 * @author Mehdi Adham (mehdi.adham@yahoo.com)
 * @brief This library implements the Modbus protocol.
 * @version 0.1
 * @date 2022-07-30
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "modbus.h"

void MODBUS_FARME_PROCESS(unsigned char *RequestFrame, unsigned char *ResponseFrame)
{
    unsigned char function = RequestFrame[1];
    if (function == Read_Coil_Status)
    {
        Read_Coil_Status_Operation(RequestFrame, ResponseFrame);
    }
}
/**
 * @brief 
 * 
 * @param RequestFrame 
 * @param Constructed_ResponseFrame 
 * @return return Quantity of Byte
 */
unsigned char SLAVE_Read_Coil_Status_Operation(unsigned char *RequestFrame,
                                               unsigned char *Constructed_ResponseFrame)
{
    int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    int Num_of_coil = 0xff00 & (RequestFrame[4] << 8) | RequestFrame[5];
    int Start_Coil = Start_address;

    /* Constructing the response frame to the master */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Addrress */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */

    int Byte_Count = 0;

    unsigned int coil_count = Start_Coil;
    unsigned char coil, array_byte = 3;
    int bit = 0;
    int Coil_len = Num_of_coil;

#ifdef debug
    printf("Coil len [%d]\n", Coil_len);
#endif

    while (Coil_len--)
    {
        if (bit == 0)
            Byte_Count++;

        /* 1. get coil value from mem */
        coil = (COIL_MEM[(coil_count / 8) - 1] >> coil_count % 8) & 1;

        /* 2. set finded coli in reponse (in bit located) */
        Constructed_ResponseFrame[array_byte] |= (coil << bit);

        /* 2.1 bit 0 - 7 then plus array byte */
        if (bit++ == 7)
        {
            bit = 0;
            array_byte++;
        }

        coil_count++;
    }
#ifdef debug
    printf("Byte Count [%d]\n", Byte_Count);
#endif
    Constructed_ResponseFrame[2] = Byte_Count; /* Byte Count */

    unsigned short crc = CRC16(Constructed_ResponseFrame, 3 + Byte_Count);
#ifdef debug
    printf("crc 0x%02X\n", crc);
#endif
    Constructed_ResponseFrame[3 + Byte_Count] = crc >> 8;
    Constructed_ResponseFrame[3 + Byte_Count + 1] = crc;

    return 5 + Byte_Count;
}