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


/* private function */
static unsigned char SLAVE_Read_Coil_Status_Operation(unsigned char *RequestFrame,
                                               unsigned char *Constructed_ResponseFrame);
static unsigned char SLAVE_Read_Input_Status_Operation(unsigned char *RequestFrame,
                                               unsigned char *Constructed_ResponseFrame);                                              
static unsigned char SLAVE_Read_Holding_Registers_Operation(unsigned char *RequestFrame,
                                               unsigned char *Constructed_ResponseFrame);


unsigned char MODBUS_FARME_PROCESS(unsigned char *RequestFrame, unsigned char *ResponseFrame)
{
    unsigned char function = RequestFrame[1];
    if (function == Read_Coil_Status){
       return SLAVE_Read_Coil_Status_Operation(RequestFrame, ResponseFrame);
    }
    else if(function == Read_Input_Status){
       return SLAVE_Read_Input_Status_Operation(RequestFrame, ResponseFrame);
    }
    else 
       return SLAVE_Read_Holding_Registers_Operation(RequestFrame, ResponseFrame);
    
}
/**
 * @brief 
 * 
 * @param RequestFrame 
 * @param Constructed_ResponseFrame 
 * @return return Frame length
 */
unsigned char SLAVE_Read_Coil_Status_Operation(unsigned char *RequestFrame,
                                               unsigned char *Constructed_ResponseFrame)
{
    int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    int Num_of_coil = RequestFrame[4] << 8 | RequestFrame[5];
    int Start_Coil = Start_address;

    /* Constructing the response frame to the master */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Addrress */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */

    int Byte_Count = 0;

    unsigned int coil_count = Start_Coil;
    unsigned char coil, array_byte = 3;
    int bit = 7;
    int Coil_len = Num_of_coil;

#ifdef debug
    printf("Coil len [%d]\n", Coil_len);
#endif

    while (Coil_len--)
    {
        if (bit == 7)
            Byte_Count++;

        /* 1. get coil value from mem. note: coils 1–16 are addressed as 0–15 (coil_count % 8) */
        coil = (COIL_MEM[(coil_count / 8) - 1] >> coil_count % 8) & 1;

        /* 2. set finded coli in reponse (in bit located) */
        Constructed_ResponseFrame[array_byte] |= (coil << bit);

        /* 2.1 bit 0 - 7 then plus array byte */
        if (bit-- == 0)
        {
            bit = 7;
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
    Constructed_ResponseFrame[3 + Byte_Count] = crc >> 8; /* CRC Lo */
    Constructed_ResponseFrame[3 + Byte_Count + 1] = crc; /* CRC Hi */

    return 5 + Byte_Count;
}

/**
 * @brief 
 * 
 * @param RequestFrame 
 * @param Constructed_ResponseFrame 
 * @return return Frame length
 */
unsigned char SLAVE_Read_Input_Status_Operation(unsigned char *RequestFrame,
                                               unsigned char *Constructed_ResponseFrame)
{
    int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    int Num_of_Input = RequestFrame[4] << 8 | RequestFrame[5];
    int Start_Input = Start_address;

    /* Constructing the response frame to the master */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Addrress */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */

    int Byte_Count = 0;

    unsigned int Input_count = Start_Input;
    unsigned char Input, array_byte = 3;
    int bit = 7;
    int Input_len = Num_of_Input;

#ifdef debug
    printf("Input len [%d]\n", Input_len);
#endif

    while (Input_len--)
    {
        if (bit == 7)
            Byte_Count++;

        /* 1. get Input value from mem. note: Inputs 1–16 are addressed as 0–15 (Input_count % 8) */
        Input = (INPUT_MEM[(Input_count / 8) - 1] >> Input_count % 8) & 1;

        /* 2. set finded coli in reponse (in bit located) */
        Constructed_ResponseFrame[array_byte] |= (Input << bit);

        /* 2.1 bit 0 - 7 then plus array byte */
        if (bit-- == 0)
        {
            bit = 7;
            array_byte++;
        }

        Input_count++;
    }
#ifdef debug
    printf("Byte Count [%d]\n", Byte_Count);
#endif
    Constructed_ResponseFrame[2] = Byte_Count; /* Byte Count */

    unsigned short crc = CRC16(Constructed_ResponseFrame, 3 + Byte_Count);
#ifdef debug
    printf("crc 0x%02X\n", crc);
#endif
    Constructed_ResponseFrame[3 + Byte_Count] = crc >> 8; /* CRC Lo */
    Constructed_ResponseFrame[3 + Byte_Count + 1] = crc; /* CRC Hi */

    return 5 + Byte_Count;
}

/**
 * @brief 
 * 
 * @param RequestFrame 
 * @param Constructed_ResponseFrame 
 * @return return Frame length
 */
unsigned char SLAVE_Read_Holding_Registers_Operation(unsigned char *RequestFrame,
                                               unsigned char *Constructed_ResponseFrame){
    int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    int Num_of_Holding_Registers = RequestFrame[4] << 8 | RequestFrame[5];
    int Start_Holding_Registers = Start_address;

        /* Constructing the response frame to the master */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Addrress */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */

    int Byte_Count = Num_of_Holding_Registers * 2;  /* Byte Count */

    int Holding_Registers_len = Num_of_Holding_Registers;
    unsigned int Holding_Registers_count = Start_Holding_Registers;
    unsigned char byte = 3;
    while (Holding_Registers_len--)
    {
        Constructed_ResponseFrame[byte++] = 
        HOLDING_REGISTERS_MEM[Holding_Registers_count] >> 8;        /* Hi Holding Registers */

        Constructed_ResponseFrame[byte++] = 
        0x00ff & HOLDING_REGISTERS_MEM[Holding_Registers_count];     /* Lo Holding Registers */
   
        Holding_Registers_count++;

    }

    Constructed_ResponseFrame[2] = Byte_Count; /* Byte Count */   

    unsigned short crc = CRC16(Constructed_ResponseFrame, 3 + Byte_Count);

    Constructed_ResponseFrame[3 + Byte_Count] = crc >> 8; /* CRC Lo */
    Constructed_ResponseFrame[3 + Byte_Count + 1] = crc; /* CRC Hi */

    return 5 + Byte_Count;
}
