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
#include "CRC.h"

unsigned char Run_for_first_time = First_time;

static unsigned char COIL_MEM[MAX_COIL / 8] /* start for test */ = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0}; /* end for test */

static unsigned char INPUT_MEM[MAX_INPUT / 8] /* start for test */ = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1}; /* end for test */

static uint16_t HOLDING_REGISTERS_MEM[MAX_HOLDING_REGISTERS] /* start for test */ = {
    0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0, 0x4180, 0x0141, 0xC001, 0x80C0,
    0x4180, 0x0041, 0xC100, 0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0, 0x4180,
    0x0041, 0xC100, 0x81C1, 0x4081, 0x0040, 0xC100, 0x81C1, 0x4081, 0x0140,
    0xC001, 0x80C0, 0x4180, 0x0141, 0xC001, 0x80C0, 0x4180, 0x0041, 0xC100,
    0x81C1, 0x4081, 0x0040, 0xC100, 0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0,
    0x4180, 0x0041, 0xC100, 0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0, 0x4180,
    0x0141, 0xC001, 0x80C0, 0x4180, 0x0041, 0xC100, 0x81C1, 0x4081, 0x0140,
    0xC001, 0x80C0, 0x4180, 0x0041, 0xC100, 0x81C1, 0x4081, 0x0040, 0xC100,
    0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0, 0x4180, 0x0041, 0xC100, 0x81C1,
    0x4081, 0x0140, 0xC001, 0x80C0, 0x4180, 0x0141, 0xC001, 0x00C0, 0xC100,
    0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0, 0x4180, 0x0141, 0xC001, 0x80C0,
    0x4180, 0x0041, 0xC100, 0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0, 0x4180,
    0x0041, 0xC100, 0x81C1, 0x4081, 0x0040, 0xC100, 0x81C1, 0x4081, 0x0140,
    0xC001, 0x80C0, 0x4180, 0x0141, 0xC001, 0x80C0, 0x4180, 0x0041}; /* end for test */

static uint16_t INPUT_REGISTERS_MEM[MAX_INPUT_REGISTERS] /* start for test */ = {
    0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0, 0x4180, 0x0141, 0xC001, 0x80C0,
    0x4180, 0x0041, 0xC100, 0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0, 0x4180,
    0x0041, 0xC100, 0x81C1, 0x4081, 0x0040, 0xC100, 0x81C1, 0x4081, 0x0140,
    0xC001, 0x80C0, 0x4180, 0x0141, 0xC001, 0x80C0, 0x4180, 0x0041, 0xC100,
    0x81C1, 0x4081, 0x0040, 0xC100, 0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0,
    0x4180, 0x0041, 0xC100, 0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0, 0x4180,
    0x0141, 0xC001, 0x80C0, 0x4180, 0x0041, 0xC100, 0x81C1, 0x4081, 0x0140,
    0xC001, 0x80C0, 0x4180, 0x0041, 0xC100, 0x81C1, 0x4081, 0x0040, 0xC100,
    0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0, 0x4180, 0x0041, 0xC100, 0x81C1,
    0x4081, 0x0140, 0xC001, 0x80C0, 0x4180, 0x0141, 0xC001, 0x00C0, 0xC100,
    0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0, 0x4180, 0x0141, 0xC001, 0x80C0,
    0x4180, 0x0041, 0xC100, 0x81C1, 0x4081, 0x0140, 0xC001, 0x80C0, 0x4180,
    0x0041, 0xC100, 0x81C1, 0x4081, 0x0040, 0xC100, 0x81C1, 0x4081, 0x0140,
    0xC001, 0x80C0, 0x4180, 0x0141, 0xC001, 0x80C0, 0x4180, 0x0041}; /* end for test */

/* private function's */
static unsigned char SLAVE_Read_Coil_Status_Operation(unsigned char *RequestFrame,
                                                      unsigned char *Constructed_ResponseFrame);
static unsigned char SLAVE_Read_Input_Status_Operation(unsigned char *RequestFrame,
                                                       unsigned char *Constructed_ResponseFrame);
static unsigned char SLAVE_Read_Holding_Registers_Operation(unsigned char *RequestFrame,
                                                            unsigned char *Constructed_ResponseFrame);
static unsigned char SLAVE_Read_Input_Registers_Operation(unsigned char *RequestFrame,
                                                          unsigned char *Constructed_ResponseFrame);
static unsigned char SLAVE_Force_Single_Coil_Operation(unsigned char *RequestFrame,
                                                       unsigned char *Constructed_ResponseFrame);
static unsigned char SLAVE_Preset_Single_Register_Operation(unsigned char *RequestFrame,
                                                            unsigned char *Constructed_ResponseFrame);
static unsigned char SLAVE_Read_Exception_Status_Operation(unsigned char *RequestFrame,
                                                           unsigned char *Constructed_ResponseFrame);
static unsigned char SLAVE_Fetch_Comm_Event_Counter_Operation(unsigned char *RequestFrame,
                                                              unsigned char *Constructed_ResponseFrame);
static unsigned char SLAVE_Fetch_Comm_Event_Log_Operation(unsigned char *RequestFrame,
                                                          unsigned char *Constructed_ResponseFrame);
static unsigned char SLAVE_Force_Multiple_Coils_Operation(unsigned char *RequestFrame,
                                                          unsigned char *Constructed_ResponseFrame);
static unsigned char SLAVE_Preset_Multiple_Register_Operation(unsigned char *RequestFrame,
                                                          unsigned char *Constructed_ResponseFrame);
/**
 * @brief This function is to prepare the response to the master and perform the commands
 * (write/read on the coil and register, etc.) according to the function code.
 *
 * @param RequestFrame
 * @param ResponseFrame
 * @return Return length Of Response Frame
 */
unsigned char MODBUS_FARME_PROCESS(unsigned char *RequestFrame, unsigned char *ResponseFrame)
{
    unsigned char lengthOfResponseFrame;
    unsigned char function = RequestFrame[1];

#ifdef WITHOUT_EEPROM
    /**
     * After the micro reset, when the monitor function is called for the first
     * time in the program, a write command must be given by the master, not a
     * read command, because the memory of all registers, coils, etc has been
     * reset to zero, and it must be initialized again.
     */
    if (Run_for_first_time != First_time)
    {
#endif

        if (function == Read_Coil_Status)
        {
            lengthOfResponseFrame = SLAVE_Read_Coil_Status_Operation(RequestFrame,
                                                                     ResponseFrame);
        }
        else if (function == Read_Input_Status)
        {
            lengthOfResponseFrame = SLAVE_Read_Input_Status_Operation(RequestFrame,
                                                                      ResponseFrame);
        }
        else if (function == Read_Holding_Registers)
        {
            lengthOfResponseFrame = SLAVE_Read_Holding_Registers_Operation(RequestFrame,
                                                                           ResponseFrame);
        }
        else if (function == Read_Input_Registers)
        {
            lengthOfResponseFrame = SLAVE_Read_Input_Registers_Operation(RequestFrame,
                                                                         ResponseFrame);
        }

#ifdef WITHOUT_EEPROM
    }
    else if (Run_for_first_time == First_time)
    {
        Run_for_first_time = !First_time;
#endif

        else if (function == Force_Single_Coil)
        {
            lengthOfResponseFrame = SLAVE_Force_Single_Coil_Operation(RequestFrame,
                                                                      ResponseFrame);
        }
        else if (function == Preset_Single_Register)
        {
            lengthOfResponseFrame = SLAVE_Preset_Single_Register_Operation(RequestFrame,
                                                                           ResponseFrame);
        }
        else if(function == Force_Multiple_Coils){
            lengthOfResponseFrame = SLAVE_Force_Multiple_Coils_Operation(RequestFrame,
                                                                           ResponseFrame);
        }

#ifdef WITHOUT_EEPROM
    }
#endif

    return lengthOfResponseFrame;
}
/**
 * @brief
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Read_Coil_Status_Operation(unsigned char *RequestFrame,
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
        coil = (COIL_MEM[(coil_count / 8)] >> coil_count % 8) & 1;

        /* 2. set finded coli in reponse (in bit located) */
        if (coil == 1)
            Constructed_ResponseFrame[array_byte] |= (1 << bit);
        else
            Constructed_ResponseFrame[array_byte] &= ~(1 << bit);

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
    Constructed_ResponseFrame[3 + Byte_Count + 1] = crc;  /* CRC Hi */

    return 5 + Byte_Count;
}

/**
 * @brief
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Read_Input_Status_Operation(unsigned char *RequestFrame,
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
        Input = (INPUT_MEM[(Input_count / 8)] >> Input_count % 8) & 1;

        /* 2. set finded coli in reponse (in bit located) */
        if (Input == 1)
            Constructed_ResponseFrame[array_byte] |= (1 << bit);
        else
            Constructed_ResponseFrame[array_byte] &= ~(1 << bit);

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
    Constructed_ResponseFrame[3 + Byte_Count + 1] = crc;  /* CRC Hi */

    return 5 + Byte_Count;
}

/**
 * @brief
 * @note Note:Data is scanned in the slave at the rate of 125 registers per scan for
 *  984–X8X controllers (984–685, etc), and at the rate of 32 registers per scan for
 * all other controllers. The response is returned when the data is completely assembled
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Read_Holding_Registers_Operation(unsigned char *RequestFrame,
                                                            unsigned char *Constructed_ResponseFrame)
{
    int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    int Num_of_Holding_Registers = RequestFrame[4] << 8 | RequestFrame[5];
    int Start_Holding_Registers = Start_address;

    /* Constructing the response frame to the master */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Addrress */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */

    int Byte_Count = Num_of_Holding_Registers * 2; /* Byte Count */

    int Holding_Registers_len = Num_of_Holding_Registers;
    unsigned int Holding_Registers_count = Start_Holding_Registers;
    unsigned char byte = 3;
    while (Holding_Registers_len--)
    {
        Constructed_ResponseFrame[byte++] =
            HOLDING_REGISTERS_MEM[Holding_Registers_count] >> 8; /* Hi Holding Registers */

        Constructed_ResponseFrame[byte++] =
            0x00ff & HOLDING_REGISTERS_MEM[Holding_Registers_count]; /* Lo Holding Registers */

        Holding_Registers_count++;
    }

    Constructed_ResponseFrame[2] = Byte_Count; /* Byte Count */

    unsigned short crc = CRC16(Constructed_ResponseFrame, 3 + Byte_Count);

    Constructed_ResponseFrame[3 + Byte_Count] = crc >> 8; /* CRC Lo */
    Constructed_ResponseFrame[3 + Byte_Count + 1] = crc;  /* CRC Hi */

    return 5 + Byte_Count;
}

/**
 * @brief
 * @note Note:Data is scanned in the slave at the rate of 125 registers per scan for
 *  984–X8X controllers (984–685, etc), and at the rate of 32 registers per scan for
 * all other controllers. The response is returned when the data is completely assembled
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Read_Input_Registers_Operation(unsigned char *RequestFrame,
                                                          unsigned char *Constructed_ResponseFrame)
{
    int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    int Num_of_Input_Registers = RequestFrame[4] << 8 | RequestFrame[5];

    /* Constructing the response frame to the master */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Addrress */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */

    int Byte_Count = Num_of_Input_Registers * 2; /* Byte Count */

    int Input_Registers_len = Num_of_Input_Registers;
    unsigned int Input_Registers_count = Start_address;
    unsigned char byte = 3;
    while (Input_Registers_len--)
    {
        Constructed_ResponseFrame[byte++] =
            INPUT_REGISTERS_MEM[Input_Registers_count] >> 8; /* Hi Input Registers */

        Constructed_ResponseFrame[byte++] =
            0x00ff & INPUT_REGISTERS_MEM[Input_Registers_count]; /* Lo Input Registers */

        Input_Registers_count++;
    }

    Constructed_ResponseFrame[2] = Byte_Count; /* Byte Count */

    unsigned short crc = CRC16(Constructed_ResponseFrame, 3 + Byte_Count);

    Constructed_ResponseFrame[3 + Byte_Count] = crc >> 8; /* CRC Lo */
    Constructed_ResponseFrame[3 + Byte_Count + 1] = crc;  /* CRC Hi */

    return 5 + Byte_Count;
}

/**
 * @brief
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Force_Single_Coil_Operation(unsigned char *RequestFrame,
                                                       unsigned char *Constructed_ResponseFrame)
{
    int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    int Force_Data = RequestFrame[4] << 8 | RequestFrame[5];

    /* Write */
    if (Force_Data == 0xff00)
        COIL_MEM[(Start_address / 8)] |= (1 << (Start_address % 8));
    else
        COIL_MEM[(Start_address / 8)] &= ~(1 << (Start_address % 8));

    /* The normal response is an echo of the query, returned after the coil
    state has been forced. */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Addrress */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */
    Constructed_ResponseFrame[2] = RequestFrame[2]; /* Coil Address Hi */
    Constructed_ResponseFrame[3] = RequestFrame[3]; /* Coil Address Lo */
    Constructed_ResponseFrame[4] = RequestFrame[4]; /* Force Data Hi */
    Constructed_ResponseFrame[5] = RequestFrame[5]; /* Force Data Lo */
    Constructed_ResponseFrame[6] = RequestFrame[6]; /* CRC Hi */
    Constructed_ResponseFrame[7] = RequestFrame[7]; /* CRC Lo */

    return 8;
}

/**
 * @brief
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Preset_Single_Register_Operation(unsigned char *RequestFrame,
                                                            unsigned char *Constructed_ResponseFrame)
{
    int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    int Preset_Data = RequestFrame[4] << 8 | RequestFrame[5];

    /* Write */
    HOLDING_REGISTERS_MEM[Start_address] = Preset_Data;

    /* The normal response is an echo of the query, returned after the register contents
        have been preset. */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Addrress */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */
    Constructed_ResponseFrame[2] = RequestFrame[2]; /* Register Address Hi */
    Constructed_ResponseFrame[3] = RequestFrame[3]; /* Register Address Lo */
    Constructed_ResponseFrame[4] = RequestFrame[4]; /* Preset Data Hi */
    Constructed_ResponseFrame[5] = RequestFrame[5]; /* Preset Data Lo */
    Constructed_ResponseFrame[6] = RequestFrame[6]; /* CRC Hi */
    Constructed_ResponseFrame[7] = RequestFrame[7]; /* CRC Lo */

    return 8;
}
/**
 * @brief Reads the contents of eight Exception Status coils within the slave controller.
 * @note The function provides a simple method for accessing this information, because the
 * Exception Coil references are known (no coil reference is needed in the function).
 * The predefined Exception Coil assignments are:
 * Controller Model 		Coil 			Assignment
 * M84, 184/384, 584,984 	1 – 8 		    User defined
 * 484 					    257 			Battery Status
 *					        258 – 264 		User defined
 * 884 					    761 			Battery Status
 *					        762 			Memory Protect Status
 *					        763 			RIO Health Status
 *					        764–768 		User defined
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Read_Exception_Status_Operation(unsigned char *RequestFrame,
                                                           unsigned char *Constructed_ResponseFrame)
{

    return 0; // 6;
}

/**
 * @brief Returns a status word and an event count from the slave’s communications event
 * counter. By fetching the current count before and after a series of messages, a
 * master can determine whether the messages were handled normally by the slave.
 * Example:
 * example, the status word is FF FF hex, indicating that a program function is
 * still in progress in the slave. The event count shows that 264 (01 08 hex) events
 * have been counted by the controller.
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Fetch_Comm_Event_Counter_Operation(unsigned char *RequestFrame,
                                                              unsigned char *Constructed_ResponseFrame)
{

    return 0; // 8
}

/**
 * @brief
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Fetch_Comm_Event_Log_Operation(unsigned char *RequestFrame,
                                                          unsigned char *Constructed_ResponseFrame)
{

    return 0; //
}

/**
 * @brief Forces each coil in a sequence of coils to either ON or OFF.
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Force_Multiple_Coils_Operation(unsigned char *RequestFrame,
                                                          unsigned char *Constructed_ResponseFrame)
{
    int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    int Quantity_of_Coils = RequestFrame[4] << 8 | RequestFrame[5];

    unsigned char coil_counter = Start_address;
    int bit = 0;
    unsigned char Byte_Counter = 0;
    unsigned char coil;

    /* Write */
    while (coil_counter < Quantity_of_Coils + Start_address)
    {
        /* 1. get coil value from data frame. */
        coil = (RequestFrame[7 + Byte_Counter] >> bit % 8) & 1;

        /* 2. set finded coli in coil_mem (in bit located) */
        if (coil == 1)
            COIL_MEM[coil_counter / 8] |= (1 << coil_counter % 8);
        else
            COIL_MEM[coil_counter / 8] &= ~(1 << coil_counter % 8);

        /* 2.1 bit 0 - 7 then plus array byte */
        if (bit++ == 7)
        {
            bit = 0;
            Byte_Counter++;
        }

        coil_counter++;
    }

    /*
    The normal response returns the slave address, function code, starting address,
    and quantity of coils forced. */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Addrress */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */
    Constructed_ResponseFrame[2] = RequestFrame[2]; /* Coil Address Hi */
    Constructed_ResponseFrame[3] = RequestFrame[3]; /* Coil Address Lo */
    Constructed_ResponseFrame[4] = RequestFrame[4]; /* Quantity of Coils Hi */
    Constructed_ResponseFrame[5] = RequestFrame[5]; /* Quantity of Coils Lo */

    unsigned short crc = CRC16(Constructed_ResponseFrame, 6);

    Constructed_ResponseFrame[6] = crc >> 8;    /* CRC Lo */
    Constructed_ResponseFrame[7] = crc;         /* CRC Hi */


    return 8; //
}

/**
 * @brief Presets values into a sequence of holding registers. Whenbroadcast, 
 * the function presets the same register references in all attached slaves.
 * 
 * @param RequestFrame 
 * @param Constructed_ResponseFrame 
 * @return return Frame length
 */
static unsigned char SLAVE_Preset_Multiple_Register_Operation(unsigned char *RequestFrame,
                                                          unsigned char *Constructed_ResponseFrame){
    int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    int Number_of_Registers = RequestFrame[4] << 8 | RequestFrame[5];

    int registers_counter = Start_address;
    unsigned char Byte_Counter = 7;

    /* Write */
    while (registers_counter < Number_of_Registers + Start_address)
    {
       HOLDING_REGISTERS_MEM[registers_counter++] =  RequestFrame[Byte_Counter] << 8 | RequestFrame[Byte_Counter++];
    }

    /*
    The normal response returns the slave address, function code, starting address,
    and quantity of registers preset. */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Addrress */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */
    Constructed_ResponseFrame[2] = RequestFrame[2]; /* Coil Address Hi */
    Constructed_ResponseFrame[3] = RequestFrame[3]; /* Coil Address Lo */
    Constructed_ResponseFrame[4] = RequestFrame[4]; /* No. of Registers Hi */
    Constructed_ResponseFrame[5] = RequestFrame[5]; /* No. of Registers Lo */

    unsigned short crc = CRC16(Constructed_ResponseFrame, 6);

    Constructed_ResponseFrame[6] = crc >> 8;    /* CRC Lo */
    Constructed_ResponseFrame[7] = crc;         /* CRC Hi */


    return 8;
}