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
#include "modbus_handler.h"

static unsigned char SLAVE_ADDRESS;

unsigned char frame_buffer[MAX_BUFFER];
unsigned char response_buffer[MAX_BUFFER];

/* Memory map for COIL, INPUT, HOLDING_REGISTERS, INPUT_REGISTERS */

#if MAX_COIL > 0
static unsigned char COIL_MEM[MAX_COIL / 8];
#endif

#if MAX_INPUT > 0
static unsigned char INPUT_MEM[MAX_INPUT / 8];
#endif

#if MAX_HOLDING_REGISTERS > 0
static uint16_t HOLDING_REGISTERS_MEM[MAX_HOLDING_REGISTERS];
#endif

#if MAX_INPUT_REGISTERS > 0
static uint16_t INPUT_REGISTERS_MEM[MAX_INPUT_REGISTERS];
#endif

/* private function's */

#if MAX_COIL > 0
static unsigned char SLAVE_Read_Coil_Status_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
#endif
#if MAX_INPUT > 0
static unsigned char SLAVE_Read_Input_Status_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
#endif
#if MAX_HOLDING_REGISTERS > 0
static unsigned char SLAVE_Read_Holding_Registers_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
#endif
#if MAX_INPUT_REGISTERS > 0
static unsigned char SLAVE_Read_Input_Registers_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
#endif
#if MAX_COIL > 0
static unsigned char SLAVE_Force_Single_Coil_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
#endif
#if MAX_HOLDING_REGISTERS > 0
static unsigned char SLAVE_Preset_Single_Register_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
#endif
#if MAX_COIL > 0
static unsigned char SLAVE_Force_Multiple_Coils_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
#endif
#if MAX_HOLDING_REGISTERS > 0
static unsigned char SLAVE_Preset_Multiple_Register_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
#endif

/* */
unsigned char SLAVE_Read_Exception_Status_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
unsigned char SLAVE_Fetch_Comm_Event_Counter_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
unsigned char SLAVE_Fetch_Comm_Event_Log_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);

unsigned char SLAVE_Report_Slave_ID_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
unsigned char SLAVE_Read_General_Reference_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
unsigned char SLAVE_Write_General_Reference_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
unsigned char SLAVE_Mask_Write_4X_Register_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
unsigned char SLAVE_Read_Write_4X_Registers_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
unsigned char SLAVE_Read_FIFO_Queue_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame);
/* */

/**
 * @brief UART initialization
 *
 * @param serial UART parameter
 */
void modbus_serial_init(Serial_t *serial)
{
    modbus_uart_init_Handler(serial);
}

/**
 * @brief
 * * @param slave_ID
 */
void set_slave_ID(unsigned char slave_ID)
{
    SLAVE_ADDRESS = slave_ID;
}

/**
 * @brief
 * @return SLAVE_ADDRESS
 */
unsigned char get_slave_ID()
{
    return SLAVE_ADDRESS;
}

/**
 * @brief Get coil status from coil array COIL_MEM[].
 *
 * @param coil
 * @return Return coil status from coil array COIL_MEM[].
 */
unsigned char Get_coil_status(int coil)
{
    coil--;
    return (COIL_MEM[coil / 8] >> coil & 1);
}

/**
 * @brief Set coil status into coil array COIL_MEM[].
 *
 * @param coil
 * @param coil status
 * @return Return coil status from coil array COIL_MEM[].
 */
void Set_coil_status(int coil, unsigned int status)
{
    coil--;
    if (status == 1)
        COIL_MEM[(coil / 8)] |= (1 << coil % 8);
    else
        COIL_MEM[(coil / 8)] &= ~(1 << coil % 8);
}
/**
 * @brief Get holding register from array HOLDING_REGISTERS_MEM[].
 *
 * @param holding register
 * @return Return holding register from array HOLDING_REGISTERS_MEM[].
 */
unsigned int Get_holding_register(int Holding_Register_Address)
{
    Holding_Register_Address--;
    return HOLDING_REGISTERS_MEM[Holding_Register_Address];
}

/**
 * @brief Set holding register into array HOLDING_REGISTERS_MEM[].
 *
 * @param holding register
 * @param holding register value
 */
void Set_holding_register(int Holding_Register_Address, unsigned int value)
{
    Holding_Register_Address--;
    HOLDING_REGISTERS_MEM[Holding_Register_Address] = value;
}

/**
 * @brief This function is to prepare the response to the master and perform the commands
 * (write/read on the coil and register, etc.) according to the function code.
 *
 * @param RequestFrame
 * @param ResponseFrame
 * @return Return length Of Response Frame (without CRC OR LRC in serial mode)
 */
unsigned char MODBUS_FARME_PROCESS(unsigned char *RequestFrame,
                                   unsigned char *ResponseFrame)
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
#if MAX_COIL > 0
            lengthOfResponseFrame = SLAVE_Read_Coil_Status_Operation(RequestFrame,
                                                                     ResponseFrame);
#else
        lengthOfResponseFrame = Modbus_Exception(ILLEGAL_DATA_ADDRESS, ResponseFrame);
#endif
        }
        else if (function == Read_Input_Status)
        {
#if MAX_INPUT > 0
            lengthOfResponseFrame = SLAVE_Read_Input_Status_Operation(RequestFrame,
                                                                      ResponseFrame);
#else
        lengthOfResponseFrame = Modbus_Exception(ILLEGAL_DATA_ADDRESS, ResponseFrame);
#endif
        }
        else if (function == Read_Holding_Registers)
        {
#if MAX_HOLDING_REGISTERS > 0
            lengthOfResponseFrame = SLAVE_Read_Holding_Registers_Operation(
                RequestFrame, ResponseFrame);
#else
        lengthOfResponseFrame = Modbus_Exception(ILLEGAL_DATA_ADDRESS, ResponseFrame);
#endif
        }
        else if (function == Read_Input_Registers)
        {
#if MAX_INPUT_REGISTERS > 0
            lengthOfResponseFrame = SLAVE_Read_Input_Registers_Operation(
                RequestFrame, ResponseFrame);
#else
        lengthOfResponseFrame = Modbus_Exception(ILLEGAL_DATA_ADDRESS, ResponseFrame);
#endif
        }

#ifdef WITHOUT_EEPROM
    }
    else if (Run_for_first_time == First_time)
    {
        Run_for_first_time = !First_time;
#endif

        else if (function == Force_Single_Coil)
        {
#if MAX_COIL > 0
            lengthOfResponseFrame = SLAVE_Force_Single_Coil_Operation(RequestFrame,
                                                                      ResponseFrame);
#else
        lengthOfResponseFrame = Modbus_Exception(ILLEGAL_DATA_ADDRESS, ResponseFrame);
#endif
        }
        else if (function == Preset_Single_Register)
        {
#if MAX_HOLDING_REGISTERS > 0
            lengthOfResponseFrame = SLAVE_Preset_Single_Register_Operation(
                RequestFrame, ResponseFrame);
#else
        lengthOfResponseFrame = Modbus_Exception(ILLEGAL_DATA_ADDRESS, ResponseFrame);
#endif
        }
        else if (function == Force_Multiple_Coils)
        {
#if MAX_COIL > 0
            lengthOfResponseFrame = SLAVE_Force_Multiple_Coils_Operation(
                RequestFrame, ResponseFrame);
#else
        lengthOfResponseFrame = Modbus_Exception(ILLEGAL_DATA_ADDRESS, ResponseFrame);
#endif
        }
        else if (function == Preset_Multiple_Registers)
        {
#if MAX_HOLDING_REGISTERS > 0
            lengthOfResponseFrame = SLAVE_Preset_Multiple_Register_Operation(
                RequestFrame, ResponseFrame);
#else
        lengthOfResponseFrame = Modbus_Exception(ILLEGAL_DATA_ADDRESS, ResponseFrame);
#endif
        }
        else
        {
            /* Function Not Supported */
            lengthOfResponseFrame = Modbus_Exception(ILLEGAL_FUNCTION, ResponseFrame);
        }

#ifdef WITHOUT_EEPROM
    }
#endif

    return lengthOfResponseFrame;
}

#if MAX_COIL > 0
/**
 * @brief Reads the ON/OFF status of discrete outputs in the slave.
 * maximum parameters supported by various controller models:
 * 184/384:[800 coil]   484:[512 coil]     584/984/884:[2000 coil]    M84:[64 coil]
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Read_Coil_Status_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{
    unsigned int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    unsigned int Num_of_coil = RequestFrame[4] << 8 | RequestFrame[5];
    unsigned int Start_Coil = Start_address;

    if (Start_address + Num_of_coil > MAX_COIL)
        return Modbus_Exception(ILLEGAL_DATA_ADDRESS, Constructed_ResponseFrame);

    /* Constructing the response frame to the master */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Address */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */

    unsigned int Byte_Count = 0; /*< [unsigned char] For 2000 coil, Maximum byte: 250 + 5 = 255 */
    unsigned int array_byte = 3; /*< [unsigned char] For 2000 coil, Maximum byte: 250 + 3 = 253 */

    unsigned int coil_counter = Start_Coil;
    unsigned char coil = 0;
    char bit = 0;
    int Coil_len = Num_of_coil;

    while (Coil_len--)
    {
        if (bit == 0)
            Byte_Count++;

        /* 1. Get coil value from COIL_MEM. note: coils 1–16 are addressed as 0–15 (coil_counter % 8) */
        coil = (COIL_MEM[(coil_counter / 8)] >> coil_counter % 8) & 1;

        /* 2. Set coil in response (in bit located) */
        if (coil == 1)
            Constructed_ResponseFrame[array_byte] |= (1 << bit);
        else
            Constructed_ResponseFrame[array_byte] &= ~(1 << bit);

        /* 2.1 bit 0 - 7 then plus array byte */
        if (bit++ == 7)
        {
            bit = 0;
            array_byte++;
        }

        coil_counter++;
    }

    Constructed_ResponseFrame[2] = Byte_Count; /* Byte Count */

    return 3 + Byte_Count;
}
#endif

#if MAX_INPUT > 0
/**
 * @brief Reads the ON/OFF status of discrete inputs in the slave.
 * maximum parameters supported by various controller models:
 * 184/384:[800 coil]   484:[512 coil]     584/984/884:[2000 coil]    M84:[64 coil]
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Read_Input_Status_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{
    unsigned int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    unsigned int Num_of_Input = RequestFrame[4] << 8 | RequestFrame[5];
    unsigned int Start_Input = Start_address;

    if (Start_address + Num_of_Input > MAX_INPUT)
        return Modbus_Exception(ILLEGAL_DATA_ADDRESS, Constructed_ResponseFrame);

    /* Constructing the response frame to the master */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Address */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */

    unsigned int Byte_Count = 0; /*< [unsigned char] For 2000 input, Maximum byte: 250 + 5 = 255 */
    unsigned int array_byte = 3; /*< [unsigned char] For 2000 input, Maximum byte: 250 + 3 = 253 */

    unsigned int Input_counter = Start_Input;
    unsigned char Input = 0;
    char bit = 0;
    int Input_len = Num_of_Input;

    while (Input_len--)
    {
        if (bit == 0)
            Byte_Count++;

        /* 1. Get Input value from INPUT_MEM. note: Inputs 1–16 are addressed as 0–15 (Input_counter % 8) */
        Input = (INPUT_MEM[(Input_counter / 8)] >> Input_counter % 8) & 1;

        /* 2. Set input in response (in bit located) */
        if (Input == 1)
            Constructed_ResponseFrame[array_byte] |= (1 << bit);
        else
            Constructed_ResponseFrame[array_byte] &= ~(1 << bit);

        /* 2.1 bit 0 - 7 then plus array byte */
        if (bit++ == 7)
        {
            bit = 0;
            array_byte++;
        }

        Input_counter++;
    }

    Constructed_ResponseFrame[2] = Byte_Count; /* Byte Count */

    return 3 + Byte_Count;
}
#endif

#if MAX_HOLDING_REGISTERS > 0
/**
 * @brief Reads the binary contents of holding registers in the slave.
 * maximum parameters supported by various controller models:
 * 184/384:[100 holding registers]   484:[254 holding registers]
 * 584/984/884:[125 holding registers]    M84:[64 holding registers]
 *
 * @note Note: Data is scanned in the slave at the rate of 125 registers per scan for
 *  984–X8X controllers (984–685, etc), and at the rate of 32 registers per scan for
 * all other controllers. The response is returned when the data is completely assembled
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Read_Holding_Registers_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{
    unsigned int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    unsigned int Num_of_Holding_Registers = RequestFrame[4] << 8 | RequestFrame[5];
    unsigned int Start_Holding_Registers = Start_address;

    if (Start_address + Num_of_Holding_Registers > MAX_HOLDING_REGISTERS)
        return Modbus_Exception(ILLEGAL_DATA_ADDRESS, Constructed_ResponseFrame);

    /* Constructing the response frame to the master */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Address */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */

    unsigned int Byte_Count = Num_of_Holding_Registers * 2; /* Byte Count */

    int Holding_Registers_len = Num_of_Holding_Registers;
    unsigned int Holding_Registers_count = Start_Holding_Registers;
    unsigned int byte = 3;

    while (Holding_Registers_len--)
    {
        Constructed_ResponseFrame[byte++] =
            HOLDING_REGISTERS_MEM[Holding_Registers_count] >> 8; /* Hi Holding Registers */

        Constructed_ResponseFrame[byte++] = 0x00ff & HOLDING_REGISTERS_MEM[Holding_Registers_count]; /* Lo Holding Registers */

        Holding_Registers_count++;
    }

    Constructed_ResponseFrame[2] = Byte_Count; /* Byte Count */

    return 3 + Byte_Count;
}
#endif

#if MAX_INPUT_REGISTERS > 0
/**
 * @brief Reads the binary contents of input registers in the slave.
 * maximum parameters supported by various controller models:
 * 184/384:[100 input registers]   484:[32 input registers]
 * 584/984/884:[125 input registers]    M84:[4 input registers]
 *
 * @note Note:Data is scanned in the slave at the rate of 125 registers per scan for
 *  984–X8X controllers (984–685, etc), and at the rate of 32 registers per scan for
 * all other controllers. The response is returned when the data is completely assembled
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Read_Input_Registers_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{
    unsigned int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    unsigned int Num_of_Input_Registers = RequestFrame[4] << 8 | RequestFrame[5];

    if (Start_address + Num_of_Input_Registers > MAX_INPUT_REGISTERS)
        return Modbus_Exception(ILLEGAL_DATA_ADDRESS, Constructed_ResponseFrame);

    /* Constructing the response frame to the master */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Address */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */

    unsigned int Byte_Count = Num_of_Input_Registers * 2; /* Byte Count */

    int Input_Registers_len = Num_of_Input_Registers;
    unsigned int Input_Registers_count = Start_address;
    unsigned int byte = 3;

    while (Input_Registers_len--)
    {
        Constructed_ResponseFrame[byte++] =
            INPUT_REGISTERS_MEM[Input_Registers_count] >> 8; /* Hi Input Registers */

        Constructed_ResponseFrame[byte++] = 0x00ff & INPUT_REGISTERS_MEM[Input_Registers_count]; /* Lo Input Registers */

        Input_Registers_count++;
    }

    Constructed_ResponseFrame[2] = Byte_Count; /* Byte Count */

    return 3 + Byte_Count;
}
#endif

#if MAX_COIL > 0
/**
 * @brief Forces a single coil to either ON or OFF. When broadcast, the
 * function forces the same coil reference in all attached slaves.
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Force_Single_Coil_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{
    unsigned int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    unsigned int Force_Data = RequestFrame[4] << 8 | RequestFrame[5];

    if (Start_address > MAX_COIL)
        return Modbus_Exception(ILLEGAL_DATA_ADDRESS, Constructed_ResponseFrame);

    /* Write */
    if (Force_Data == 0xff00)
        COIL_MEM[(Start_address / 8)] |= (1 << Start_address % 8);
    else
        COIL_MEM[(Start_address / 8)] &= ~(1 << Start_address % 8);

    /* The normal response is an echo of the query, returned after the coil
     state has been forced. */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Address */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */
    Constructed_ResponseFrame[2] = RequestFrame[2]; /* Coil Address Hi */
    Constructed_ResponseFrame[3] = RequestFrame[3]; /* Coil Address Lo */
    Constructed_ResponseFrame[4] = RequestFrame[4]; /* Force Data Hi */
    Constructed_ResponseFrame[5] = RequestFrame[5]; /* Force Data Lo */

    return 6;
}
#endif

#if MAX_HOLDING_REGISTERS > 0
/**
 * @brief Presets a value into a single holding register. When broadcast, the
 * function presets the same register reference in all attached slaves.
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Preset_Single_Register_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{
    unsigned int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    unsigned int Preset_Data = RequestFrame[4] << 8 | RequestFrame[5];

    if (Start_address > MAX_HOLDING_REGISTERS)
        return Modbus_Exception(ILLEGAL_DATA_ADDRESS, Constructed_ResponseFrame);

    /* Write */
    HOLDING_REGISTERS_MEM[Start_address] = Preset_Data;

    /* The normal response is an echo of the query, returned after the register contents
     have been preset. */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Address */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */
    Constructed_ResponseFrame[2] = RequestFrame[2]; /* Register Address Hi */
    Constructed_ResponseFrame[3] = RequestFrame[3]; /* Register Address Lo */
    Constructed_ResponseFrame[4] = RequestFrame[4]; /* Preset Data Hi */
    Constructed_ResponseFrame[5] = RequestFrame[5]; /* Preset Data Lo */

    return 6;
}
#endif

/**
 * @brief Reads the contents of eight Exception Status coils within the slave controller.
 * @note The function provides a simple method for accessing this information, because the
 * Exception Coil references are known (no coil reference is needed in the function).
 *
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
unsigned char SLAVE_Read_Exception_Status_Operation(unsigned char *RequestFrame,
                                                    unsigned char *Constructed_ResponseFrame)
{

    return 0; // 6;
}

/**
 * @brief Returns a status word and an event count from the slave’s communications event
 * counter. By fetching the current count before and after a series of messages, a
 * master can determine whether the messages were handled normally by the slave.
 * Example:
 * status word is FF FF hex, indicating that a program function is still in progress
 * in the slave. The event count shows that 264 (01 08 hex) events have been counted
 * by the controller.
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
unsigned char SLAVE_Fetch_Comm_Event_Counter_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{

    return 0; // 8
}

/**
 * @brief Returns a status word, event count, message count, and a field of event bytes
 * from the slave. Broadcast is not supported.
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
unsigned char SLAVE_Fetch_Comm_Event_Log_Operation(unsigned char *RequestFrame,
                                                   unsigned char *Constructed_ResponseFrame)
{

    return 0; //
}

#if MAX_COIL > 0
/**
 * @brief Forces each coil in a sequence of coils to either ON or OFF.
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Force_Multiple_Coils_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{
    unsigned int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    unsigned int Quantity_of_Coils = RequestFrame[4] << 8 | RequestFrame[5];

    if (Start_address + Quantity_of_Coils > MAX_COIL)
        return Modbus_Exception(ILLEGAL_DATA_ADDRESS, Constructed_ResponseFrame);

    unsigned int coil_counter = Start_address;
    char bit = 0;
    unsigned int Byte_Counter = 0;
    unsigned char coil;

    /* Write */
    while (coil_counter < Quantity_of_Coils + Start_address)
    {
        /* 1. Get coil value from data frame. */
        coil = (RequestFrame[7 + Byte_Counter] >> bit % 8) & 1;

        /* 2. Set coli's in COIL_MEM (in bit located) */
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

    /* The normal response returns the slave address, function code, starting address,
     and quantity of coils forced. */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Address */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */
    Constructed_ResponseFrame[2] = RequestFrame[2]; /* Coil Address Hi */
    Constructed_ResponseFrame[3] = RequestFrame[3]; /* Coil Address Lo */
    Constructed_ResponseFrame[4] = RequestFrame[4]; /* Quantity of Coils Hi */
    Constructed_ResponseFrame[5] = RequestFrame[5]; /* Quantity of Coils Lo */

    return 6; //
}
#endif

#if MAX_HOLDING_REGISTERS > 0
/**
 * @brief Presets values into a sequence of holding registers. When broadcast,
 * the function presets the same register references in all attached slaves.
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
static unsigned char SLAVE_Preset_Multiple_Register_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{
    unsigned int Start_address = RequestFrame[2] << 8 | RequestFrame[3];
    unsigned int Number_of_Registers = RequestFrame[4] << 8 | RequestFrame[5];

    if (Start_address + Number_of_Registers > MAX_HOLDING_REGISTERS)
        return Modbus_Exception(ILLEGAL_DATA_ADDRESS, Constructed_ResponseFrame);

    unsigned int registers_counter = Start_address;
    unsigned int Byte_Counter = 7;

    /* Write */
    while (registers_counter < Number_of_Registers + Start_address)
    {
        HOLDING_REGISTERS_MEM[registers_counter] = RequestFrame[Byte_Counter++] << 8;
        HOLDING_REGISTERS_MEM[registers_counter] |= RequestFrame[Byte_Counter++];
        registers_counter++;
    }

    /*
     The normal response returns the slave address, function code, starting address,
     and quantity of registers preset. */
    Constructed_ResponseFrame[0] = RequestFrame[0]; /* Slave Address */
    Constructed_ResponseFrame[1] = RequestFrame[1]; /* Function Code */
    Constructed_ResponseFrame[2] = RequestFrame[2]; /* Coil Address Hi */
    Constructed_ResponseFrame[3] = RequestFrame[3]; /* Coil Address Lo */
    Constructed_ResponseFrame[4] = RequestFrame[4]; /* No. of Registers Hi */
    Constructed_ResponseFrame[5] = RequestFrame[5]; /* No. of Registers Lo */

    return 6;
}
#endif

/**
 * @brief
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
unsigned char SLAVE_Report_Slave_ID_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{

    return 0;
}

/**
 * @brief
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
unsigned char SLAVE_Read_General_Reference_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{

    return 0;
}

/**
 * @brief
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
unsigned char SLAVE_Write_General_Reference_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{

    return 0;
}

/**
 * @brief
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
unsigned char SLAVE_Mask_Write_4X_Register_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{

    return 0;
}

/**
 * @brief
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
unsigned char SLAVE_Read_Write_4X_Registers_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{

    return 0;
}

/**
 * @brief Reads the contents of a First–In–First–Out (FIFO) queue of 4XXXX registers.
 *
 * @param RequestFrame
 * @param Constructed_ResponseFrame
 * @return return Frame length
 */
unsigned char SLAVE_Read_FIFO_Queue_Operation(
    unsigned char *RequestFrame, unsigned char *Constructed_ResponseFrame)
{

    return 0;
}

/**
 * @brief
 *
 * @param Modbus_Exception
 * @param ResponseFrame
 * @return return Frame length
 */
unsigned char Modbus_Exception(Modbus_Exception_Code_t Modbus_Exception_Code, unsigned char *ResponseFrame)
{
    ResponseFrame[0] = SLAVE_ADDRESS;         /* Slave Address */
    ResponseFrame[1] = 0x81;                  /* Function */
    ResponseFrame[2] = Modbus_Exception_Code; /* Exception Code */

    return 3;
}

/**
 * @brief
 *
 * @param frame_parameter
 * @param communication_parameter
 * @param transmission_mode
 * @param Tick
 * @return ModbusStatus_t
 */
ModbusStatus_t MODBUS_MASTER_PROCESS(frame_parameter_t *frame_parameter,
                                     communication_parameter_t *communication_parameter,
                                     Serial_Transmission_Modes_t transmission_mode, volatile uint32_t *Tick)
{
    unsigned char (*receive_uart_fun)() = modbus_uart_receive_Handler;
    void (*transmit_uart_fun)(uint8_t * Data,
                              uint16_t length) = modbus_uart_transmit_Handler;
    unsigned char rec_byte;
    ModbusStatus_t res;
    uint32_t tickstart_for_comm_timeout;
    uint32_t currenttick_for_comm_timeout;

    /* 1. make request frame */
    frame_buffer[0] = frame_parameter->slave_ID;             /* Slave Address */
    frame_buffer[1] = frame_parameter->function;             /* Function Code */
    frame_buffer[2] = frame_parameter->start_address >> 8;   /* Address Hi */
    frame_buffer[3] = frame_parameter->start_address & 0xff; /* Address Lo */
    frame_buffer[4] = frame_parameter->quantity >> 8;        /* No. of Registers Hi */
    frame_buffer[5] = frame_parameter->quantity & 0xff;      /* No. of Registers Lo */

    unsigned char fun = frame_parameter->function;
    int len = 6;
    if (fun == Force_Single_Coil || fun == Preset_Single_Register)
    {
    }
    else if (fun == Force_Multiple_Coils)
    {
        frame_buffer[6] = (frame_parameter->quantity / 8) + 1; /* Byte Count */
        len++;

        unsigned int coil_counter = frame_parameter->start_address;
        char bit = 0;
        unsigned int Byte_Counter = 0;
        unsigned char coil;

        /* Write */
        while (coil_counter < frame_parameter->quantity + frame_parameter->start_address)
        {
            /* 1. Get coil value from COIL_MEM. */
            coil = (COIL_MEM[coil_counter / 8] >> bit % 8) & 1;

            /* 2. Set coli's in data frame (in bit located) */
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
        len += Byte_Counter;
    }
    else if (fun == Preset_Multiple_Registers)
    {
        frame_buffer[6] = frame_parameter->quantity / 2; /* Byte Count */
        len++;

        unsigned int registers_counter = frame_parameter->start_address;

        while (registers_counter < frame_parameter->quantity + frame_parameter->start_address)
        {
            frame_buffer[len++] = HOLDING_REGISTERS_MEM[registers_counter] << 8;
            frame_buffer[len++] |= HOLDING_REGISTERS_MEM[registers_counter];
            registers_counter++;
        }
    }

    if (transmission_mode == RTU)
    {
        /* 2. Add CRC to frame */
        unsigned short crc = CRC16(response_buffer, len);
        frame_buffer[len] = crc >> 8; /* CRC Lo */
        frame_buffer[len + 1] = crc;  /* CRC Hi */

        len += 2;

        unsigned char retry_Times = communication_parameter->communication_retry_Times;
        set_slave_ID(frame_parameter->slave_ID);
        do
        {
            /* 3. Transmit frame */
            (*transmit_uart_fun)(frame_buffer, len);

            /* 5. */
            ModbusStatus_t res = MODBUS_RTU_MONITOR(response_buffer,
                                                    communication_parameter->communication_timeout,
                                                    Tick, Listen_Only);
            /* 6. */
            if (res == MODBUS_MONITOR_TIMEOUT)
                continue;
            /* 7. */
            else
                return MODBUS_OK;

        } while (retry_Times--); /*< 4. */
    }

    return MODBUS_OK;
}
