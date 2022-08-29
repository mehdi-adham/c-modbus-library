# c-modbus-library
**Note: This library used for microcontroller**

## How to use the Modbus library (current version)

### Setup memory for
- Coil (if used by the user)
- Input discrete(if used by the user)
- Holding Register (if used by the user)
- Input register (if used by the user)

To do this, in the Modbus library, change the following definitions in this file according to the needs of your program.

```c
/* Modbus memory map for COIL, INPUT, HOLDING_REGISTERS, INPUT_REGISTERS */
#define MAX_COIL      				8 /*< 184/384:[800]   484:[512]     584/984/884:[2000]    M84:[64] */
#define MAX_INPUT      				0 /*< 184/384:[800]   484:[512]     584/984/884:[2000]    M84:[64] */
#define MAX_HOLDING_REGISTERS       		8 /*< 184/384:[100]   484:[254]     584/984/884:[125]    M84:[64] */
#define MAX_INPUT_REGISTERS         		0 /*< 184/384:[100]   484:[32]     584/984/884:[125]    M84:[4] */
```

### Set slave device
```c
set_slave_ID(17);
```

### Serial communication setup
- Port number (Register address of the UART in the microcontroller)
- Baudrate initialization
- Parity
- Stop bit

```c
    Serial_t default_serial;
    default_serial.UART = (uint32_t*) USART1;
    default_serial.BaudRate = 9600;
    default_serial.Parity = NONE_PARITY;
    default_serial.StopBit = StopBit_1;

    modbus_serial_init(&default_serial);
```

### MODBUS RTU MONITOR function
we use the Modbus network monitor function According to the above settings, this function receives the frame related to this device from the serial, processes it and applies commands (reading/writing to registers or coils) and prepares the response or exception response for the master. and sends via serial.

- **param mbus_frame_buffer**: Saves the frame in the user buffer
- **param Tick**: Get a pointer to tick value in millisecond. For setting the timeout to exit the function if the master device  does not send the frame related to this device
- **param Mode**: 1. Normal mode (by responding to the master and applying commands) 2. only listening mode (without responding to the master and without applying commands)
- **return ModbusStatus_t**: Return Modbus Status 

```c
	/* Modbus network monitor to receive frames from the master device */
	ModbusStatus_t res = MODBUS_RTU_MONITOR(buff, 3000, &uwTick, Normal);
```

### How to use Modbus Handler function's (current version)

Copy the Modbus Handler functions from the modbus_handler.h file and use it in your file for implementation.

```c
/**
 * @brief
 * NOTE : This function should not be modified, when the callback is needed,
 the uart_receive Handler could be implemented in the user file
 * @return Modbus Status
 */
__attribute__((weak))	 ModbusStatus_t modbus_uart_receive_Handler(uint8_t *Data) {
	return 0;
}

/**
 * @brief
 * NOTE : This function should not be modified, when the callback is needed,
 the uart transmit Handler could be implemented in the user file
 * @return None
 */
__attribute__((weak)) 	void modbus_uart_transmit_Handler(uint8_t *Data,
		uint16_t length) {

}

/**
 * @brief
 * NOTE : This function should not be modified, when the callback is needed,
 the uart_init Handler could be implemented in the user file
 * @return Modbus Status
 */
__attribute__((weak))	 void modbus_uart_init_Handler(Serial_t *Serial) {

} 
```


### An example of a Modbus Handler implementation for the HAL library (STM32).

```c
ModbusStatus_t modbus_uart_receive_Handler(uint8_t *Data) {
	ModbusStatus_t res;
	res = (ModbusStatus_t) HAL_UART_Receive(&huart1, Data, 1, timeout_3_5C);
	return res;
}

void modbus_uart_transmit_Handler(uint8_t *Data, uint16_t length) {
	HAL_Delay(5);
	HAL_UART_Transmit(&huart1, Data, length, 100);
}

void modbus_uart_init_Handler(Serial_t *Serial) {
	huart1.Instance = (USART_TypeDef*) Serial->UART;
	huart1.Init.BaudRate = Serial->BaudRate;

	if (Serial->StopBit == StopBit_1)
		huart1.Init.StopBits = UART_STOPBITS_1;
	else
		huart1.Init.StopBits = UART_STOPBITS_2;

	if (Serial->Parity == NONE_PARITY)
		huart1.Init.Parity = UART_PARITY_NONE;
	else if (Serial->Parity == ODD_PARITY)
		huart1.Init.Parity = UART_PARITY_ODD;
	else
		huart1.Init.Parity = UART_PARITY_EVEN;

	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;

	HAL_UART_Init(&huart1);
}
```
