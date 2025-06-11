/*
 * f103_usart.hpp
 *
 *  Created on: Jun 2, 2025
 *      Author: Dogukan AKCA
 */

#ifndef INC_F103_USART_HPP_
#define INC_F103_USART_HPP_


/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx.h"

#define SYSTEM_CORE_CLOCK 72000000

#ifndef SYSTEM_CORE_CLOCK
#define SYSTEM_CORE_CLOCK 72000000
#endif

class UART{
public:
	enum UART_Hw_Flow_Control {Hw_None = 0, Hw_RTS, Hw_CTS, Hw_RTS_CTS};
	enum UART_Mode {TX = 0, RX,TX_RX};
	enum UART_StopBits {Stop_Bits_1 = 0, Stop_Bits_2, Stop_Bits_0_5, Stop_Bits_1_5};
	enum UART_WordLength {Bits_8 = 0, Bits_9};
	enum UART_Parity {None = 0, Even,Odd};
	enum UART_Interrupt {TXE_Interrupt = 0, RXNE_Interrupt, TXE_RXNE_Interrupt, TC_Interrupt};

	UART(USART_TypeDef *huart, uint32_t baudrate_u32 = 115200,
			UART_Parity parity_e = None, UART_WordLength wordLength_e = Bits_8,
			UART_StopBits stopBits_e = Stop_Bits_1, UART_Mode mode_e = TX_RX,
			UART_Hw_Flow_Control hw_control = Hw_None);

	void USART_Transmit_Data(uint8_t *data_p, uint16_t dataSize_u16);
	void USART_Receive_Data(uint8_t *data_p, uint16_t dataSize_u16);
	void USART_IT_Config_v(UART_Interrupt typeOfInterrupt_e);

private:
	USART_TypeDef* _huart;
	uint32_t _baudrate_u32;
	UART_Parity _parity_e;
	UART_WordLength _wordLength_e;
	UART_StopBits _stopBits_e;
	UART_Mode _mode_e;
	UART_Hw_Flow_Control _hw_control;

	void UART_Init();
	void Set_Clock();
	void Set_GPIO();
	void USART_SetBaudRate(uint32_t pclk_u32);
};


#endif /* INC_F103_USART_HPP_ */
