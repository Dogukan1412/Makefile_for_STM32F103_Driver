/*
 * f103_usart.cpp
 *
 *  Created on: Jun 2, 2025
 *      Author: Dogukan AKCA
 */


/* Includes ------------------------------------------------------------------*/
#include "../inc/f103_usart.hpp"

/*
 * Constructor of UART CLASS
 */
UART::UART(USART_TypeDef *huart, uint32_t baudrate_u32, UART_Parity parity_e,
		UART_WordLength wordLength_e, UART_StopBits stopBits_e,
		UART_Mode mode_e, UART_Hw_Flow_Control hw_control)
    : _huart(huart), _baudrate_u32(baudrate_u32), _parity_e(parity_e),
	  _wordLength_e(wordLength_e), _stopBits_e(stopBits_e),
	  _mode_e(mode_e), _hw_control(hw_control)
{
    UART_Init();
}


/**
  * @brief  Enables the USARTx's Clock.
  * @param  huart: Select the USART.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3.
  * @retval None
  */
void UART::Set_Clock()
{
	if (_huart == USART1)
    	RCC->APB2ENR |= (1 << 14);									// USART1 clock enabled
    else if (_huart == USART2)
    	RCC->APB1ENR |= (1 << 17);									// USART2 clock enabled
    else if (_huart == USART3)
    	RCC->APB1ENR |= (1 << 18);									// USART3 clock enabled
}


/**
  * @brief  Initializes the USARTx GPIO pins.
  * @param  huart: Select the USART.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3.
  * @retval None
  */
void UART::Set_GPIO()
{
	RCC->APB2ENR |= (1 << 0);									// Alternate Function IO clock enabled

    if (_huart == USART1)
    {
    	/* USART1 GPIO Configuration
    		PA9      ------> USART1_TX
    		PA10     ------> USART1_RX
    	*/
    	RCC->APB2ENR |= (1 << 2);								// PORT A clock enabled (for USART1 pins)

    	GPIOA->CRH &= ~(0xF << 4);								// Clear bits
    	GPIOA->CRH |= (0b1011 << 4);							// Output mode, max speed 50 MHz | Alternate function output Push-pull for PA9

    	GPIOA->CRH &= ~(0xF << 8);								// Clear bits
    	GPIOA->CRH |= (0b0100 << 8);							// No pull | Floating input for PA10

    }
    else if (_huart == USART2)
    {
    	/* USART2 GPIO Configuration
    		PA2      ------> USART2_TX
    		PA3      ------> USART2_RX
    	 */
    	RCC->APB2ENR |= (1 << 2);

    	GPIOA->CRL &= ~((0xF << 8) | (0xF << 12));
    	GPIOA->CRL |=  ((0b1011 << 8) | (0b0100 << 12));
    }
    else if (_huart == USART3)
    {
    	/* USART3 GPIO Configuration
    		PB10      ------> USART3_TX
    		PB11      ------> USART3_RX
    	*/
    	RCC->APB2ENR |= (1 << 3);

    	GPIOB->CRH &= ~((0xF << 8) | (0xF << 12));
    	GPIOB->CRH |=  ((0b1011 << 8) | (0b0100 << 12));
    }
}


/**
  * @brief  Initializes the USARTx peripheral according to the specified
  *         parameters in the huart .
  * @param  huart: Select the USART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3.
  * @retval None
  */
void UART::UART_Init()
{
    Set_Clock();
    Set_GPIO();

    uint32_t pclk_u32 = (_huart == USART1) ? SYSTEM_CORE_CLOCK : (SYSTEM_CORE_CLOCK / 2);
    USART_SetBaudRate(pclk_u32);

    _huart->CR1 = 0;

    // Parity
    if (_parity_e == Even)
        _huart->CR1 |= (1 << 10) | (0 << 9);
    else if (_parity_e == Odd)
        _huart->CR1 |= (1 << 10) | (1 << 9);
    else if (_parity_e == None)
    	_huart->CR1 |= (0 << 10);

    // Mode
    if (_mode_e == TX)
    	_huart->CR1 |= (1 << 3);
    else if (_mode_e == RX)
    	_huart->CR1 |= (1 << 2);
    else if (_mode_e == TX_RX)
    	_huart->CR1 |= (1 << 3) | (1 << 2);

    // Stop bits
    _huart->CR2 &= ~(3 << 12);
    if (_stopBits_e == Stop_Bits_2)
        _huart->CR2 |= (2 << 12);
    else if (_stopBits_e == Stop_Bits_1)
    	_huart->CR2 |= (0 << 12);
    else if (_stopBits_e == Stop_Bits_1_5)
    	_huart->CR2 |= (3 << 12);
    else if (_stopBits_e == Stop_Bits_0_5)
    	_huart->CR2 |= (1 << 12);

    // Word Length
    if(_wordLength_e == Bits_8)
    	_huart->CR1 |= (0 << 12);
    else if (_wordLength_e == Bits_9)
    	_huart->CR1 |= (1 << 12);

    // Hardware Flow Control
    if(_hw_control == Hw_None)
    	_huart->CR3 |= (0b00 << 8);
    else if (_hw_control == Hw_RTS_CTS)
    	_huart->CR3 |= (0b11 << 8);
    else if (_hw_control == Hw_RTS)
        _huart->CR3 |= (1 << 8);
    else if (_hw_control == Hw_CTS)
        _huart->CR3 |= (1 << 9);

    _huart->CR1 |= (1 << 13);							// USART Enabled
}


/**
  * @brief  Sets Baudrate of USARTx.
  * @param  huart: Select the USART.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3.
  *   @param  pclk_u32: Clock bus value of USARTx.
  *   @param  baudrate_u32: Baudrate value.
  * @retval None
  */
void UART::USART_SetBaudRate(uint32_t pclk_u32)
{
	/*
	* Tx/Rx baud = Fck / (16 * USARTDIV)
	* Fck -> Input clock to the peripheral (PCLK1 for USART2, 3, 4, 5 or PCLK2 for USART1)
	* */
    float usartdiv_f = (float)pclk_u32 / (16 * _baudrate_u32);
    uint32_t mantissa_u32 = (uint32_t)usartdiv_f;
    uint32_t fraction_u32 = (uint32_t)((usartdiv_f - mantissa_u32) * 16);
    _huart->BRR = (mantissa_u32 << 4) | (fraction_u32 & 0xF);
}


/**
  * @brief  Sends an amount of data in blocking mode.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the sent data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 provided through pData.
  * @param  data_p Pointer to data buffer (u8 or u16 data elements).
  * @param  dataSize_u16  Amount of data elements (u8 or u16) to be sent
  */
void UART::USART_Transmit_Data(uint8_t *data_p, uint16_t dataSize_u16)
{
	uint16_t *data16bits_p;

	/* In case of 9bits/No Parity transfer, pData needs to be handled as a uint16_t pointer */
	if((_wordLength_e == Bits_9) && (_parity_e == None))
	{
		data16bits_p = (uint16_t *)data_p;
	}
	else
	{
		data16bits_p = nullptr;
	}

	while (dataSize_u16 > 0)
	{
		while (!(_huart->SR & (1 << 7)));					// Wait for transmit data register empty

		if(data16bits_p == nullptr)
		{
			_huart->DR = (uint8_t)((*data_p) & (uint8_t)(0xFF));
			data_p++;
			dataSize_u16--;
		}
		else
		{
			_huart->DR = (uint16_t)((*data16bits_p) & (uint16_t)(0x01FF));
			data16bits_p++;
			dataSize_u16 -= 2;
		}
	}
	while (!(_huart->SR & (1 << 6)));						// Wait for transmit complete
}


/**
  * @brief  Receives an amount of data in blocking mode.
  * @note   When UART parity is not enabled (PCE = 0), and Word Length is configured to 9 bits (M1-M0 = 01),
  *         the received data is handled as a set of u16. In this case, Size must indicate the number
  *         of u16 available through pData.
  * @param  data_p Pointer to data buffer (u8 or u16 data elements).
  * @param  dataSize_u16  Amount of data elements (u8 or u16) to be received.
  */
void UART::USART_Receive_Data(uint8_t *data_p, uint16_t dataSize_u16)
{
	uint16_t *data16bits_p;

    /* In case of 9bits/No Parity transfer, pRxData needs to be handled as a uint16_t pointer */
    if ((_wordLength_e == Bits_9) && (_parity_e == None))
    {
      data16bits_p = (uint16_t *) data_p;
    }
    else
    {
    	data16bits_p = nullptr;
    }

    while (dataSize_u16 > 0)
    {
    	//while (!(USART_Get_Flag_Status_fs(huart, USART_FLAG_RXNE_D)));
    	while (!(_huart->SR & (1 << 5)))					// Wait for Read data register not empty
    	{

    	}

		if(data16bits_p == nullptr)
		{
			if ((_wordLength_e == Bits_9) || ((_wordLength_e == Bits_8) && (_parity_e == None)))
			{
				*data_p = (uint8_t)(_huart->DR & (uint8_t)0x00FF);
			}
			else
			{
				*data_p = (uint8_t)(_huart->DR & (uint8_t)0x007F);
			}
			data_p++;
			dataSize_u16--;
		}
		else
		{
			*data16bits_p = (uint16_t)(_huart->DR & 0x01FF);
			data16bits_p++;
			dataSize_u16 -= 2;
		}
    }
}


/**
  * @brief  Initializes interrupt of the USARTx peripheral according to the specified
  *         parameters in the huart .
  * @param  huart: Select the USART peripheral.
  *   This parameter can be one of the following values:
  *   USART1, USART2, USART3.
  * @param  typeOfInterrupt: specifies the interrupt to select.
  *   This parameter can be one of the following values:
  *     @arg TXE_Interrupt:
  *     @arg RXNE_Interrupt:
  *     @arg TXE_RXNE_Interrupt:
  *     @arg TC_Interrupt:
  * @retval None
  */
void UART::USART_IT_Config_v(UART_Interrupt typeOfInterrupt_e)
{
	if(typeOfInterrupt_e == RXNE_Interrupt)
		_huart->CR1 |= (1 << 5);							// RXNE Interrupt Enable
	else if(typeOfInterrupt_e == TXE_Interrupt)
		_huart->CR1 |= (1 << 7);							// TXE Interrupt Enable
	else if(typeOfInterrupt_e == TXE_RXNE_Interrupt)
		_huart->CR1 |= (1 << 5) | (1 << 7);					// TXE, RXNE Interrupt Enable
	else if(typeOfInterrupt_e == TC_Interrupt)
		_huart->CR1 |= (1 << 6);							// Tranmission Complete Interrupt Enable

	// NVIC interrupt enable
	if (_huart == USART1)
		NVIC->ISER[1] = (1 << 5);
	else if (_huart == USART2)
		NVIC->ISER[1] = (1 << 6);
	else if (_huart == USART3)
		NVIC->ISER[1] = (1 << 7);
}
