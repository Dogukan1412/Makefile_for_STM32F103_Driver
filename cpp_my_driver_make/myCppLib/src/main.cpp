#include "../inc/f103_usart.hpp"
#include "../inc/ring_buffer.hpp"
#include "../inc/f103_gpio.hpp"

extern "C" void _init(void) {}		// During make i get _init error because of .ld file. 
									// To avoid error i used extern "C" void _init(void){}


// GNU++ 11

void RCC_Config_v(void);

char message[64];
const char TERMINATOR = '\n';

volatile uint8_t messageReady = 0;
volatile uint8_t messageIndex = 0;

UART UartDeneme(USART1);
RingBuffer<uint8_t, 64> ringBuffer;

int main(void)
{
	RCC_Config_v();

	GPIO LedUser(GPIOC, 13, GPIO::GPIO_OUTPUT_2MHZ_PP);

	UartDeneme.USART_IT_Config_v(UART::RXNE_Interrupt);

	while (1)
	{
		LedUser.Toggle();

		if (messageReady || (ringBuffer.is_empty() != 1))
		{
		    messageIndex = 0;
		    uint8_t c;

		    while (ringBuffer.get(c) && messageIndex < sizeof(message))
		    {
		        message[messageIndex++] = c;
		        if (c == TERMINATOR) break;
		    }

		    UartDeneme.USART_Transmit_Data((uint8_t*)message, messageIndex);
		    messageReady = 0;
		}

		for(int i = 0; i < (7200000); i++);
	}
}


void RCC_Config_v(void)								// 72 MHz
{
    // HSE ON
    RCC->CR |= (1 << 16);
    while (!(RCC->CR & (1 << 17)));  				// HSE ready

    // AHB prescaler = 1 (no division)
    RCC->CFGR &= ~(0xF << 4);        				// Bits 7:4 = 0b0000

    // APB1 prescaler = /2 (bit 10:8 = 0b100)
    RCC->CFGR &= ~(0x7 << 8);        				// Clear
    RCC->CFGR |=  (0x4 << 8);        				// 0b100 = divide by 2

    // APB2 prescaler = /1
    RCC->CFGR &= ~(0x7 << 11);       				// 0b000 = divide by 1

    // PLL config: source = HSE, mul = x9
    RCC->CFGR &= ~(1 << 16);         				// PLLSRC = 0 (clear first)
    RCC->CFGR |=  (1 << 16);         				// PLL source = HSE
    RCC->CFGR &= ~(0xF << 18);       				// PLLMUL bits
    RCC->CFGR |=  (0x7 << 18);       				// PLLMUL = x9

    // Enable PLL
    RCC->CR |= (1 << 24);
    while (!(RCC->CR & (1 << 25)));  				// Wait for PLL ready

    // Select PLL as system clock
    RCC->CFGR &= ~(0x3 << 0);        				// Clear SW bits
    RCC->CFGR |=  (0x2 << 0);        				// Select PLL
    while ((RCC->CFGR & (0x3 << 2)) != (0x2 << 2)); // Wait till PLL is used

    // (Optional) Flash latency (2 wait state for 72 MHz)
    FLASH->ACR |= (0x2 << 0); 						// 2 WS
}


/*
 * The "weak" reference just means that the routine will be replaced
 * by a routine in your code of the same name. When using C this is
 * simple, the names will always be identical but C++ name mangles the
 * functions (for function overloading etc) so the compiled name will
 * probably not match the default ISR name. You need to wrap the function
 * (or at least a forward reference, I'm not sure of specifics I mostly work in C)
 * in an extern "C" wrapper to force the compiler to not mangle the name.
*/
extern "C" void USART1_IRQHandler(void)			// For echo
{
	if (USART1->SR & (1 << 5))					// RXNE Read data register not empty
	{
		uint8_t c = (uint8_t)(USART1->DR & 0xFF);

		ringBuffer.put(c);

		if (c == TERMINATOR)
		{
			messageReady = 1;
		}
	}
}
