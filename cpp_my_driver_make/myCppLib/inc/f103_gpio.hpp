/*
 * f103_gpio.hpp
 *
 *  Created on: Jun 3, 2025
 *      Author: Dogukan AKCA
 */

#ifndef INC_F103_GPIO_HPP_
#define INC_F103_GPIO_HPP_


#include "stm32f1xx.h"

class GPIO {
public:
    enum Mode {
        GPIO_INPUT_ANALOG  				= 0b0000,
		GPIO_INPUT_FLOATING            	= 0b0100,
		GPIO_INPUT_PP_PD            	= 0b1000,

        GPIO_OUTPUT_2MHZ_PP   			= 0b0010,
		GPIO_OUTPUT_2MHZ_OD				= 0b0110,
		GPIO_OUTPUT_AF_2MHZ_PP			= 0b1010,
		GPIO_OUTPUT_AF_2MHZ_OD			= 0b1110,

		GPIO_OUTPUT_10MHZ_PP   			= 0b0001,
		GPIO_OUTPUT_10MHZ_OD			= 0b0101,
		GPIO_OUTPUT_AF_10MHZ_PP			= 0b1001,
		GPIO_OUTPUT_AF_10MHZ_OD			= 0b1101,

		GPIO_OUTPUT_50MHZ_PP   			= 0b0011,
		GPIO_OUTPUT_50MHZ_OD			= 0b0111,
		GPIO_OUTPUT_AF_50MHZ_PP			= 0b1011,
		GPIO_OUTPUT_AF_50MHZ_OD			= 0b1111
    };

    GPIO(GPIO_TypeDef* port, uint8_t pin, Mode mode);

    void Set();
    void Reset();
    void Toggle();
    bool Read() const;

private:
    GPIO_TypeDef* port_;
    uint8_t pin_;
    Mode mode_;

    void Set_Mode();
    void Enable_Port_Clock();
};


#endif /* INC_F103_GPIO_HPP_ */
