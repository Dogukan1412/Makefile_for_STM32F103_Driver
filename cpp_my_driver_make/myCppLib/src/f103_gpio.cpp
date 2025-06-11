/*
 * f103_gpio.cpp
 *
 *  Created on: Jun 3, 2025
 *      Author: Dogukan AKCA
 */


#include "../inc/f103_gpio.hpp"

GPIO::GPIO(GPIO_TypeDef* port, uint8_t pin, Mode mode)
    : port_(port), pin_(pin), mode_(mode)
{
	Set_Mode();
}


void GPIO::Enable_Port_Clock()
{
    if (port_ == GPIOA)
    	RCC->APB2ENR |= (1 << 2);					// PORT A clock enabled;
    else if (port_ == GPIOB)
    	RCC->APB2ENR |= (1 << 3);					// PORT B clock enabled;
    else if (port_ == GPIOC)
    	RCC->APB2ENR |= (1 << 4);					// PORT C clock enabled;
    else if (port_ == GPIOD)
        RCC->APB2ENR |= (1 << 5);					// PORT D clock enabled;
    else if (port_ == GPIOE)
        RCC->APB2ENR |= (1 << 6);					// PORT E clock enabled;
}


void GPIO::Set_Mode()
{
	Enable_Port_Clock();

    volatile uint32_t* configReg;
    uint8_t shift;

    if (pin_ < 8)
    {
        configReg = &port_->CRL;
        shift = pin_ * 4;
    }
    else if (pin_ >= 8 && pin_ < 16)
    {
        configReg = &port_->CRH;
        shift = (pin_ - 8) * 4;
    }
    else
    {
		NULL;
	}

    *configReg &= ~(0xF << shift);
    *configReg |= (mode_ << shift);
}


void GPIO::Set()
{
    port_->BSRR = (1 << pin_);
}


void GPIO::Reset()
{
    port_->BRR = (1 << pin_);
}


void GPIO::Toggle()
{
	port_->ODR ^= (1 << pin_);
}


bool GPIO::Read() const
{
    return ((port_->IDR & (1 << pin_)) != 0);
}

