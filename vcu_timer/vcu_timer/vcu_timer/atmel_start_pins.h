/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef ATMEL_START_PINS_H_INCLUDED
#define ATMEL_START_PINS_H_INCLUDED

#include <hal_gpio.h>

// SAME70 has 4 pin functions

#define GPIO_PIN_FUNCTION_A 0
#define GPIO_PIN_FUNCTION_B 1
#define GPIO_PIN_FUNCTION_C 2
#define GPIO_PIN_FUNCTION_D 3

#define PA5 GPIO(GPIO_PORTA, 5)
#define PA6 GPIO(GPIO_PORTA, 6)
#define PA19 GPIO(GPIO_PORTA, 19)
#define PB0 GPIO(GPIO_PORTB, 0)
#define PB1 GPIO(GPIO_PORTB, 1)
#define PB2 GPIO(GPIO_PORTB, 2)
#define PB3 GPIO(GPIO_PORTB, 3)
#define PC12 GPIO(GPIO_PORTC, 12)
#define PC13 GPIO(GPIO_PORTC, 13)
#define ASS_PIN_RELAY GPIO(GPIO_PORTC, 19)
#define PC30 GPIO(GPIO_PORTC, 30)
#define PC31 GPIO(GPIO_PORTC, 31)
#define TS_INPUT GPIO(GPIO_PORTD, 20)
#define PD25 GPIO(GPIO_PORTD, 25)
#define PD26 GPIO(GPIO_PORTD, 26)
#define PD30 GPIO(GPIO_PORTD, 30)

#endif // ATMEL_START_PINS_H_INCLUDED
