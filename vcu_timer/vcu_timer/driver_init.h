/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */
#ifndef DRIVER_INIT_H_INCLUDED
#define DRIVER_INIT_H_INCLUDED

#include "atmel_start_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_atomic.h>
#include <hal_delay.h>
#include <hal_gpio.h>
#include <hal_init.h>
#include <hal_io.h>
#include <hal_sleep.h>

#include <hal_adc_async.h>
#include <hal_adc_async.h>

#include <hal_flash.h>

#include <hal_timer.h>

#include <hal_usart_sync.h>
#include <hpl_uart_base.h>
#include <hal_usart_async.h>
#include <hpl_uart_base.h>
#include <hal_usart_async.h>
#include <hpl_uart_base.h>

#include <hal_can_async.h>

extern struct adc_async_descriptor ADC_0;

/* The enabled channel for ADC */
#define CONF_ADC_0_CHANNEL_0 0

#define CONF_ADC_0_CHANNEL_8 8

#define CONF_ADC_0_CHANNEL_10 10

extern struct adc_async_descriptor ADC_1;

/* The enabled channel for ADC */
#define CONF_ADC_1_CHANNEL_0 0
#define CONF_ADC_1_CHANNEL_1 1

#define CONF_ADC_1_CHANNEL_3 3

#define CONF_ADC_1_CHANNEL_5 5
#define CONF_ADC_1_CHANNEL_6 6

extern struct flash_descriptor FLASH_0;
extern struct timer_descriptor TIMER_0;

extern struct usart_sync_descriptor  TARGET_IO;
extern struct usart_async_descriptor UART_MC_1;
extern struct usart_async_descriptor UART_MC_2;
extern struct can_async_descriptor   CAN_0;

void FLASH_0_init(void);
void FLASH_0_CLOCK_init(void);

void TARGET_IO_PORT_init(void);
void TARGET_IO_CLOCK_init(void);
void TARGET_IO_init(void);
void TARGET_IO_example(void);

void UART_MC_1_PORT_init(void);
void UART_MC_1_CLOCK_init(void);
void UART_MC_1_init(void);
void UART_MC_1_example(void);

void UART_MC_2_PORT_init(void);
void UART_MC_2_CLOCK_init(void);
void UART_MC_2_init(void);
void UART_MC_2_example(void);

void CAN_0_PORT_init(void);
void CAN_0_CLOCK_init(void);
void CAN_0_init(void);
void CAN_0_example(void);

/**
 * \brief Perform system initialization, initialize pins and clocks for
 * peripherals
 */
void system_init(void);

#ifdef __cplusplus
}
#endif
#endif // DRIVER_INIT_H_INCLUDED
