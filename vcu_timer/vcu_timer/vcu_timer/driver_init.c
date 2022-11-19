/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_init.h"
#include <hal_init.h>
#include <hpl_pmc.h>
#include <peripheral_clk_config.h>
#include <utils.h>
#include <hpl_tc.h>

/* The channel amount for ADC */
#define ADC_0_CH_AMOUNT 0

/* The buffer size for ADC */

/* The maximal channel number of enabled channels */

/* The channel amount for ADC */
#define ADC_1_CH_AMOUNT 0

/* The buffer size for ADC */

/* The maximal channel number of enabled channels */

/*! The buffer size for USART */
#define UART_MC_1_BUFFER_SIZE 16
/*! The buffer size for USART */
#define UART_MC_2_BUFFER_SIZE 16

struct adc_async_descriptor ADC_0;
#if ADC_0_CH_AMOUNT < 1
/* Avoid compiling errors. */
struct adc_async_channel_descriptor ADC_0_ch[1];
#warning none of ADC channel is enabled, please check
#else
struct adc_async_channel_descriptor ADC_0_ch[ADC_0_CH_AMOUNT];
#endif
struct adc_async_descriptor ADC_1;
#if ADC_1_CH_AMOUNT < 1
/* Avoid compiling errors. */
struct adc_async_channel_descriptor ADC_1_ch[1];
#warning none of ADC channel is enabled, please check
#else
struct adc_async_channel_descriptor ADC_1_ch[ADC_1_CH_AMOUNT];
#endif
struct timer_descriptor       TIMER_0;
struct usart_async_descriptor UART_MC_1;
struct usart_async_descriptor UART_MC_2;
struct can_async_descriptor   CAN_0;

#ifdef ADC_0_CH_MAX
static uint8_t ADC_0_map[ADC_0_CH_MAX + 1];
#endif

#ifdef ADC_1_CH_MAX
static uint8_t ADC_1_map[ADC_1_CH_MAX + 1];
#endif

static uint8_t UART_MC_1_buffer[UART_MC_1_BUFFER_SIZE];
static uint8_t UART_MC_2_buffer[UART_MC_2_BUFFER_SIZE];

struct flash_descriptor FLASH_0;

/**
 * \brief ADC initialization function
 *
 * Enables ADC peripheral, clocks and initializes ADC driver
 */
static void ADC_0_init(void)
{
	_pmc_enable_periph_clock(ID_AFEC0);
#ifdef ADC_0_CH_MAX
	adc_async_init(&ADC_0, AFEC0, ADC_0_map, ADC_0_CH_MAX, ADC_0_CH_AMOUNT, &ADC_0_ch[0], (void *)NULL);
#endif

	gpio_set_pin_function(PD30, GPIO_PIN_FUNCTION_OFF);

	gpio_set_pin_function(PA19, GPIO_PIN_FUNCTION_OFF);

	gpio_set_pin_function(PB0, GPIO_PIN_FUNCTION_OFF);
}

/**
 * \brief ADC initialization function
 *
 * Enables ADC peripheral, clocks and initializes ADC driver
 */
static void ADC_1_init(void)
{
	_pmc_enable_periph_clock(ID_AFEC1);
#ifdef ADC_1_CH_MAX
	adc_async_init(&ADC_1, AFEC1, ADC_1_map, ADC_1_CH_MAX, ADC_1_CH_AMOUNT, &ADC_1_ch[0], (void *)NULL);
#endif

	gpio_set_pin_function(PB1, GPIO_PIN_FUNCTION_OFF);

	gpio_set_pin_function(PC13, GPIO_PIN_FUNCTION_OFF);

	gpio_set_pin_function(PC12, GPIO_PIN_FUNCTION_OFF);

	gpio_set_pin_function(PC30, GPIO_PIN_FUNCTION_OFF);

	gpio_set_pin_function(PC31, GPIO_PIN_FUNCTION_OFF);
}

void FLASH_0_CLOCK_init(void)
{
}

void FLASH_0_init(void)
{
	FLASH_0_CLOCK_init();
	flash_init(&FLASH_0, EFC);
}

void TIMER_0_PORT_init(void)
{
}
/**
 * \brief Timer initialization function
 *
 * Enables Timer peripheral, clocks and initializes Timer driver
 */
static void TIMER_0_init(void)
{
	_pmc_enable_periph_clock(ID_TC0_CHANNEL0);
	TIMER_0_PORT_init();
	timer_init(&TIMER_0, TC0, _tc_get_timer());
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void UART_MC_1_CLOCK_init()
{
	_pmc_enable_periph_clock(ID_UART1);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void UART_MC_1_PORT_init()
{

	gpio_set_pin_function(PA5, MUX_PA5C_UART1_URXD1);

	gpio_set_pin_function(PA6, MUX_PA6C_UART1_UTXD1);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void UART_MC_1_init(void)
{
	UART_MC_1_CLOCK_init();
	usart_async_init(&UART_MC_1, UART1, UART_MC_1_buffer, UART_MC_1_BUFFER_SIZE, _uart_get_usart_async());
	UART_MC_1_PORT_init();
}

/**
 * \brief USART Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void UART_MC_2_CLOCK_init()
{
	_pmc_enable_periph_clock(ID_UART2);
}

/**
 * \brief USART pinmux initialization function
 *
 * Set each required pin to USART functionality
 */
void UART_MC_2_PORT_init()
{

	gpio_set_pin_function(PD25, MUX_PD25C_UART2_URXD2);

	gpio_set_pin_function(PD26, MUX_PD26C_UART2_UTXD2);
}

/**
 * \brief USART initialization function
 *
 * Enables USART peripheral, clocks and initializes USART driver
 */
void UART_MC_2_init(void)
{
	UART_MC_2_CLOCK_init();
	usart_async_init(&UART_MC_2, UART2, UART_MC_2_buffer, UART_MC_2_BUFFER_SIZE, _uart_get_usart_async());
	UART_MC_2_PORT_init();
}

/**
 * \brief MCAN Clock initialization function
 *
 * Enables register interface and peripheral clock
 */
void CAN_0_CLOCK_init()
{
	_pmc_enable_periph_clock(ID_MCAN0);
}

/**
 * \brief MCAN pinmux initialization function
 *
 * Set each required pin to MCAN functionality
 */
void CAN_0_PORT_init(void)
{

	gpio_set_pin_function(PB3, MUX_PB3A_MCAN0_CANRX0);

	gpio_set_pin_function(PB2, MUX_PB2A_MCAN0_CANTX0);
}
/**
 * \brief CAN initialization function
 *
 * Enables CAN peripheral, clocks and initializes CAN driver
 */
void CAN_0_init(void)
{
	CAN_0_CLOCK_init();
	CAN_0_PORT_init();
	can_async_init(&CAN_0, MCAN0);
}

void system_init(void)
{
	init_mcu();

	_pmc_enable_periph_clock(ID_PIOC);

	_pmc_enable_periph_clock(ID_PIOD);

	/* Disable Watchdog */
	hri_wdt_set_MR_WDDIS_bit(WDT);

	/* GPIO on PC19 */

	gpio_set_pin_level(ASS_PIN_RELAY,
	                   // <y> Initial level
	                   // <id> pad_initial_level
	                   // <false"> Low
	                   // <true"> High
	                   false);

	// Set pin direction to output
	gpio_set_pin_direction(ASS_PIN_RELAY, GPIO_DIRECTION_OUT);

	gpio_set_pin_function(ASS_PIN_RELAY, GPIO_PIN_FUNCTION_OFF);

	/* GPIO on PD20 */

	// Set pin direction to input
	gpio_set_pin_direction(TS_INPUT, GPIO_DIRECTION_IN);

	gpio_set_pin_pull_mode(TS_INPUT,
	                       // <y> Pull configuration
	                       // <id> pad_pull_config
	                       // <GPIO_PULL_OFF"> Off
	                       // <GPIO_PULL_UP"> Pull-up
	                       // <GPIO_PULL_DOWN"> Pull-down
	                       GPIO_PULL_UP);

	gpio_set_pin_function(TS_INPUT, GPIO_PIN_FUNCTION_OFF);

	ADC_0_init();
	ADC_1_init();

	FLASH_0_init();

	TIMER_0_init();
	UART_MC_1_init();
	UART_MC_2_init();

	CAN_0_init();
}
