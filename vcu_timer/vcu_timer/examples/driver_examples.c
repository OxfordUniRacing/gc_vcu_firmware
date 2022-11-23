/*
 * Code generated from Atmel Start.
 *
 * This file will be overwritten when reconfiguring your Atmel Start project.
 * Please copy examples or other code you want to keep to a separate file
 * to avoid losing it when reconfiguring.
 */

#include "driver_examples.h"
#include "driver_init.h"
#include "utils.h"

static void convert_cb_ADC_0_CHANNEL_0(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
}

static void convert_cb_ADC_0_CHANNEL_8(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
}

static void convert_cb_ADC_0_CHANNEL_10(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
}

/**
 * Example of using ADC_0 to generate waveform.
 */
void ADC_0_example(void)
{
	adc_async_register_callback(&ADC_0, CONF_ADC_0_CHANNEL_0, ADC_ASYNC_CONVERT_CB, convert_cb_ADC_0_CHANNEL_0);
	adc_async_register_callback(&ADC_0, CONF_ADC_0_CHANNEL_8, ADC_ASYNC_CONVERT_CB, convert_cb_ADC_0_CHANNEL_8);
	adc_async_register_callback(&ADC_0, CONF_ADC_0_CHANNEL_10, ADC_ASYNC_CONVERT_CB, convert_cb_ADC_0_CHANNEL_10);
	adc_async_enable_channel(&ADC_0, CONF_ADC_0_CHANNEL_0);
	adc_async_enable_channel(&ADC_0, CONF_ADC_0_CHANNEL_8);
	adc_async_enable_channel(&ADC_0, CONF_ADC_0_CHANNEL_10);
	adc_async_start_conversion(&ADC_0);
}

static void convert_cb_ADC_1_CHANNEL_0(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
}

static void convert_cb_ADC_1_CHANNEL_1(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
}

static void convert_cb_ADC_1_CHANNEL_3(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
}

static void convert_cb_ADC_1_CHANNEL_5(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
}

static void convert_cb_ADC_1_CHANNEL_6(const struct adc_async_descriptor *const descr, const uint8_t channel)
{
}

/**
 * Example of using ADC_1 to generate waveform.
 */
void ADC_1_example(void)
{
	adc_async_register_callback(&ADC_1, CONF_ADC_1_CHANNEL_0, ADC_ASYNC_CONVERT_CB, convert_cb_ADC_1_CHANNEL_0);
	adc_async_register_callback(&ADC_1, CONF_ADC_1_CHANNEL_1, ADC_ASYNC_CONVERT_CB, convert_cb_ADC_1_CHANNEL_1);
	adc_async_register_callback(&ADC_1, CONF_ADC_1_CHANNEL_3, ADC_ASYNC_CONVERT_CB, convert_cb_ADC_1_CHANNEL_3);
	adc_async_register_callback(&ADC_1, CONF_ADC_1_CHANNEL_5, ADC_ASYNC_CONVERT_CB, convert_cb_ADC_1_CHANNEL_5);
	adc_async_register_callback(&ADC_1, CONF_ADC_1_CHANNEL_6, ADC_ASYNC_CONVERT_CB, convert_cb_ADC_1_CHANNEL_6);
	adc_async_enable_channel(&ADC_1, CONF_ADC_1_CHANNEL_0);
	adc_async_enable_channel(&ADC_1, CONF_ADC_1_CHANNEL_1);
	adc_async_enable_channel(&ADC_1, CONF_ADC_1_CHANNEL_3);
	adc_async_enable_channel(&ADC_1, CONF_ADC_1_CHANNEL_5);
	adc_async_enable_channel(&ADC_1, CONF_ADC_1_CHANNEL_6);
	adc_async_start_conversion(&ADC_1);
}

static uint8_t src_data[IFLASH_PAGE_SIZE];
static uint8_t chk_data[IFLASH_PAGE_SIZE];

/**
 * Example of using FLASH_0 to read and write buffer.
 */
void FLASH_0_example(void)
{
	uint32_t page_size;
	uint16_t i;

	/* Init source data */
	page_size = flash_get_page_size(&FLASH_0);

	for (i = 0; i < page_size; i++) {
		src_data[i] = i;
	}

	/* Write data to flash */
	flash_write(&FLASH_0, 0x3200, src_data, page_size);

	/* Read data from flash */
	flash_read(&FLASH_0, 0x3200, chk_data, page_size);
}

/**
 * Example of using TIMER_0.
 */
static struct timer_task TIMER_0_task1, TIMER_0_task2;

static void TIMER_0_task1_cb(const struct timer_task *const timer_task)
{
}

static void TIMER_0_task2_cb(const struct timer_task *const timer_task)
{
}

void TIMER_0_example(void)
{
	TIMER_0_task1.interval = 100;
	TIMER_0_task1.cb       = TIMER_0_task1_cb;
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT;
	TIMER_0_task2.interval = 200;
	TIMER_0_task2.cb       = TIMER_0_task2_cb;
	TIMER_0_task2.mode     = TIMER_TASK_REPEAT;

	timer_add_task(&TIMER_0, &TIMER_0_task1);
	timer_add_task(&TIMER_0, &TIMER_0_task2);
	timer_start(&TIMER_0);
}

/**
 * Example of using UART_MC_1 to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_UART_MC_1[12] = "Hello World!";

static void tx_cb_UART_MC_1(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void UART_MC_1_example(void)
{
	struct io_descriptor *io;

	usart_async_register_callback(&UART_MC_1, USART_ASYNC_TXC_CB, tx_cb_UART_MC_1);
	/*usart_async_register_callback(&UART_MC_1, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&UART_MC_1, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&UART_MC_1, &io);
	usart_async_enable(&UART_MC_1);

	io_write(io, example_UART_MC_1, 12);
}

/**
 * Example of using UART_MC_2 to write "Hello World" using the IO abstraction.
 *
 * Since the driver is asynchronous we need to use statically allocated memory for string
 * because driver initiates transfer and then returns before the transmission is completed.
 *
 * Once transfer has been completed the tx_cb function will be called.
 */

static uint8_t example_UART_MC_2[12] = "Hello World!";

static void tx_cb_UART_MC_2(const struct usart_async_descriptor *const io_descr)
{
	/* Transfer completed */
}

void UART_MC_2_example(void)
{
	struct io_descriptor *io;

	usart_async_register_callback(&UART_MC_2, USART_ASYNC_TXC_CB, tx_cb_UART_MC_2);
	/*usart_async_register_callback(&UART_MC_2, USART_ASYNC_RXC_CB, rx_cb);
	usart_async_register_callback(&UART_MC_2, USART_ASYNC_ERROR_CB, err_cb);*/
	usart_async_get_io_descriptor(&UART_MC_2, &io);
	usart_async_enable(&UART_MC_2);

	io_write(io, example_UART_MC_2, 12);
}

void CAN_0_tx_callback(struct can_async_descriptor *const descr)
{
	(void)descr;
}
void CAN_0_rx_callback(struct can_async_descriptor *const descr)
{
	struct can_message msg;
	uint8_t            data[64];
	msg.data = data;
	can_async_read(descr, &msg);
	return;
}

/**
 * Example of using CAN_0 to Encrypt/Decrypt datas.
 */
void CAN_0_example(void)
{
	struct can_message msg;
	struct can_filter  filter;
	uint8_t            send_data[4];
	send_data[0] = 0x00;
	send_data[1] = 0x01;
	send_data[2] = 0x02;
	send_data[3] = 0x03;

	msg.id   = 0x45A;
	msg.type = CAN_TYPE_DATA;
	msg.data = send_data;
	msg.len  = 4;
	msg.fmt  = CAN_FMT_STDID;
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_0_tx_callback);
	can_async_enable(&CAN_0);
	/**
	 * CAN_0_tx_callback callback should be invoked after call
	 * can_async_write, and remote device should recieve message with ID=0x45A
	 */
	can_async_write(&CAN_0, &msg);

	msg.id  = 0x100000A5;
	msg.fmt = CAN_FMT_EXTID;
	/**
	 * remote device should recieve message with ID=0x100000A5
	 */
	can_async_write(&CAN_0, &msg);

	/**
	 * CAN_0_rx_callback callback should be invoked after call
	 * can_async_set_filter and remote device send CAN Message with the same
	 * content as the filter.
	 */
	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback);
	filter.id   = 0x469;
	filter.mask = 0;
	can_async_set_filter(&CAN_0, 0, CAN_FMT_STDID, &filter);

	filter.id   = 0x10000096;
	filter.mask = 0;
	can_async_set_filter(&CAN_0, 1, CAN_FMT_EXTID, &filter);
}
