#include <atmel_start.h>
#include <stdlib.h>
//=========================LOCAL DEFINITIONS==========================================
//COMMS timeouts
#define TIMEOUT_INV1	500
#define TIMEOUT_INV2	500
#define TIMEOUT_PB		500
#define TIMEOUT_BMS		500

//CAN Buffer values @@@
#define CAN_RX_BUFFER_SIZE 16
//CAN Ids
#define CAN_ID_PEDAL_BOARD	0x100
#define CAN_ID_BMS_CELL_BROADCAST 0x6B0 
#define CAN_ID_AUX_STATES		0x900

#define ASS_CLOSED 1
#define ASS_OPEN 0
//Pins that werent defined in atstart because its shit
#define ASS_RELAY_PIN PIN_PC19


static bool has_delay_passed(uint32_t start_time, uint32_t);
static uint32_t current_time_ms(void);
//=========================LOCAL TYPES====================================

//========================ADC========================== checking ts main 
// PD30 is TS
// PC31 brakelight 
static void adc_init(void)	// This function already exists in the adc code from the driver itself
{
	//needs to setup adc0 and adc1
}

static bool ts_active(void) // @@ ask george what was said with the ts active pin
{
	//We have setup an async ADC routine - think we want it syncronous
	//reads adc1 pin PD30
	//need to check its above an amount
	
	//return false;
	return true;
}

/*
ADC is async, meaning we tell it to convert, and an interrupt will occur when its done
Therefore we need a handler to take the data from the interrupt into usable information for other modules
As well as trigger the next adc samples

@@ Probably easier doing it non-async
*/
static void handle_adc(void)
{
	
	
}

//====================================TIMEOUTS==========================
struct
{
	bool inv1;
	bool inv2;
	bool pb;
	bool bms;
	
}comms_active;

static uint32_t inv1_rx_time = 0;
static uint32_t inv2_rx_time = 0;
static uint32_t pb_rx_time = 0;
static uint32_t bms_rx_time = 0;


void handle_timeouts(void)
{
	if(has_delay_passed(inv1_rx_time,TIMEOUT_INV1))		comms_active.inv1 = false;
	else												comms_active.inv1 = true;
	
	if(has_delay_passed(inv2_rx_time,TIMEOUT_INV1))		comms_active.inv2 = false;
	else												comms_active.inv2 = true;
	
	if(has_delay_passed(pb_rx_time,TIMEOUT_INV1))		comms_active.pb = false;
	else												comms_active.pb = true;
	
	if(has_delay_passed(bms_rx_time,TIMEOUT_INV1))		comms_active.bms = false;
	else												comms_active.bms = true;
}

//=========================BMS=============================================
struct {
	//inputs
	uint16_t voltage;
	
	//outputs
	bool precharge_enable;
}bms;
//======================ASS===============================================
#define ASS_RELAY_STATE //@@What pin do we read the ASS loop on?
// @@ relay starts open and then after RTD is toggled then relay closes - hardwire OR do can 
#define ASS_LOOP_STATE gpio_get_pin_level(ASS_LOOP_PIN)

struct  
{	
	//Turn off VCU ass relay conditions
	bool break_loop_precharge;
	bool break_loop_carIntact;
	bool break_loop_wantToStart;
}ass;

void handle_ass(void) // to check ass state possibly splice and see the state either using a reg or mfet thing from #george 
{
	bool ass_close =	!ass.break_loop_precharge ||
						!ass.break_loop_carIntact ||
						!ass.break_loop_wantToStart;
	bool new_ass_state = ass_close ? ASS_CLOSED : ASS_OPEN;		
	
	gpio_set_pin_level(ASS_RELAY_PIN, new_ass_state);
}

//======================INVERTERS=========================================

// Motor stream is the uart recive buffer
//State of Motor Controller as given from the plettenberg manual (this is the info we are getting from the MCs)
typedef struct {
	char inputmethod;
	float aux_input;
	float throttle_input;
	int pwm;
	float voltage;
	float phase_current;
	int rpm;
	int power_stage_temp;
	int motor_temp; // this is the one we want for dash display (to be sent across can)
	
}inv_t;

inv_t inv1;
inv_t inv2;

uint16_t get_inv_lowest_voltage(void)
{
	if( inv1.voltage < inv2.voltage) return inv1.voltage;
	else return inv2.voltage;
}

// #Transmit #function - io write so we don't have one
// handle to send the pedal stuff

// Parse upon receive and handle mode
static void inv_parse_rx(char* msg, size_t len, inv_t* inv, struct io_descriptor *io) // @@ Check if we can read the inverter stuff for reading the voltage
{
	// IO DESCRIPTOR ARG Is for writing the s
	
	// Startup check - when the inverters start they send a menu with a * character which we should ignore, 
	// Therefore, only do the rest of the code (pare into the struct) when st_c = 0;
	int st_c = 1;
	
	// Check for garbage
	switch(msg[0])
	{
		case '*': // garbled start up message
			st_c = 1;
			break;
		case 'T': // big letter active - we want to change but maybe we cant @@ as a future safety thing, may want to turn off inverter, send lowercase s and then turn on
			st_c = 0;
			break;
		case 'S':
			st_c = 0;
			break;
		case 't': // inactive analogue
			// Change to 's'
			st_c = 0;
			io_write(io,"s\r\n",4); // length including null terminator 
			break; 
		case 's':
			st_c = 0;
			break;
		default:
			return; // not any of those values so return (bad message!!!) 
	}

	
	if (st_c == 0) //  if its not start up noise, do the good shit
	{
		char* curr = (char*) msg;
		
			// Get the message and put in struct
			#define ERROR_CHECK if ((uintptr_t)curr > (uintptr_t)msg+len) goto error;
			inv->throttle_input = strtof(curr+2, &curr); // starts at curr 2 however many digits it taks to complete it it will increm cur pointer
			ERROR_CHECK
			inv->aux_input = strtof(curr+4, &curr); // check what string to long returns if garbled // check garbled maybe???
			ERROR_CHECK
			inv->pwm = strtol(curr+6, &curr, 10);
			ERROR_CHECK
			inv->voltage = strtof(curr+3, &curr);
			ERROR_CHECK
			inv->phase_current = strtof(curr+4, &curr);
			ERROR_CHECK
			inv->rpm = strtol(curr+6, &curr, 10);
			ERROR_CHECK
			inv->power_stage_temp = strtol(curr+5, &curr, 10);
			ERROR_CHECK
			inv->motor_temp = strtol(curr+7, &curr, 10); // debugged to +7
			ERROR_CHECK
			#undef ERROR_CHECK
			// @@@@ NOTE: when testing in C ide, the compiler does not recognize the degree character
			return;
		
			error:
			//log_warn("mc_parser: read beyond the end of the string!!!");
			return;
	}
	
	
}



/////////////////////////////////////// TESTIN TESTIN /////////////////////////////////////
static char test_buf[2]; // @CAREFULEEEEE
static void handle_inverter(inv_t* inv, struct io_descriptor *io)
{
	char bob[] = "1f";
	test_buf[0] = bob[0];
	test_buf[1] = bob[1];
	
	// pHASE TWO OF TESTING
	
	io_write(io,test_buf,2); // why we send the othe riowrite wiht \n but not anything else!
}

//=======================UART=============================================

static bool uart1_ready;
static bool uart2_ready;

static char inv1_rx_buf[128];
uint8_t inv1_rx_ptr = 0;
static char inv2_rx_buf[128];
uint8_t inv2_rx_ptr = 0;

static void rx_cb_UART_MC_1(const struct usart_async_descriptor *const io_descr) // @@ where io_desc used my guy @@@@
{
	char temp_char;
	int msg_len_counter = 0;
	
	while(io_read(&UART_MC_1.io,&temp_char,1) && (msg_len_counter <=77))	//Keep reading until the buffer is empty // ask george 
	// addition -> added <= 77 as that is the total length of the message, worried if this condition not there and there are extra chars it will overwrite the message at the start as the pointer goes back to 0 once /n is reached
	{
		inv1_rx_time = current_time_ms();
		
		inv1_rx_buf[inv1_rx_ptr] = temp_char;
		if(inv1_rx_ptr + 1 < sizeof(inv1_rx_buf)) inv1_rx_ptr++;
		else inv1_rx_ptr = 0;
		
		if(temp_char == '\n') // the end termination character is newline not /0 like we thought hahaha im in pain
		{
			uart1_ready = true;
			inv1_rx_ptr = 0;
		}
		msg_len_counter++;
	}

	//io_read(&UART_MC_1.io,&inv1_rx_buf,) <- might want to make the last arg take the full message which is 77 chars including the \n


	// @@@@ set the limit in the third argument to read the whole thing
	// And then, add the null terminator at the end to make it an actual string (it doesn't with null, we also want to send no null)
}

static void rx_cb_UART_MC_2(const struct usart_async_descriptor *const io_descr)
{
	char temp_char;
	int msg_len_counter = 0;
	
	while(io_read(&UART_MC_2.io,&temp_char,1) && (msg_len_counter <=77))	//Keep reading until the buffer is empty
	{
		inv2_rx_time = current_time_ms();
		
		inv2_rx_buf[inv2_rx_ptr] = temp_char;
		if(inv2_rx_ptr + 1 < sizeof(inv2_rx_buf)) inv2_rx_ptr++;
		else inv2_rx_ptr = 0;
		
		if(temp_char == '\0')
		{
			uart2_ready = true;
			
			inv2_rx_ptr = 0;
		}
		msg_len_counter++;
	}
}

static void handle_uart(void) // memory safe - io read is mem safe and only looking at buffer.
{
	if(uart1_ready)	//If the uart 1 interrupt has been called
	{
		inv_parse_rx(inv1_rx_buf, inv1_rx_ptr, &inv1, &UART_MC_1.io);
		uart1_ready = false;
	}
	if(uart2_ready)
	{
		inv_parse_rx(inv2_rx_buf, inv2_rx_ptr, &inv2, &UART_MC_2.io);
		uart2_ready = false;
	}
}

static void uart_init(void)
{
	usart_async_register_callback(&UART_MC_1, USART_ASYNC_RXC_CB, rx_cb_UART_MC_1);
	usart_async_register_callback(&UART_MC_2, USART_ASYNC_RXC_CB, rx_cb_UART_MC_2);
	usart_async_enable(&UART_MC_1);
	usart_async_enable(&UART_MC_2);
}

//=======================CAN==============================================
//CAN circular buffer
struct can_message can_rx_queue[CAN_RX_BUFFER_SIZE];
uint8_t can_rx_data_buffer[CAN_RX_BUFFER_SIZE][8];
uint8_t can_rx_queue_head = 0;
uint8_t can_rx_queue_tail = 0;
uint8_t can_rx_queue_len = 0;

//Sets up the buffer for the CAN rx queue
void can_rx_queue_initialise(void)
{
	for(uint8_t i = 0; i < CAN_RX_BUFFER_SIZE;i++)
	{
		can_rx_queue[i].data = can_rx_data_buffer[i];
	}
}

//returns a pointer to where the next can message can be stored
struct can_message* can_rx_queue_push(void)
{
	struct can_message* ret = &can_rx_queue[can_rx_queue_head];
	
	if(can_rx_queue_head + 1< CAN_RX_BUFFER_SIZE) can_rx_queue_head++;
	else can_rx_queue_head = 0;
	
	if(can_rx_queue_len < CAN_RX_BUFFER_SIZE) can_rx_queue_len++;
	
	return ret;
}

//returns a pointer to the can message at the bottom of the queue
struct can_message* can_rx_queue_pop(void)
{
	struct can_message* ret = &can_rx_queue[can_rx_queue_tail];
	
	if(can_rx_queue_tail + 1 < CAN_RX_BUFFER_SIZE) can_rx_queue_tail++;
	else can_rx_queue_tail = 0;
	
	if(can_rx_queue_len > 0) can_rx_queue_len--;
	else
	{
		//&&Need a return case for if the length is 0
	}	
	return ret;
}

static void CAN_0_tx_callback(struct can_async_descriptor *const descr) {
	gpio_toggle_pin_level(PIN_PC8); // this should just be to toggle the led or to let us use a multimeter to check
	(void)descr; 
}


static void CAN_0_rx_callback(struct can_async_descriptor *const descr) {
	gpio_toggle_pin_level(PIN_PC8); // blink when msg is received
	
	//Adds new buffers for 
	struct can_message *msg;
	msg = can_rx_queue_push();
	can_async_read(descr,msg);
}

static void can_init(void)
{
	//Initialise the interrupts
	can_async_register_callback(&CAN_0, CAN_ASYNC_RX_CB, (FUNC_PTR)CAN_0_rx_callback);
	can_async_register_callback(&CAN_0, CAN_ASYNC_TX_CB, (FUNC_PTR)CAN_0_tx_callback);
	can_async_enable(&CAN_0);
	
	//Take CAN modules out of standby @@ check this??
	gpio_set_pin_direction(PIN_PD11, GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PIN_PD11, 0);
	
	//Sets no filter
	struct can_filter filter;
	filter.id   = 0;
	filter.mask = 0;
	can_async_set_filter(&CAN_0, 0, CAN_FMT_STDID, &filter);
}

//Handles one element from the CAN buffer
static void handle_can(void)
{
	if(can_rx_queue_len > 0) // If the can_rx_queue has messages, then we have to handle them
	{
		struct can_message* msg;
		msg = can_rx_queue_pop();
		
		switch(msg->id)
		{
			case CAN_ID_PEDAL_BOARD: 
				pb_rx_time = current_time_ms();
				//Need to handle the messages
				break;
			case CAN_ID_BMS_CELL_BROADCAST:
				bms_rx_time = current_time_ms();
				//Byte 2 - Pack inst voltage high??
				//Byte 3 - Pack inst voltage low??
				//0.1V per bit
				bms.voltage = (((uint16_t)msg->data[2] << 8) + msg->data[3])/10;	//Maybe can leave in 0.1V/bit, just have to adjust elsewhere

				//Need to handle the messages
				break;
			case CAN_ID_AUX_STATES:
				//Need to handle the messages
				break;
		}
	}
}

//===============================TIMER=====================================
/*
Flip flop timer

When the timer interrupt occurs, 
	if flipflop = 1, timer_0 is edited,
	else timer_1 is edited
	
Therefore when reading the current time
	if flipflop = 1, read timer_1, as it won't be edited for at least 1ms
	else read timer_0
*/

static volatile uint32_t ms_timer_0 = 0;
static volatile uint32_t ms_timer_1 = 1;
static volatile bool ms_timer_flipflop;

static bool has_delay_passed(uint32_t start_time, uint32_t delay)
{
	if ((current_time_ms() - start_time) > delay) return true;
	else return false;
}

static uint32_t current_time_ms(void)
{
	if(ms_timer_flipflop == 1)	return ms_timer_1;
	else				return ms_timer_0;
}

static void TIMER_0_task1_cb(const struct timer_task *const timer_task)
{
	if(ms_timer_flipflop == 1)		ms_timer_0 += 2;
	else							ms_timer_1 += 2;	
	
	ms_timer_flipflop ^= 1;
}

static void init_timer(void)
{
	static struct timer_task TIMER_0_task1;
	TIMER_0_task1.interval = 1; // 1 ms interrupt
	TIMER_0_task1.cb       = TIMER_0_task1_cb;
	TIMER_0_task1.mode     = TIMER_TASK_REPEAT
	
	
	
	
	timer_add_task(&TIMER_0, &TIMER_0_task1);
	timer_start(&TIMER_0);
}

//=================================PRECHARGE==========================================

static void handle_precharge(void)
{
	static enum {
		PC_TS_OFF,						//The TS is not active
		PC_WAIT_FOR_INVERTERP,			//Waiting for communication from the inverters
		PC_WAIT_FOR_FINAL_VOLTAGE,		//Waiting for the inverter voltage to reach 95% of battery voltage
		PC_READY,						//Ready to for BMS to exit precharge
		PC_FAILED						//A timeout has occured, set flag for VCU relay to open 
	} PRECHARGE_STATE = PC_TS_OFF;		//Different possible precharge states
	
	static uint32_t precharge_start_time = 0;

	
	switch(PRECHARGE_STATE)
	{
		case PC_TS_OFF:
			bms.precharge_enable = false;
			ass.break_loop_precharge = true;
		
			if(ts_active())
			{
				PRECHARGE_STATE = PC_WAIT_FOR_INVERTERP;
				precharge_start_time = current_time_ms();
			}
			break;
		case PC_WAIT_FOR_INVERTERP:
			bms.precharge_enable = false;
			ass.break_loop_precharge = true;
			
			if(comms_active.inv1 && comms_active.inv2)
			{
				PRECHARGE_STATE = PC_WAIT_FOR_FINAL_VOLTAGE;
			}
			else
			{
				//@@ Needs to calculate total delay
				//t = RC ln ( Vbat / (Vbat - 30*1.2) ) * 1000
				if(has_delay_passed(precharge_start_time, 5000))
				{
					PRECHARGE_STATE = PC_FAILED;
				}
				//Checked we have received communication from inverter
				//else check how much time has passed
			}
			

			break;
		case PC_WAIT_FOR_FINAL_VOLTAGE:
			bms.precharge_enable = false;
			ass.break_loop_precharge = true;
		
			//Read battery voltage
			//Check inverter voltage has reached 95% of battery voltage
			//If not, check how much time has passed, exit if necessary
			
			if(get_inv_lowest_voltage() > bms.voltage * 0.95)
			{
				PRECHARGE_STATE = PC_READY;
			}
			else
			{
				//t = 1000* -RCln(1-0.95) =1000* RCln20 = 3743
				if(has_delay_passed(precharge_start_time, 3743))
				{
					PRECHARGE_STATE = PC_FAILED;
				}
			}
			
			break;
		case PC_READY:
			bms.precharge_enable = true;
			ass.break_loop_precharge = false;
			break;
		case PC_FAILED:
			bms.precharge_enable = false;
			ass.break_loop_precharge = true;
			break;
	}
}

int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();
	can_init();
	init_timer();
	uart_init();
	can_rx_queue_initialise();
	
	gpio_set_pin_level(ASS_RELAY_PIN, ASS_OPEN); // 0 is open 1 is closed
	
	/* Replace with your application code */
	while (1) 
	{
		
		// if ready do something!!
		handle_uart();
		handle_inverter(&inv2, &UART_MC_2.io); // change to 2 if needed
		
		// handle inverters --> will use the inverter instances and transmit the appropriate throttle
		// handle dash --> send from the inv struct the avg motor temp 
		
		//handle_can();
		//handle_precharge();
		//handle_ass();
		//handle_timeouts();
		//handle_uart();
		
		// read the message (assume over 30V)
			// when you get a message there is an interrupt which is handled in handle uart
		
		// Write a throttle input
		
		//Get it to spam s to the inverter
		io_write(&UART_MC_1.io.,"s\r\n",4);
		io_write(&UART_MC_2.io,"s\r\n",4);
		
		
		delay_ms(100);
		
	}
}
