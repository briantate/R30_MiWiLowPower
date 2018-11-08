/*
 * app.c
 *
 * Created: 10/23/2018 1:57:40 PM
 *  Author: C41175
 */ 

#include <asf.h>
#include "app.h"
#include "debug_interface.h"
#include "miwi_api.h"
#include "conf_clocks.h" //needed for sleep mode operation
#include "custom_board.h"

#define SEND_BUFFER_SIZE  1

static bool switchLastState = false;
static bool switchState = false;
static uint8_t cntVal = 0;
static uint8_t sendDataBuffer[SEND_BUFFER_SIZE];
uint8_t msghandledemo = 0;
uint8_t extintCount = 0;

static bool txComplete = true;



static void configure_extint_channel(void);
static void dataConfcb(uint8_t handle, miwi_status_t status, uint8_t* msgPointer);
static void extint_callback(void);
static void main_clock_select_osc16m(void);
static void main_clock_select_dfll(void);
static void main_clock_select(const enum system_clock_source clock_source);


void AppInit(void)
{
	configure_console();
	
	printf("R30 Low-Power MiWi Project\r\n");
	uint32_t cpuClock = system_cpu_clock_get_hz();
	DEBUG_PRINT(printf("CPU clock %lu Hz\r\n", cpuClock));
	
	//enable wakeup via SW0
	configure_extint_channel();
}

void AppTask(void)
{
	if(role==true) //PAN coordinator
	{
		switchState = port_pin_get_input_level(SW0_PIN);
		if(!switchState & switchLastState)
		{
			port_pin_toggle_output_level(LED1);
			sendDataBuffer[0] = 0x41 + cntVal%10; //start at ascii A
			cntVal++;
			DEBUG_PRINT(printf("sending char: %u\r\n", sendDataBuffer[0]));
			
			//send broadcast data
			uint16_t broadcastAddress = 0xFFFF;
			MiApp_SendData(SHORT_ADDR_LEN, (uint8_t *)&broadcastAddress,
				SEND_BUFFER_SIZE, sendDataBuffer, msghandledemo++, true, dataConfcb);
		}
		switchLastState = switchState;
	}
	else //edge node
	{
		//send data
		sendDataBuffer[0] = 0x41 + cntVal%10; //start at ascii A
		cntVal++;
		
		//send broadcast data
		uint16_t broadcastAddress = 0xFFFF;
		DEBUG_PRINT(printf("sending char: %u\r\n", sendDataBuffer[0]));
		txComplete = false;
		MiApp_SendData(SHORT_ADDR_LEN, (uint8_t *)&broadcastAddress,
			SEND_BUFFER_SIZE, sendDataBuffer, msghandledemo++, true, dataConfcb);
			
		//Wait until the transmission is complete before sleeping
//		delay_ms(10);
		while(txComplete != true)
		{
			P2PTasks();
		}
		
		//sleep radio
		DEBUG_PRINT(printf("sleeping the radio\r\n"));

		MiApp_TransceiverPowerState(POWER_STATE_SLEEP);
		
		//sleep MCU
		DEBUG_PRINT(printf("sleeping the MCU\r\n"));
		
		// Scaling down clock frequency and then Scaling down the performance level
		// ***ToDo: Are there timing requirements before these are completed? 
		// ***      If I step through these slowly, the sleep current is lower
		main_clock_select(SYSTEM_CLOCK_SOURCE_OSC16M);
		system_switch_performance_level(SYSTEM_PERFORMANCE_LEVEL_0);
		system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
		system_sleep();
		
		//wait for interrupt to wake the MCU

		//reset clocks
		// Scaling up the performance level first and then increase clock frequency
		system_switch_performance_level(SYSTEM_PERFORMANCE_LEVEL_2);
		main_clock_select(SYSTEM_CLOCK_SOURCE_DFLL);
		DEBUG_PRINT(printf("Good Morning\r\n"));
		extintCount--;
	
		//wake radio
		MiApp_TransceiverPowerState(POWER_STATE_WAKEUP);
	}
}

static void dataConfcb(uint8_t handle, miwi_status_t status, uint8_t* msgPointer)
{
	txComplete = true;
	port_pin_toggle_output_level(LED0);
	
	//DEBUG_PRINT(printf("\nData Confirm: Handle: %d status:%d\r\n", handle, status));
	//MiMem_Free(msgPointer);
}

static void extint_callback(void)
{
	extintCount++;
}

void ReceivedDataIndication (RECEIVED_MESSAGE *ind)
{
	uint8_t startPayloadIndex = 0;
	/*******************************************************************/
	// If a packet has been received, handle the information available
	// in rxMessage.
	/*******************************************************************/
	DEBUG_PRINT(printf("data received: "));
	
	// Toggle LED to indicate receiving a packet.
	port_pin_toggle_output_level(LED0);
	
	for(uint8_t i=startPayloadIndex; i<rxMessage.PayloadSize;i++)
	{
		DEBUG_PRINT(putchar(rxMessage.Payload[i]));
	}
	DEBUG_PRINT(printf("\r\n"));
	
}

/**
 * \brief Config external interrupt.
 */
static void configure_extint_channel(void)
{

	struct extint_chan_conf config_extint_chan;
	extint_chan_get_config_defaults(&config_extint_chan);
	config_extint_chan.gpio_pin           = SW0_EIC_PIN;
	config_extint_chan.gpio_pin_mux       = SW0_EIC_MUX;
	config_extint_chan.gpio_pin_pull      = EXTINT_PULL_UP;
	config_extint_chan.detection_criteria = EXTINT_DETECT_FALLING;
	extint_chan_set_config(SW0_EIC_LINE, &config_extint_chan);
	extint_register_callback(extint_callback,SW0_EIC_LINE,EXTINT_CALLBACK_TYPE_DETECT);
 	extint_chan_enable_callback(SW0_EIC_LINE,EXTINT_CALLBACK_TYPE_DETECT);
	while (extint_chan_is_detected(SW0_EIC_LINE)) {
		extint_chan_clear_detected(SW0_EIC_LINE);
	}
}

/**
 * \brief Select OSC16M as main clock source.
 */
static void main_clock_select_osc16m(void)
{
	struct system_gclk_gen_config gclk_conf;
	struct system_clock_source_osc16m_config osc16m_conf;

	/* Switch to new frequency selection and enable OSC16M */
	system_clock_source_osc16m_get_config_defaults(&osc16m_conf);
	osc16m_conf.fsel = CONF_CLOCK_OSC16M_FREQ_SEL;
	osc16m_conf.on_demand = 0;
	osc16m_conf.run_in_standby = CONF_CLOCK_OSC16M_RUN_IN_STANDBY;
	system_clock_source_osc16m_set_config(&osc16m_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_OSC16M);
	while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_OSC16M));

	/* Select OSC16M as mainclock */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_OSC16M;
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);
	if (CONF_CLOCK_OSC16M_ON_DEMAND) {
		OSCCTRL->OSC16MCTRL.reg |= OSCCTRL_OSC16MCTRL_ONDEMAND;
	}

}

/**
 * \brief Setect DFLL as main clock source.
 */
static void main_clock_select_dfll(void)
{
	struct system_gclk_gen_config gclk_conf;

	/* Select OSCULP32K as new clock source for mainclock temporarily */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_XOSC32K;
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);

	/* Select XOSC32K for GCLK1. */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_XOSC32K;
	gclk_conf.division_factor = CONF_CLOCK_GCLK_1_PRESCALER;
	system_gclk_gen_set_config(GCLK_GENERATOR_1, &gclk_conf);
	system_gclk_gen_enable(GCLK_GENERATOR_1);

	struct system_gclk_chan_config dfll_gclk_chan_conf;

	system_gclk_chan_get_config_defaults(&dfll_gclk_chan_conf);
	dfll_gclk_chan_conf.source_generator = GCLK_GENERATOR_1;
	system_gclk_chan_set_config(OSCCTRL_GCLK_ID_DFLL48, &dfll_gclk_chan_conf);
	system_gclk_chan_enable(OSCCTRL_GCLK_ID_DFLL48);
	
	struct system_clock_source_dfll_config dfll_conf;
	system_clock_source_dfll_get_config_defaults(&dfll_conf);

	dfll_conf.loop_mode      = SYSTEM_CLOCK_DFLL_LOOP_MODE_CLOSED;
	dfll_conf.on_demand      = false;
	dfll_conf.run_in_stanby  = CONF_CLOCK_DFLL_RUN_IN_STANDBY;
	dfll_conf.multiply_factor = CONF_CLOCK_DFLL_MULTIPLY_FACTOR;
	system_clock_source_dfll_set_config(&dfll_conf);
	system_clock_source_enable(SYSTEM_CLOCK_SOURCE_DFLL);
	while(!system_clock_source_is_ready(SYSTEM_CLOCK_SOURCE_DFLL));
	if (CONF_CLOCK_DFLL_ON_DEMAND) {
		OSCCTRL->DFLLCTRL.bit.ONDEMAND = 1;
	}

	/* Select DFLL for mainclock. */
	system_gclk_gen_get_config_defaults(&gclk_conf);
	gclk_conf.source_clock = SYSTEM_CLOCK_SOURCE_DFLL;
	system_gclk_gen_set_config(GCLK_GENERATOR_0, &gclk_conf);

}

/**
 * \brief Main clock source selection between DFLL and OSC16M.
 */
static void main_clock_select(const enum system_clock_source clock_source)
{
	if (clock_source == SYSTEM_CLOCK_SOURCE_DFLL) {
		main_clock_select_dfll();
		system_clock_source_disable(SYSTEM_CLOCK_SOURCE_OSC16M);
	} else if (clock_source == SYSTEM_CLOCK_SOURCE_OSC16M) {
		main_clock_select_osc16m();
		system_clock_source_disable(SYSTEM_CLOCK_SOURCE_DFLL);
		system_gclk_chan_disable(OSCCTRL_GCLK_ID_DFLL48);
		system_gclk_gen_disable(GCLK_GENERATOR_1);
	} else {
		return ;
	}
}
