/*
 * board_function.c
 *
 *  Created on: 02.11.2013
 *  Author: Florian Mahlecke
 */


/**
	\file
	\brief System hardware settings/functions (i.e. LEDs or serial)

 */

#include "LPC1768Function.h"


/*********************************************************************//**
 * @brief		initializes the LEDs
 * @param[in]	none
 * @return		none
 **********************************************************************/

void LEDSetup( void )
{

	// P2.0...P2.7 Output LEDs on PORT2 defined as Output
	LPC_GPIO2->FIODIR |= 0x000000ff;

	/* Configure the LCD Control pins */
	LPC_GPIO0->FIODIR   |= 0x03f80000;
	LPC_GPIO0->FIOSET    = 0x03f80000;

}


/*********************************************************************//**
 * @brief		rnable one specific LED
 * @param[in]	num LED no. (i.e. int 1)
 * @return		none
 **********************************************************************/

void LED_On(unsigned int num)
{
 	LPC_GPIO2->FIOPIN |= led_mask[num];
}


/*********************************************************************//**
 * @brief		disable one specific LED
 * @param[in]	num LED no. (i.e. int 2)
 * @return		none
 **********************************************************************/

void LED_Off(unsigned int num)
{
	LPC_GPIO2->FIOPIN &= ~led_mask[num];
}

/*********************************************************************//**
 * @brief		enable LEDs (1,3,5,7)
 * @param[in]	none
 * @return		none
 *
 **********************************************************************/

void LEDOn( void )
{

	LPC_GPIO1->FIOSET=(1<<23);		// puts LED4 - P1.23 ON
	LPC_GPIO1->FIOSET=(1<<21);		// puts LED3 - P1.21 ON
	LPC_GPIO1->FIOSET=(1<<20);		// puts LED2 - P1.20 ON
	LPC_GPIO1->FIOSET=(1<<18);		// puts LED1 - P1.18 ON

}

/*********************************************************************//**
 * @brief		disable LEDs (1,3,5,7)
 * @param[in]	none
 * @return		none
 **********************************************************************/

void LEDOff( void )
{

	LPC_GPIO1->FIOCLR=(1<<23);		// puts LED4 - P1.23 OFF
	LPC_GPIO1->FIOCLR=(1<<21);		// puts LED3 - P1.21 OFF
	LPC_GPIO1->FIOCLR=(1<<20);		// puts LED2 - P1.20 OFF
	LPC_GPIO1->FIOCLR=(1<<18);		// puts LED1 - P1.18 OFF

}


/*********************************************************************//**
 * @brief		print message to serial port
 * @param[in]	lpc_uart UART port (i.e. LPC_UART2)
 * @param[in]	string message
 * @return		none
 **********************************************************************/

void printToSerialPort(LPC_UART_TypeDef *lpc_uart, char *string)
{
	for ( ;; )
	{
		UARTPuts_(lpc_uart, string);
	}
}

/*********************************************************************//**
 * @brief		initializes UART2 as serial console in-/output
 * @param[in]	none
 * @return		none
 *
 * <b>Note:</b> Hardware reference manual <a href="https://sourceforge.net/p/smallpearl/code/ci/master/tree/runtime/cortex_m3_landtiger_freertos/documentation/landtiger_v2.0_-_manual__v1.1.pdf">
 * 				LandTiger V2.0 LPC17XX Development Board</a>
 **********************************************************************/

void init_UART2 ( void ) {

	PINSEL_CFG_Type PinCfg;
	UART_CFG_Type UARTConfigStruct;


	//	Initialize UART2 pin connect

	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 10;
	PinCfg.Portnum = 0;
	PINSEL_ConfigPin(&PinCfg);
	PinCfg.Pinnum = 11;
	PINSEL_ConfigPin(&PinCfg);


	/*
		Initialize UART Configuration parameter structure to default state:
		Baudrate = 9600bps
		8 data bit
		1 Stop bit
		None parity
	*/
	UART_ConfigStructInit(&UARTConfigStruct);

	//	Initialize DEBUG_UART_PORT peripheral with given to corresponding parameter
	UART_Init(LPC_UART2, &UARTConfigStruct);

	//	Enable UART Transmit
	UART_TxCmd(LPC_UART2, ENABLE);
}


/*********************************************************************//**
 * @brief		initializes TFT (not working!!)
 * @param[in]	none
 * @return		none
 **********************************************************************/

void init_LCD(void) {


}


/*********************************************************************//**
 * @brief		initializes system RTC
 * @param[in]	none
 * @return		none
 *
 * <b>Note:</b> Hardware reference manual <a href="https://sourceforge.net/p/smallpearl/code/ci/master/tree/runtime/cortex_m3_landtiger_freertos/documentation/landtiger_v2.0_-_manual__v1.1.pdf">
 * 				LandTiger V2.0 LPC17XX Development Board</a>
 **********************************************************************/

void init_RTC(void) {


}

/*********************************************************************//**
 * @brief		simple wrapper function for RTC_GetFullTime()
 * @param[in]	time
 * @return		none
 **********************************************************************/

void getRTCTime(RTC_TIME_Type* time) {

}



/*********************************************************************//**
 * @brief		Initializes KEY1/SW4 as button
 * @param[in]	None
 * @return		None
 *
 * <b>Note:</b> If button is pressed the debug routine started immediately. Hardware
 * 				reference manual <a href="https://sourceforge.net/p/smallpearl/code/ci/master/tree/runtime/cortex_m3_landtiger_freertos/documentation/landtiger_v2.0_-_manual__v1.1.pdf">
 * 				LandTiger V2.0 LPC17XX Development Board</a>
 **********************************************************************/

void init_EINT1(void){

	PINSEL_CFG_Type PinCfg;
	EXTI_InitTypeDef EXTICfg;

	// Initialize EXT pin and register
	// P2.11 as /EINT1 (KEY1/SW4)
	PinCfg.Funcnum = 1;
	PinCfg.OpenDrain = 0;
	PinCfg.Pinmode = 0;
	PinCfg.Pinnum = 11;
	PinCfg.Portnum = 2;
	PINSEL_ConfigPin(&PinCfg);

	EXTI_Init();
	EXTICfg.EXTI_Line = EXTI_EINT1;
	// edge sensitive
	EXTICfg.EXTI_Mode = EXTI_MODE_EDGE_SENSITIVE;
	EXTICfg.EXTI_polarity = EXTI_POLARITY_LOW_ACTIVE_OR_FALLING_EDGE;
	EXTI_ClearEXTIFlag(EXTI_EINT1);
	EXTI_Config(&EXTICfg);

	NVIC_SetPriorityGrouping(4);
	NVIC_SetPriority(EINT1_IRQn, 0);
	NVIC_EnableIRQ (EINT1_IRQn);

}


/*********************************************************************//**
 * @brief		EINT1 IRQ Handler routine
 * @param[in]	None
 * @return		None
 **********************************************************************/

void EINT1_IRQHandler(void)
{

	// clear the EINT0 flag
	EXTI_ClearEXTIFlag(EXTI_EINT1);
	char buffer[] = "Button pressed!";

	while (1)
	{

		printDebugMessage(buffer);
	}

}


/*
void RTC_IRQHandler(void) {

	if (RTC_GetIntPending(LPC_RTC, RTC_INT_ALARM))
	{
		// Send debug information
		printDebugMessage("ALARM!");

		// Clear pending interrupt
		RTC_ClearIntPending(LPC_RTC, RTC_INT_ALARM);
		RTC_AlarmIntConfig (LPC_RTC, RTC_TIMETYPE_SECOND, DISABLE);
	}
}
*/

void setAlarmTime(int alarm) {

	//RTC_SetAlarmTime (LPC_RTC, RTC_TIMETYPE_SECOND, alarm);
}


/*********************************************************************//**
 * @brief		Initializes hardware
 * @param[in]	None
 * @return		None
 *
 * <b>Note:</b> Board and FreeRTOS specific settings
 **********************************************************************/

void prvSetupHardware( void )
{

	/*
		PLL Konfiguration,
		CPU=80MHz
		ExtOsc=12MHz,
		For details see System_LPC17xx.c
		in folder Libraries->NXP->CORE->CM3->DeviceSupport->NXP->LPC17xx
	*/

	SystemCoreClockUpdate();

	// 	Enable LEDs
	LPC_GPIO0->FIODIR|=(1<<1);
    LEDSetup();

	// 	Enable UART2
	LPC_SC->PCONP |= ( 1 << 24);
	UART_CFG_Type UARTConfigStruct;
	UART_ConfigStructInit(&UARTConfigStruct);
	init_UART2();

	// Enable KEY1/SW4
	init_EINT1();

	// Enable LCD
	// init_LCD();

	// Enable RTC
	//init_RTC();


}

/*********************************************************************//**
 * @brief		FreeRTOS IDLE task
 * @param[in]	None
 * @return		None
 *
 * <b>Note:</b> FreeRTOS specific application IDLE task. Task priority tskIDLE_PRIORITY
 **********************************************************************/

void vApplicationIdleHook(void) {

	char string[] = "Idle-Task is running!";
	UARTPuts_(LPC_UART2, string);

}

/*********************************************************************//**
 * @brief		Print char string to serial console
 * @param[in]	message debug message
 * @return		None
 *
 *  <b>Note:</b> Print char string to serial console (default setting: LPC_UART2)
 **********************************************************************/

void printDebugMessage(char* message) {

	UARTPuts_(LPC_UART2, message);
}

/*********************************************************************//**
 * @brief		Print char sign to serial console
 * @param[in]	charMsg debug char sign
 * @return		None
 *
 * <b>Note:</b> Print char sign to serial console (default setting: LPC_UART2)
 **********************************************************************/


void printDebugChar(uint8_t charMsg) {

	UARTPutChar(LPC_UART2, charMsg);

}


void printDebugInt(int64_t value) {

	UARTPutDec32(LPC_UART2, value);

}
