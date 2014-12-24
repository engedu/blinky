/*
 * LPC1768Function.h
 *
 *  Created on: 02.11.2013
 *      Author: Florian Mahlecke
 *
 */

/**
	\file
	\brief System hardware settings/functions (i.e. LEDs or serial)

 */

#ifndef LPC1768FUNCTION_H_
#define LPC1768FUNCTION_H_


#include "Date.h"


/**  @file */

#define LED_NUM     8                   /* Number of user LEDs                */
const unsigned long led_mask[] = { 1UL<<0, 1UL<<1, 1UL<<2, 1UL<< 3,
                                   1UL<< 4, 1UL<< 5, 1UL<< 6, 1UL<< 7 };


#ifdef __cplusplus
extern "C" {
#endif

#include "NXP/Core/CM3/DeviceSupport/NXP/LPC17xx/LPC17xx.h"
#include "NXP/Drivers/include/lpc17xx_pinsel.h"
#include "NXP/Drivers/include/lpc17xx_exti.h"
#include "NXP/Drivers/include/lpc17xx_rtc.h"
#include "NXP/Drivers/include/debug_frmwrk.h"


#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern "C" {
#endif


extern int counter;
extern int alarm[];
void setAlarmTime(int alarm);

void LEDSetup();

void LED_On(unsigned int num);

void LED_Off(unsigned int num);

void LEDOn();

void LEDOff();

void printToSerialPort(LPC_UART_TypeDef *lpc_uart, char *string);

void init_UART2 ();

void prvSetupHardware();

void vApplicationIdleHook(void);

void printDebugMessage(char* message);

void printDebugChar(uint8_t charMsg);

void init_EINT1(void);
void EINT1_IRQHandler(void);

void init_LCD(void);

void init_RTC(void);
void getRTCTime(RTC_TIME_Type* time);

void printDebugInt(int64_t value);

//void RTC_IRQHandler(void);

#ifdef __cplusplus
}
#endif


#endif
