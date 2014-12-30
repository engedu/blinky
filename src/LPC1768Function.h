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



/**  @file */

#define LED_NUM     8                   /* Number of user LEDs                */
const unsigned long led_mask[] = { 1UL<<0, 1UL<<1, 1UL<<2, 1UL<< 3,
                                   1UL<< 4, 1UL<< 5, 1UL<< 6, 1UL<< 7 };

#include "LPC17xx.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_exti.h"
#include "lpc17xx_rtc.h"
#include "debug_frmwrk.h"


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


#endif
