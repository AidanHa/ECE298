/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//******************************************************************************
//   Software Port Interrupt Service on S1 from LPM3 with
//                     Internal Pull-up Resistance Enabled
//
//   A hi "TO" low transition on S1 will trigger P1_ISR/P2_ISR which,
//   toggles LED1. S1 is internally enabled to pull-up. LPM3 current
//   can be measured with the LED removed, all
//   unused Px.x configured as output or inputs pulled high or low.
//   ACLK = n/a, MCLK = SMCLK = default DCO
//
//!           MSP430FR2xx_4xx Board
//             -----------------
//         /|\|                 |
//          | |                 |
//          --|RST              |
//            |                 |
//       S1-->|                 |-->LED1
//
//
//   This example uses the following peripherals and I/O signals.  You must
//   review these and change as needed for your own board:
//   - GPIO Port peripheral
//
//   This example uses the following interrupt handlers.  To use this example
//   in your own application you must add these interrupt handlers to your
//   vector table.
//   - PORT1_VECTOR/PORT2_VECTOR
//******************************************************************************
#include "driverlib.h"
#include "Board.h"

#include "msp430fr4133.h"
#include "HAL_FR4133LP_LCD.h"
#include "HAL_FR4133LP_Learn_Board.h"

#define COMPARE_VALUE 50000
#define TIMER_A_PERIOD  1000 //T = 1/f = (TIMER_A_PERIOD * 1 us)
#define HIGH_COUNT      500  //Number of cycles signal is high (Duty Cycle = HIGH_COUNT / TIMER_A_PERIOD)

volatile unsigned int i;
uint8_t time_digit_1;
uint8_t time_digit_2;
uint8_t time_digit_3;
uint8_t time_digit_4;
int time_inc = 0;
int time_seconds = 0;
int time_minutes = 0;
int time_hours = 0;

int alarm_time_minutes = 0;
int alarm_time_hours = 0;

char is_armed = 0;
char tripped = 0;
char user_request_arm = 0;

Timer_A_outputPWMParam param; //Timer configuration data structure for PWM

void delay()
{
	i = 100000;                          // SW Delay
	do
		i--;
	while (i != 0);
}

void turn_off()
{
	GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
	GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN3);
	tripped = 1;
	time_digit_1 = 69;
	time_digit_2 = 69;
	time_digit_3 = 69;
	time_digit_4 = 69;

	time_digit_1 = pollKeypad();
	while (time_digit_1 != 2)
	{                          //2
		time_digit_1 = pollKeypad();
	}
//	LCD_Clear();
//	LCD_Display_digit(pos1, time_digit_1);

	i = 100000;                          // SW Delay
	do
		i--;
	while (i != 0);

	time_digit_2 = pollKeypad();
	while (time_digit_2 != 5)
	{
		time_digit_2 = pollKeypad();
	}
//	LCD_Clear();
//	LCD_Display_digit(pos1, time_digit_1);
//	LCD_Display_digit(pos2, time_digit_2);

	i = 100000;                          // SW Delay
	do
		i--;
	while (i != 0);

	time_digit_3 = pollKeypad();
	while (time_digit_3 != 4)
	{                          //5
		time_digit_3 = pollKeypad();
	}
//	LCD_Clear();
//	LCD_Display_digit(pos1, time_digit_1);
//	LCD_Display_digit(pos2, time_digit_2);
//	LCD_Display_digit(pos4, time_digit_3);

	i = 100000;                          // SW Delay
	do
		i--;
	while (i != 0);

	time_digit_4 = pollKeypad();
	while (time_digit_4 != 6)
	{
		time_digit_4 = pollKeypad();
	}
//	LCD_Clear();
//	LCD_Display_digit(pos1, time_digit_1);
//	LCD_Display_digit(pos2, time_digit_2);
//	LCD_Display_digit(pos4, time_digit_3);
//	LCD_Display_digit(pos5, time_digit_4);

	GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
	GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
	is_armed = 0;
	tripped = 0;
}

char armed()
{
	char temp = 1;
	temp = GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN3) & GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN2) & GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0);
			//& GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN2) & GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN0);
			if (temp)
			{
				return 0;
			}
			return 1;
		}

		char pollKeypadHelper(int col)
		{

			uint8_t row_1;
			uint8_t row_2;
			uint8_t row_3;
			uint8_t row_4;

			row_1 = GPIO_getInputPinValue(GPIO_PORT_P5, GPIO_PIN1);
			row_2 = GPIO_getInputPinValue(GPIO_PORT_P8, GPIO_PIN1);
			row_3 = 0;         //GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1);
			row_4 = GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN7);
			if (col == 1)
			{
				if (row_1)
				{
					return 1;
				}
				else if (row_2)
				{
					return 4;
				}
				else if (row_3)
				{
					return 7;
				}
				else if (row_4)
				{
					return 69;
				}
			}
			else if (col == 2)
			{
				if (row_1)
				{
					return 2;
				}
				else if (row_2)
				{
					return 5;
				}
				else if (row_3)
				{
					return 8;
				}
				else if (row_4)
				{
					return 0;
				}
			}
			else if (col == 3)
			{
				if (row_1)
				{
					return 3;
				}
				else if (row_2)
				{
					return 6;
				}
				else if (row_3)
				{
					return 9;
				}
				else if (row_4)
				{
					return 69;
				}
			}
			return 69;

		}

		char pollKeypad()
		{

			GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN0);
			GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
			GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
			if (pollKeypadHelper(1) != 69)
			{
				return pollKeypadHelper(1);
			}

			GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
			GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN5);
			GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
			if (pollKeypadHelper(2) != 69)
			{
				return pollKeypadHelper(2);
			}

			GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN0);
			GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN5);
			GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);

			if (pollKeypadHelper(3) != 69)
			{
				return pollKeypadHelper(3);
			}

			return 96;

		}

		void setTime()
		{
			char temp = 0;
			time_digit_1 = pollKeypad();
			while (time_digit_1 > 2)
			{                          //2
				time_digit_1 = pollKeypad();
			}
			if (time_digit_1 == 2)
			{
				temp = 1;
			}
			LCD_Clear();
			LCD_Display_digit(pos1, time_digit_1);

			i = 100000;                          // SW Delay
			do
				i--;
			while (i != 0);

			time_digit_2 = pollKeypad();
			while ((temp && time_digit_2 > 3) || (!temp && time_digit_2 > 9))
			{
				time_digit_2 = pollKeypad();
			}
			LCD_Clear();
			LCD_Display_digit(pos1, time_digit_1);
			LCD_Display_digit(pos2, time_digit_2);

			i = 100000;                          // SW Delay
			do
				i--;
			while (i != 0);

			time_digit_3 = pollKeypad();
			while (time_digit_3 > 5)
			{                          //5
				time_digit_3 = pollKeypad();
			}
			LCD_Clear();
			LCD_Display_digit(pos1, time_digit_1);
			LCD_Display_digit(pos2, time_digit_2);
			LCD_Display_digit(pos4, time_digit_3);

			i = 100000;                          // SW Delay
			do
				i--;
			while (i != 0);

			time_digit_4 = pollKeypad();
			while (time_digit_4 > 9)
			{
				time_digit_4 = pollKeypad();
			}
			LCD_Clear();
			LCD_Display_digit(pos1, time_digit_1);
			LCD_Display_digit(pos2, time_digit_2);
			LCD_Display_digit(pos4, time_digit_3);
			LCD_Display_digit(pos5, time_digit_4);

			i = 100000;                          // SW Delay
			do
				i--;
			while (i != 0);

			delay();
			delay();
			delay();

			LCD_Clear();
			LCD_Display_letter(pos2, 18);                          //S
			LCD_Display_letter(pos3, 4);                          //E
			LCD_Display_letter(pos4, 19);                          //T
			delay();
			delay();
			delay();
			delay();
			delay();

			LCD_Clear();
			int h_1 = (int) time_digit_1;
			int h_2 = (int) time_digit_2;
			int m_1 = (int) time_digit_3;
			int m_2 = (int) time_digit_4;
			time_hours = h_1 * 10 + h_2;
			time_minutes = m_1 * 10 + m_2;
		}

		void change_time_LCD()
		{
			LCD_Clear();
			char digit_1 = (char) (time_hours / 10);
			char digit_2 = (char) (time_hours % 10);
			char digit_3 = (char) (time_minutes / 10);
			char digit_4 = (char) (time_minutes % 10);
			LCD_Display_digit(pos1, digit_1);
			LCD_Display_digit(pos2, digit_2);
			LCD_Display_digit(pos4, digit_3);
			LCD_Display_digit(pos5, digit_4);
		}

		void setAlarmTime()
		{
			char temp = 0;
			LCD_Clear();
			LCD_Display_letter(pos1, 0);                          //A
			LCD_Display_letter(pos2, 11);                         //L
			LCD_Display_letter(pos3, 0);                          //A
			LCD_Display_letter(pos4, 17);                         //R
			LCD_Display_letter(pos5, 12);                         //M

			time_digit_1 = pollKeypad();
			while (time_digit_1 > 2)
			{                          //2
				time_digit_1 = pollKeypad();
			}
			if (time_digit_1 == 2)
			{
				temp = 1;
			}
			LCD_Clear();
			LCD_Display_digit(pos1, time_digit_1);

			i = 100000;                          // SW Delay
			do
				i--;
			while (i != 0);

			time_digit_2 = pollKeypad();
			while ((temp && time_digit_2 > 3) || (!temp && time_digit_2 > 9))
			{
				time_digit_2 = pollKeypad();
			}
			LCD_Clear();
			LCD_Display_digit(pos1, time_digit_1);
			LCD_Display_digit(pos2, time_digit_2);

			i = 100000;                          // SW Delay
			do
				i--;
			while (i != 0);

			time_digit_3 = pollKeypad();
			while (time_digit_3 > 5)
			{                          //5
				time_digit_3 = pollKeypad();
			}
			LCD_Clear();
			LCD_Display_digit(pos1, time_digit_1);
			LCD_Display_digit(pos2, time_digit_2);
			LCD_Display_digit(pos4, time_digit_3);

			i = 100000;                          // SW Delay
			do
				i--;
			while (i != 0);

			time_digit_4 = pollKeypad();
			while (time_digit_4 > 9)
			{
				time_digit_4 = pollKeypad();
			}
			LCD_Clear();
			LCD_Display_digit(pos1, time_digit_1);
			LCD_Display_digit(pos2, time_digit_2);
			LCD_Display_digit(pos4, time_digit_3);
			LCD_Display_digit(pos5, time_digit_4);

			i = 100000;                          // SW Delay
			do
				i--;
			while (i != 0);

			delay();
			delay();
			delay();

			LCD_Clear();
			LCD_Display_letter(pos2, 18);                          //S
			LCD_Display_letter(pos3, 4);                          //E
			LCD_Display_letter(pos4, 19);                          //T
			delay();
			delay();
			delay();
			delay();
			delay();

			LCD_Clear();
			int h_1 = (int) time_digit_1;
			int h_2 = (int) time_digit_2;
			int m_1 = (int) time_digit_3;
			int m_2 = (int) time_digit_4;
			alarm_time_hours = h_1 * 10 + h_2;
			alarm_time_minutes = m_1 * 10 + m_2;
			is_armed = 1;
			GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
			GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN5);
			user_request_arm = 0;
		}

		/* Buzzer PWM Initialization */
		void Init_Buzzer(void)
		{
			/*
			 * The internal timers (TIMER_A) can auto-generate a PWM signal without needing to
			 * flip an output bit every cycle in software. The catch is that it limits which
			 * pins you can use to output the signal, whereas manually flipping an output bit
			 * means it can be on any GPIO. This function populates a data structure that tells
			 * the API to use the timer as a hardware-generated PWM source.
			 *
			 */
			//Generate PWM - Timer runs in Up-Down mode
			param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
			param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
			param.timerPeriod = TIMER_A_PERIOD; //Defined in main.h
			param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
			param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
			param.dutyCycle = HIGH_COUNT; //Defined in main.h

			//BZ1 (defined in main.h) as PWM output
			GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1, GPIO_PIN3,
			GPIO_PRIMARY_MODULE_FUNCTION);
			//Timer_A_outputPWM(TIMER_A0_BASE, &param);
		}

		void main(void)
		{
			//Turn off interrupts during initialization
			__disable_interrupt();
			//Stop watchdog timer
			WDT_A_hold(WDT_A_BASE);
			Init_LCD();
			Init_Buzzer();

			PM5CTL0 &= ~LOCKLPM5;

			//col1
			GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN0);
			//col2
			GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN5);
			//col3
			GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
			//row1
			GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P5, GPIO_PIN1);
			//row2
			GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P8, GPIO_PIN1);
			//row3
			//GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN1);
			//row4
			GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P2, GPIO_PIN7);
			//Set LED1 to output direction
			//GPIO_setAsOutputPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
			//Set LED2 to output direction
			GPIO_setAsOutputPin(GPIO_PORT_LED2, GPIO_PIN_LED2);
			//photosensor
			GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P5, GPIO_PIN2);
			GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P5, GPIO_PIN3);
			GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P5, GPIO_PIN0);
			//LED
			//RED
			GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN3);
			GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
			//YELLOW
			GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN5);
			GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
			//GREEN
			GPIO_setAsOutputPin(GPIO_PORT_P8, GPIO_PIN2);
			GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN2);
			//Enable S1 internal resistance as pull-Up resistance
			GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_S1, GPIO_PIN_S1);
			//Enable S2 internal resistance as pull-Up resistance
			GPIO_setAsPeripheralModuleFunctionInputPin(
					GPIO_PORT_S2, GPIO_PIN_S2,
					GPIO_PRIMARY_MODULE_FUNCTION);
			GPIO_selectInterruptEdge(GPIO_PORT_S2, GPIO_PIN_S2,
			GPIO_HIGH_TO_LOW_TRANSITION);
			GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_S2, GPIO_PIN_S2);
			//S2 interrupt enabled
			GPIO_clearInterrupt(GPIO_PORT_S2, GPIO_PIN_S2);
			GPIO_enableInterrupt(GPIO_PORT_S2, GPIO_PIN_S2);

			//Start timer in continuous mode sourced by SMCLK
			Timer_A_initContinuousModeParam initContParam = { 0 };
			initContParam.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
			initContParam.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
			initContParam.timerInterruptEnable_TAIE =
			TIMER_A_TAIE_INTERRUPT_DISABLE;
			initContParam.timerClear = TIMER_A_DO_CLEAR;
			initContParam.startTimer = false;
			Timer_A_initContinuousMode(TIMER_A1_BASE, &initContParam);

			//Initiaze compare mode
			Timer_A_clearCaptureCompareInterrupt(TIMER_A1_BASE,
			TIMER_A_CAPTURECOMPARE_REGISTER_0);

			Timer_A_initCompareModeParam initCompParam = { 0 };
			initCompParam.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_0;
			initCompParam.compareInterruptEnable =
			TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
			initCompParam.compareOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
			initCompParam.compareValue = COMPARE_VALUE;
			Timer_A_initCompareMode(TIMER_A1_BASE, &initCompParam);

			Timer_A_startCounter( TIMER_A1_BASE,
			TIMER_A_CONTINUOUS_MODE);

			PMM_unlockLPM5();

			//GPIO_setOutputLowOnPin(GPIO_PORT_LED1, GPIO_PIN_LED1);
			GPIO_setOutputLowOnPin(GPIO_PORT_LED2, GPIO_PIN_LED2);

			LCD_Display_letter(pos1, 2);                          //C
			LCD_Display_letter(pos2, 11);                         //L
			LCD_Display_letter(pos3, 14);                         //O
			LCD_Display_letter(pos4, 2);                          //C
			LCD_Display_letter(pos5, 10);                         //K

			setTime();

			setAlarmTime();

			__enable_interrupt();

			while (1)
			{
				//arm the system
				if (armed() && is_armed)
				{
					//when we reach here, alarm has been set
					turn_off();
				}
				if (user_request_arm && !is_armed)
				{
					__disable_interrupt();
					setAlarmTime();
					user_request_arm = 0;
					__enable_interrupt();
				}

			}

			//For debugger
			__no_operation();
		}

#if GPIO_PORT_S2 == GPIO_PORT_P1
//******************************************************************************
//
//This is the PORT1_VECTOR interrupt vector service routine
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
		__interrupt
#elif defined(__GNUC__)
		__attribute__((interrupt(PORT1_VECTOR)))
#endif
		void P1_ISR (void)
#elif GPIO_PORT_S2 == GPIO_PORT_P2
//******************************************************************************
//
//This is the PORT2_VECTOR interrupt vector service routine
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
		__interrupt
#elif defined(__GNUC__)
		__attribute__((interrupt(PORT2_VECTOR)))
#endif
		void P2_ISR(void)
#endif // #if GPIO_PORT_S1
		{
			if (!is_armed)
			{

				user_request_arm = 1;
			}

			GPIO_clearInterrupt(GPIO_PORT_S2, GPIO_PIN_S2);
		}

//******************************************************************************
//
//This is the TIMER1_A0 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_A0_VECTOR
		__interrupt
#elif defined(__GNUC__)
		__attribute__((interrupt(TIMER1_A0_VECTOR)))
#endif
		void TIMER1_A0_ISR(void)
		{
			uint16_t compVal = Timer_A_getCaptureCompareCount(TIMER_A1_BASE,
			TIMER_A_CAPTURECOMPARE_REGISTER_0) + COMPARE_VALUE;
			time_inc++;
			if (time_inc == 21)       //  && !user_request_arm)	//magic number
			{
				//Toggle LED2
				GPIO_toggleOutputOnPin(GPIO_PORT_LED2, GPIO_PIN_LED2);
				time_inc = 0;
				time_seconds++;

				//Update time.
				if (time_seconds == 60)               //(time_seconds == 60)
				{

					time_seconds = 0;
					time_minutes++;
					if (time_minutes == 60)
					{
						time_minutes = 0;
						time_hours++;
						if (time_hours == 24)
						{
							time_hours = 0;
						}
					}
				}
				change_time_LCD();

				if (alarm_time_minutes == time_minutes
						&& alarm_time_hours == time_hours && !tripped)
				{
					is_armed = 0;
					GPIO_setOutputLowOnPin(GPIO_PORT_P8, GPIO_PIN3);
					GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN5);
					GPIO_setOutputHighOnPin(GPIO_PORT_P8, GPIO_PIN2);
				}

			}

			//Add Offset to CCR0
			Timer_A_setCompareValue(TIMER_A1_BASE,
			TIMER_A_CAPTURECOMPARE_REGISTER_0,
									compVal);
		}
