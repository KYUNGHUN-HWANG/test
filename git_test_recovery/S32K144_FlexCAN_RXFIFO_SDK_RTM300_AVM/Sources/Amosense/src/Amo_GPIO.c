#define Amo_GPIO_C_

#include "Cpu.h"
#include "Amo_main.h"
#include "Amo_CAN_Parsing.h"
#include "Amo_LIN.h"
#include "Amo_STATE_Event.h"
#include "Amo_Timer.h"
#include "Amo_GPIO.h"


/**
 * Button interrupt handler
 */

#ifdef KEY_ENABLE
Atcu_32_data Can_data_32_key;

volatile bool button1_pressed = FALSE;
volatile bool button2_pressed = FALSE;
uint16_t key_scan_cnt = 0;

uint32_t button1_state = 0;
uint32_t button2_state = 0;

uint32_t buttonsPressed = 0;

extern uint8_t key_full_left_close;
extern uint8_t key_full_right_close;

void buttonISR(void)
{	
// Check if one of the buttons was pressed
	buttonsPressed = PINS_DRV_GetPortIntFlag(BTN_PORT) &
	((1 << LH_BTN) | (1 << RH_BTN));

	if(buttonsPressed != 0)
	{
		switch (buttonsPressed)
		{
			case 1:
			{		
				button1_state = PINS_DRV_ReadPins(BTN_GPIO);
				if(!(button1_state & (1 << LH_BTN)))
				{
					button1_pressed = TRUE;
					key_scan_cnt = 0;
					PINS_DRV_ClearPinIntFlagCmd(BTN_PORT, LH_BTN);
					Amo_timer_Stop(timer_40);
					Amo_timer_Start(timer_40,100,true,Button_Func);
				}
				else
				{
					button1_pressed = FALSE;
					PINS_DRV_ClearPinIntFlagCmd(BTN_PORT, LH_BTN);
				}
				break;
			}
			case 2:
			{
				button2_state = PINS_DRV_ReadPins(BTN_GPIO);
				if(!(button2_state & (1 << RH_BTN)))
				{
					button2_pressed = TRUE;
					key_scan_cnt = 0;	
					PINS_DRV_ClearPinIntFlagCmd(BTN_PORT, RH_BTN);
					Amo_timer_Stop(timer_40);
					Amo_timer_Start(timer_40,100,true,Button_Func);
				}
				else
				{
					button2_pressed = FALSE;
					PINS_DRV_ClearPinIntFlagCmd(BTN_PORT, RH_BTN);
				}
				break;
			}
			default:
			PINS_DRV_ClearPortIntFlagCmd(BTN_PORT);
			break;
		}
	}
	else
	{

	}
}

void Button_Func(void)
{
	key_scan_cnt = key_scan_cnt +1;

	if((!button1_pressed && !button2_pressed))
	{
		if(key_scan_cnt < 8)
		{
			if(button1_state)
			{
				Amo_timer_Stop(timer_40);
				Evt_queue_add(DEVICE_LH_KEY_PRESS_EVENT);					
			}
			else if(button2_state)
			{
				Amo_timer_Stop(timer_40);
				Evt_queue_add(DEVICE_RH_KEY_PRESS_EVENT);					
			}
			else
			{

			}
			button1_state = 0;				
			button2_state = 0;				
		}
		else if(key_scan_cnt >=8)
		{
//			if(button1_state)
//			{
//				Amo_timer_Stop(timer_40);
//
//				if(!key_full_left_close)
//				{
//					key_full_left_close = 1;
//					Evt_queue_add(DEVICE_LH_LONG_PRESS_EVENT);
//				}
//				else
//				{
//					key_full_left_close = 0;
//					Evt_queue_add(DEVICE_KEY_FOCUS_PRESS_EVENT);
//				}
//
//			}
//			else if(button2_state)
//			{
//				Amo_timer_Stop(timer_40);
//
//				if(!key_full_right_close)
//				{
//					key_full_right_close = 1;
//					Evt_queue_add(DEVICE_RH_LONG_PRESS_EVENT);
//				}
//				else
//				{
//					key_full_right_close = 0;
//					Evt_queue_add(DEVICE_KEY_FOCUS_PRESS_EVENT);
//				}
//			}
//			else
//			{
//
//			}
//			button1_state = 0;
//			button2_state = 0;
		}
		else
		{
			Amo_timer_Stop(timer_40);
			button1_state = 0;
			button2_state = 0;
			//key_full_left_close = 0;
			//key_full_right_close = 0;
		}
	}
	else if(key_scan_cnt >= 8)
	{
		if((button1_state) && (button1_pressed))
		{
			Amo_timer_Stop(timer_40);
			
			if(!key_full_left_close)
			{
				//key_full_left_close = 1;
				Evt_queue_add(DEVICE_LH_LONG_PRESS_EVENT);
			}
			else
			{
				//key_full_left_close = 0;
				Evt_queue_add(DEVICE_KEY_FOCUS_PRESS_EVENT);
			}

			button1_state = 0;
		}
		else if((button2_state) && (button2_pressed))
		{
			Amo_timer_Stop(timer_40);
			
			if(!key_full_right_close)
			{
				//key_full_right_close = 1;
				Evt_queue_add(DEVICE_RH_LONG_PRESS_EVENT);
			}
			else
			{
				//key_full_right_close = 0;
				Evt_queue_add(DEVICE_KEY_FOCUS_PRESS_EVENT);
			}
			button2_state = 0;
		}
		else
		{
			button1_state = 0;				
			button2_state = 0;						
		}
	}
	else
	{

	}
}
#endif

/* @brief Function which configures the LEDs and Buttons*/
void Amo_Gpio_Init(void)
{
    /* Output direction for Pins */

    PINS_DRV_SetPinsDirection(GPIO_PORTC, (CAN_STB_MASK) | (CAN_EN_MASK) | (TEST_OUT1_MASK) /* | (TEST_OUT2_MASK) |*/);

#ifdef KEY_ENABLE
		PINS_DRV_SetPinsDirection(GPIO_PORTE, (LH_FACING_INDI_PWM_MASK));
    PINS_DRV_SetPinsDirection(GPIO_PORTB, (LH_AVOID_INDI_PWM_MASK) | (RH_FACING_INDI_PWM_MASK));
#endif

//		PINS_DRV_SetPinsDirection(GPIO_PORTC,
//#ifdef KEY_ENABLE
//		(RH_AVOID_INDI_PWM_MASK) |
//#endif
//		(CAN_WAKE_MASK) | (LIN3_ENABLE_MASK));
		
    PINS_DRV_SetPinsDirection(GPIO_PORTA, (LIN1_ENABLE_MASK) | (TEST_OUT2_MASK));
    PINS_DRV_SetPinsDirection(GPIO_PORTB, LIN2_ENABLE_MASK);

		PINS_DRV_ClearPins(GPIO_PORTC, TEST_OUT1_MASK);
//		PINS_DRV_ClearPins(GPIO_PORTA, TEST_OUT2_MASK);

#ifdef KEY_ENABLE
    /* Setup button pin */
    PINS_DRV_SetPinsDirection(BTN_GPIO, ~((1 << LH_BTN)|(1 << RH_BTN)));
#endif
    /* Setup button pins interrupt */

#ifdef KEY_ENABLE
    PINS_DRV_SetPinIntSel(BTN_PORT, LH_BTN, PORT_INT_EITHER_EDGE);
    PINS_DRV_SetPinIntSel(BTN_PORT, RH_BTN, PORT_INT_EITHER_EDGE);
#endif

    /* Set Output value CAN_STB */ //Active High
    PINS_DRV_SetPins(GPIO_PORTC, CAN_STB_MASK);
    PINS_DRV_SetPins(GPIO_PORTC, CAN_EN_MASK);
    /* Setup CAN Wake pin */
    PINS_DRV_ClearPins(GPIO_PORTC, CAN_WAKE_MASK);

#ifdef KEY_ENABLE
    PINS_DRV_ClearPins(GPIO_PORTE, LH_FACING_INDI_PWM_MASK);
    PINS_DRV_ClearPins(GPIO_PORTB, LH_AVOID_INDI_PWM_MASK);
    PINS_DRV_ClearPins(GPIO_PORTB, RH_FACING_INDI_PWM_MASK);
    PINS_DRV_ClearPins(GPIO_PORTC, RH_AVOID_INDI_PWM_MASK);
#endif

		/* Wake up LIN transceiver */
		PINS_DRV_SetPins(GPIO_PORTA, LIN1_ENABLE_MASK);
		PINS_DRV_SetPins(GPIO_PORTB, LIN2_ENABLE_MASK);
//		PINS_DRV_SetPins(GPIO_PORTC, LIN3_ENABLE_MASK);

#ifdef KEY_ENABLE
    /* Install buttons ISR */
    INT_SYS_InstallHandler(BTN_PORT_IRQn, &buttonISR, NULL);

    /* Enable buttons interrupt */
    INT_SYS_EnableIRQ(BTN_PORT_IRQn);
#endif
}

