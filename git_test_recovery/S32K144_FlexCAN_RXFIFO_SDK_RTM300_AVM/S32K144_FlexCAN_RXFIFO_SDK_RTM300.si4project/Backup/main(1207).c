/* ###################################################################
**     Filename    : main.c
**     Processor   : S32K1xx
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.00
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
#include "Cpu.h"
#include "clockMan1.h"

#include "dmaController1.h"
#include "pin_mux.h"
#include "watchdog1.h"
#include "pwrMan1.h"
#include "lpTmr1.h"
#include "lin_cfg.h"
#include "lin1.h"
#include "lin_common_api.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stdlib.h"

#include "Amo_CAN_Parsing.h"
#include "ewl_misra_types.h"
#include "Amo_STATE_Event.h"
#include "Amo_Timer.h"
#include "Amo_main.h"
#include "Amo_Adc.h"

volatile int exit_code = 0;

static DeviceStateType Board_state = DEVICE_STATE_INIT;
unsigned long long DeviceEvent = 0;

/* User includes (#include below this line is not maintained by Processor Expert) */
/******************************************************************************
 * Definitions
 ******************************************************************************/

/* This example is setup to work by default with EVB. To use it with other boards
   please comment the following line
*/

/* Definition of power modes indexes, as configured in Power Manager Component
 *  Refer to the Reference Manual for details about the power modes.
 */
#define HSRUN (0u) /* High speed run      */
#define RUN   (1u) /* Run                 */
#define VLPR  (2u) /* Very low power run  */
#define STOP1 (3u) /* Stop option 1       */
#define STOP2 (4u) /* Stop option 2       */
#define VLPS  (5u) /* Very low power stop */

/* LPIT channel used */
#define LPIT_CHANNEL        0UL
#define LPIT_Channel_IRQn   LPIT0_Ch0_IRQn

/* ADC */
extern char Ign2_adc_msg[32];
extern char Ign3_adc_msg[32];
extern char Vatt_adc_msg[32];

extern uint16_t Ign2_adcRawValue;
extern uint16_t Ign3_adcRawValue;
extern uint16_t Vatt_adc_RawValue;

extern uint16_t adcMax;
extern float Ign2_adcValue;
extern float Ign3_adcValue;
extern float Vatt_adcValue;
extern uint16_t Ign2_AdcMinValue;
extern uint16_t Ign3_AdcMinValue;
extern uint16_t Vatt_LimitValue;
extern uint8_t Evnt_IGN2_Onoff;
extern uint8_t Evnt_IGN3_Onoff;

//CAN DATA

/******************************************************************************
 * Global Can value parameter 
 ******************************************************************************/
extern TaskProiority P_Oip_Cts_Select;	
extern uint8_t P_Oip_Cts_priority;
extern Touch_mode Touch_mode_set;
extern flexio_uart_state_t   uartStateTX;
extern flexio_uart_state_t   uartStateRX;

extern Atcu_9_data Can_data_9_Info;

extern uint8_t ATCU_DetentOutStatus;
extern uint8_t ATCU_AutoBrightStatus;
extern uint8_t ATCU_NotMinimumModeBrightStatus;
extern uint8_t L_ATCU_IAUProfileValue;
extern uint8_t L_ATCU_AVNProfileValue;

extern Drivermode ATCU_Fr_Dr_Mode_display;
extern Drivermode ATCU_Fr_Ps_Mode_display;

extern Atcu_22_data Can_data_22_Info;
extern Evnt_mode Hu_Rr_Dr_Vent_Mode;
extern Evnt_mode Hu_Rr_Ps_Vent_Mode;
extern uint8_t Hu_Rr_Dr_Vent_signal;
extern uint8_t Hu_Rr_Ps_Vent_signal;
extern uint16_t Hu_RrDrEVntConsPt_X;
extern uint16_t Hu_RrDrEVntConsPt_Y;
extern uint16_t Hu_RrPsEVntConsPt_X;
extern uint16_t Hu_RrPsEVntConsPt_Y;

extern Atcu_20_data Can_data_20_Info;
extern uint16_t Hu_FrDrEVntSidePt_X;
extern uint16_t Hu_FrDrEVntSidePt_Y;
extern uint16_t Hu_FrDrEVntCtrPt_X;
extern uint16_t Hu_FrDrEVntCtrPt_Y;

extern Atcu_21_data Can_data_21_Info;

extern uint16_t Hu_FrPsEVntSidePt_X;
extern uint16_t Hu_FrPsEVntSidePt_Y;
extern uint16_t Hu_FrPsEVntCtrPt_X;
extern uint16_t Hu_FrPsEVntCtrPt_Y;

extern Atcu_26_data Can_data_26_Info;
extern uint16_t Cts_FrDrEVntSidePt_X;
extern uint16_t Cts_FrDrEVntSidePt_Y;
extern uint16_t Cts_FrDrEVntCtrPt_X;
extern uint16_t Cts_FrDrEVntCtrPt_Y;

extern Atcu_27_data Can_data_27_Info;
extern uint16_t Cts_FrPsEVntSidePt_X;
extern uint16_t Cts_FrPsEVntSidePt_Y;
extern uint16_t Cts_FrPsEVntCtrPt_X;
extern uint16_t Cts_FrPsEVntCtrPt_Y;

extern Atcu_28_data Can_data_28_Info; 
extern Evnt_mode Cts_Rr_Dr_Vent_Mode;
extern Evnt_mode Cts_Rr_Ps_Vent_Mode;
extern uint8_t Cts_Rr_Dr_Vent_siganl;
extern uint8_t Cts_Rr_Ps_Vent_siganl;
extern uint16_t Cts_RrDrEVntConsPt_X;
extern uint16_t Cts_RrDrEVntConsPt_Y;
extern uint16_t Cts_RrPsEVntConsPt_X;
extern uint16_t Cts_RrPsEVntConsPt_Y;

extern Atcu_32_data Can_data_32_Info;
extern Evnt_mode Hu_Fr_Dr_Vent_Mode;
extern Evnt_mode Hu_Fr_Ps_Vent_Mode;
extern uint8_t Hu_Fr_dr_Vent_Side_signal;
extern uint8_t Hu_Fr_dr_Vent_Center_signal;
extern uint8_t Hu_Fr_Ps_Vent_Side_signal;
extern uint8_t Hu_Fr_Ps_Vent_Center_signal;

extern Atcu_33_data Can_data_33_Info;
extern Evnt_mode Cts_Fr_Dr_Vent_Mode;
extern Evnt_mode Cts_Fr_Ps_Vent_Mode;
extern uint8_t Cts_Fr_dr_Vent_Side_siganl;
extern uint8_t Cts_Fr_dr_Vent_Center_signal;
extern uint8_t Cts_Fr_Ps_Vent_Side_signal;
extern uint8_t Cts_Fr_Ps_Vent_Center_signal;


/******************************************************************************
 * Global Can Rx pending 
 ******************************************************************************/

extern uint8_t can_rx_2_expire;
extern uint8_t can_rx_3_expire;
extern uint8_t can_rx_4_expire;
extern uint8_t can_rx_5_expire;
extern uint8_t can_rx_8_expire;
extern uint8_t can_rx_9_expire;
extern uint8_t can_rx_12_expire;
extern uint8_t can_rx_13_expire;
extern uint8_t can_rx_16_expire;
extern uint8_t can_rx_23_expire;
extern uint8_t can_rx_24_expire;
extern uint8_t can_rx_25_expire;
extern uint8_t can_rx_29_expire;
extern uint8_t can_rx_30_expire;
extern uint8_t can_rx_31_expire;

/******************************************************************************
 * Global Can Tx 
 ******************************************************************************/

extern Lvnt_1_data Can_Tx_Evnt_1;
extern Lvnt_2_data Can_Tx_Evnt_2;
extern Lvnt_3_data Can_Tx_Evnt_3;
extern Lvnt_4_data Can_Tx_Evnt_4;
extern Lvnt_5_data Can_Tx_Evnt_5;
extern Lvnt_6_data Can_Tx_Evnt_6;
extern Lvnt_7_data Can_Tx_Evnt_7;
extern Lvnt_8_data Can_Tx_Evnt_8;
extern Lvnt_9_data Can_Tx_Evnt_9;

extern uint8_t Tx_candata[8];
extern Lvnt_1_data Can_Tx_Evnt_1;

/******************************************************************************
 * Variable prototypes
 ******************************************************************************/
uint8_t registration_check = 0;
uint8_t rxMBdone=0;
uint8_t rxFIFOdone=0;

uint8_t sleepFlag = 0;

/* Define user receive buffer */
flexcan_msgbuff_t recvBuff1, recvBuff2;
extern unsigned int can_error_data;
/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void buttonISR(void);
void BoardInit(void);
void GPIOInit(void);
/******************************************************************************
 * Functions
 ******************************************************************************/

int __write_console(__file_handle handle, char *buffer, uint32_t *count)
{
(void)(handle);
 
    //uint32_t bytesRemain;
    uint32_t bytes=*count;
 
    //UART_DRV_SendData(INST_LPUART1, buffer, bytes);
    FLEXIO_UART_DRV_SendDataBlocking(&uartStateTX, (uint8_t *)buffer, bytes, TIMEOUT);
    //while(LPUART_DRV_GetTransmitStatus(INST_LPUART1, &bytesRemain) != STATUS_SUCCESS);
 
    return 0;
}

/* WatchDog IRQ handler */
void WDOG_ISR(void)
{
	/* Turn off both LEDs */
	//PINS_DRV_SetPins(LED_GPIO, (1 << LED0) | (1 << LED1));
	/* Disable the SysTick timer */
	//SysTick_Disable();
}
	

/*
 * @brief : Initialize clocks, pins and power modes
 */
void BoardInit(void)
{

    /* Initialize and configure clocks
     *  -   Setup system clocks, dividers
     *  -   Configure FlexCAN clock, GPIO
     *  -   see clock manager component for more details
     */
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT,
                        g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);

    /* Initialize Power Manager
	 * -   See PowerSettings component for more info
	 */
    POWER_SYS_Init(&powerConfigsArr, POWER_MANAGER_CONFIG_CNT, &powerStaticCallbacksConfigsArr, POWER_MANAGER_CALLBACK_CNT);
    /* Set power mode to HSRUN */
    POWER_SYS_SetMode(HSRUN, POWER_MANAGER_POLICY_AGREEMENT);

    /* Initialize pins
     *  -   Init FlexCAN, and GPIO pins
     *  -   See PinSettings component for more info
     */
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
}


/*! 
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/

int main(void)
{
	/* Write your local variable definition here */
	uint16_t test_counter = 0;
	uint16_t temp_num = 0;
	
	/*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
	#ifdef PEX_RTOS_INIT
	PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
	#endif
	/*** End of Processor Expert internal initialization.                    ***/

	/* Do the initializations required for this application */
	BoardInit();
	Amo_Gpio_Init();

	Flexio_Uart_Init();

	Lptmr_Init();
	Lin_Init();

	/* Initialize Power Manager
	 * -	 See PowerSettings component for more info
	 */
	POWER_SYS_Init(&powerConfigsArr, POWER_MANAGER_CONFIG_CNT, &powerStaticCallbacksConfigsArr, POWER_MANAGER_CALLBACK_CNT);

	Amo_Timer_Init();
	/*Initialize eDMA driver */
	EDMA_DRV_Init(&dmaController1_State, &dmaController1_InitConfig0, edmaChnStateArray, edmaChnConfigArray, EDMA_CONFIGURED_CHANNELS_COUNT);

	Amo_Adc_Init();
	Amo_Can_Init();
	/////////////////////////////////////////////////////////////////////////
	/* Install IRQ handlers for WDOG */
	INT_SYS_InstallHandler(WDOG_EWM_IRQn, WDOG_ISR, (isr_t *)0);
	/* Enable Watchdog IRQ */
	INT_SYS_EnableIRQ(WDOG_EWM_IRQn);
	//////////////////////////////////////////////////////////////////////////
	printf("Air Vent main start\r\n");

	/* Initialize WDOG */
	WDOG_DRV_Init(INST_WATCHDOG1, &watchdog1_Config0);
	Evt_queue_init();

	Start_Evnt_StatusCheck();
	
	while(1)
	{
		//Watch Dog Timer~~~~
		WDOG_DRV_Trigger(INST_WATCHDOG1);
		test_counter++;
		
		if(test_counter == 1000)
		{
			test_counter=0;
		}

		if(rxFIFOdone)		// if message was received into RXFIFO
		{
			rxFIFOdone = 0;

			/* process data from recvBuff2 */
			/* enable receiving data in RX FIFO again */
			FLEXCAN_DRV_RxFifo(INST_CANCOM1,&recvBuff2);

			test_counter = 0;
		}

		switch(Board_state)
		{
			case DEVICE_STATE_INIT:		//Default mode check
			{
					//NV Check & mode & initial define/action ...

					if(registration_check)
					{
						Board_state = DEVICE_STATE_DEFAULT_MODE_INIT;
					}
					else
					{
						Board_state = DEVICE_STATE_LAST_MODE_INIT;							
					}
				break;
			}

			case DEVICE_STATE_MAIN_IDLE:
			{
				/*
				if(work)
				{
				
				}
				else
				{
					Board_state = DEVICE_STATE_SLEEP_INIT;
				}
				*/
					Board_state = DEVICE_STATE_EVT_INIT;
				
				break;
			}

			case DEVICE_STATE_DEFAULT_MODE_INIT:
			{
			
				Board_state = DEVICE_STATE_DEFAULT_MAIN;			
				break;
			}

			case DEVICE_STATE_DEFAULT_MAIN:
			{
				//Default mode action
				Board_state = DEVICE_STATE_MAIN_IDLE;			
				break;
			}

			case DEVICE_STATE_LAST_MODE_INIT:
			{
			
				Board_state = DEVICE_STATE_LAST_MAIN;			
				break;
			}

			case DEVICE_STATE_LAST_MAIN:
			{
				//Last mode action
				Board_state = DEVICE_STATE_MAIN_IDLE;			
				break;
			}

			case DEVICE_STATE_MANUAL_MODE_INIT:
			{
			
				Board_state = DEVICE_STATE_MANUAL_MAIN;			
				break;
			}

			case DEVICE_STATE_MANUAL_MAIN:
			{
				//Manual mode action

//				CanData_Update_Check_FrDrPt(&Can_data_20_Info);
//				FrDrLinData_Parsing(&LIN_LH_EVNT_MASTER_CMD);
//				lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);

				printf("Manual mode %d \r\n", ++temp_num);
				
//				sleepFlag = 1;

				if(P_Oip_Cts_Select == P_OIP_SELECT)
				{
					if(Touch_mode_set == HU_DRIVER_TOUCH)
					{
						Hu_Fr_Dr_touch_mode_setting(Hu_FrDrEVntSidePt_X, Hu_FrDrEVntSidePt_Y, Hu_FrDrEVntCtrPt_X, Hu_FrDrEVntCtrPt_Y);
						
						Amo_timer_Stop(timer_16);
						Amo_timer_Stop(timer_17);
						Amo_timer_Stop(timer_18);
						Amo_timer_Stop(timer_19);
						Amo_timer_Stop(timer_20);
						Amo_timer_Stop(timer_21);

//						CanData_Update_Check_FrDrPt(&Can_data_20_Info);
						FrDrLinData_Parsing(&LIN_LH_EVNT_MASTER_CMD);
						lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);
						
					}
					else if(Touch_mode_set == HU_PASSENGER_TOUCH)
					{
						Hu_Fr_Pass_touch_mode_setting(Hu_FrPsEVntSidePt_X, Hu_FrPsEVntSidePt_Y, Hu_FrPsEVntCtrPt_X, Hu_FrPsEVntCtrPt_Y);

						CanData_Update_Check_FrPsPt(&Can_data_21_Info);
						FrPsLinData_Parsing(&LIN_RH_EVNT_MASTER_CMD);
						lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);
					}
					else if(Touch_mode_set == HU_REAR_TOUCH)
					{
						Hu_Rear_touch_mode_setting(Hu_RrDrEVntConsPt_X, Hu_RrDrEVntConsPt_Y, Hu_RrPsEVntConsPt_X, Hu_RrPsEVntConsPt_Y);

						CanData_Update_Check_RrPt(&Can_data_22_Info);
						RrLinData_Parsing(&LIN_RC_EVNT_MASTER_CMD);
						lin_RrmanualMode_task(&LIN_RC_EVNT_MASTER_CMD);
					}
					else
					{
						
					}
				}
				else
				{
					if(Touch_mode_set == CTS_DRIVER_TOUCH)
					{
						Cts_Fr_Dr_touch_mode_setting(Cts_FrDrEVntSidePt_X, Cts_FrDrEVntSidePt_Y, Cts_FrDrEVntCtrPt_X, Cts_FrDrEVntCtrPt_Y);
					}
					else if(Touch_mode_set == CTS_PASSENGER_TOUCH)
					{
						Cts_Fr_Pass_touch_mode_setting(Cts_FrPsEVntSidePt_X, Cts_FrPsEVntSidePt_Y, Cts_FrPsEVntCtrPt_X, Cts_FrPsEVntCtrPt_Y);
					}
					else if(Touch_mode_set == CTS_REAR_TOUCH)
					{
						Cts_Rear_touch_mode_setting(Cts_RrDrEVntConsPt_X, Cts_RrDrEVntConsPt_Y, Cts_RrPsEVntConsPt_X, Cts_RrPsEVntConsPt_Y);
					}
					else
					{
						
					}
				}

				P_Oip_Cts_priority = 0;
				Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x01;	//APPLY status return
				Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_CTS = 0x01;
				Board_state = DEVICE_STATE_MAIN_IDLE;			
				break;
			}

			case DEVICE_STATE_FULL_CLOSE_MODE_INIT:
			{
			
				Board_state = DEVICE_STATE_FULL_CLOSE_MAIN;			
				break;
			}

			case DEVICE_STATE_FULL_CLOSE_MAIN:
			{
//				sleepFlag = 2;
				
				//FULL Close mode action
				printf("Full close mode \r\n");	

				if(P_Oip_Cts_Select == P_OIP_SELECT)
				{
					Hu_Front_mode_setting();
					HU_Rear_mode_setting();
				}
				else
				{
					Cts_Front_mode_setting();
					Cts_Rear_mode_setting();
				}
				
				P_Oip_Cts_priority = 0;
				Board_state = DEVICE_STATE_MAIN_IDLE;			
				break;
			}

			case DEVICE_STATE_FOCUS_MODE_INIT:
			{
			
				Board_state = DEVICE_STATE_FOCUS_MAIN;			
				break;
			}
			case DEVICE_STATE_FOCUS_MAIN:
			{
				//Focus mode action
				printf("Focus mode \r\n");

				if(P_Oip_Cts_Select == P_OIP_SELECT)
				{
					Hu_Front_mode_setting();
					HU_Rear_mode_setting();
				}
				else
				{
					Cts_Front_mode_setting();
					Cts_Rear_mode_setting();
				}
				P_Oip_Cts_priority = 0;		
				Board_state = DEVICE_STATE_MAIN_IDLE;			
				break;
			}

			case DEVICE_STATE_SPREAD_MODE_INIT:
			{
			
				Board_state = DEVICE_STATE_SPREAD_MAIN;			
				break;
			}
			case DEVICE_STATE_SPREAD_MAIN:
			{
				//Spread mode action
				printf("Spread mode \r\n");

				if(P_Oip_Cts_Select == P_OIP_SELECT)
				{
					Hu_Front_mode_setting();
					HU_Rear_mode_setting();
				}
				else
				{
					Cts_Front_mode_setting();
					Cts_Rear_mode_setting();
				}				
				P_Oip_Cts_priority = 0;
				Board_state = DEVICE_STATE_MAIN_IDLE; 
				break;
			}

			case DEVICE_STATE_CYCLE_MODE_INIT:
			{
			
				Board_state = DEVICE_STATE_CYCLE_MAIN;			
				break;
			}
			case DEVICE_STATE_CYCLE_MAIN:
			{
				//Cycle mode action
				printf("CYCLE mode \r\n");

				if(P_Oip_Cts_Select == P_OIP_SELECT)
				{
					Hu_Front_mode_setting();
					HU_Rear_mode_setting();
				}
				else
				{
					Cts_Front_mode_setting();
					Cts_Rear_mode_setting();
				}					
				P_Oip_Cts_priority = 0;
				Board_state = DEVICE_STATE_MAIN_IDLE;			
				break;
			}

			case DEVICE_STATE_RECOVERY_MODE_INIT:
			{
			
				Board_state = DEVICE_STATE_RECOVERY_MAIN;			
				break;
			}
			case DEVICE_STATE_RECOVERY_MAIN:
			{
				//Recovery mode action
				printf("Recovery_main \r\n");
				Board_state = DEVICE_STATE_MAIN_IDLE;
				break;
			}

			case DEVICE_STATE_EVT_INIT:
			{
				Board_state = DEVICE_STATE_EVT_MAIN;	
				break;
			}

			case DEVICE_STATE_EVT_MAIN:
			{
				DeviceEvent = Evt_queue_value();

				//printf(" = %d \r\n", DeviceEvent);
								
				if(DeviceEvent & DEVICE_KEY_UP_PRESS_EVENT)
				{
					DeviceEvent = DeviceEvent & ~DEVICE_KEY_UP_PRESS_EVENT;

					Board_state = DEVICE_STATE_MANUAL_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_KEY_UP_PRESS_EVENT)
				{
					DeviceEvent = DeviceEvent & ~DEVICE_KEY_UP_PRESS_EVENT;

					Board_state = DEVICE_STATE_MANUAL_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_KEY_LONG_PRESS_EVENT)
				{
					DeviceEvent = DeviceEvent & ~DEVICE_KEY_LONG_PRESS_EVENT;

					Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_KEY_STICK_PRESS_EVENT)
				{
					DeviceEvent = DeviceEvent & ~DEVICE_KEY_STICK_PRESS_EVENT;

					Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_H_KEY_FULL_CLOSE_LIN_CAN_EVENT)  //Full Close mode Hot key LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_H_KEY_FULL_CLOSE_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;		
				}
				else if(DeviceEvent & DEVICE_STICK_FULL_CLOSE_LIN_CAN_EVENT) //Full Close mode STICK LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_STICK_FULL_CLOSE_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_H_KEY_FOCUS_LIN_CAN_EVENT)  //Focus mode Hot key LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_H_KEY_FOCUS_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_FOCUS_MODE_INIT;		
				}
				else if(DeviceEvent & DEVICE_STICK_FOCUS_LIN_CAN_EVENT) //Focus mode STICK LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_STICK_FOCUS_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_FOCUS_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_H_KEY_SPREAD_LIN_CAN_EVENT)  //Spread mode Hot key LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_H_KEY_SPREAD_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_SPREAD_MODE_INIT;		
				}
				else if(DeviceEvent & DEVICE_H_KEY_CYCLE_LIN_CAN_EVENT)  //Cycle mode Hot key LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_H_KEY_CYCLE_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_CYCLE_MODE_INIT;		
				}	
				else if(DeviceEvent & DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT)  //Manual mode input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_MANUAL_MODE_INIT;		
				}
				else if(DeviceEvent & DEVICE_CTS_MANUAL_CAN_LIN_EVENT) //Manual mode input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CTS_MANUAL_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_MANUAL_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_P_OIP_MANUAL_LIN_CAN_EVENT)  //Manual mode input LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_MANUAL_LIN_CAN_EVENT; //Manual mode input LIN->CAN

					Board_state = DEVICE_STATE_MANUAL_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_CTS_MANUAL_LIN_CAN_EVENT)
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CTS_MANUAL_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_MANUAL_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_P_OIP_FULL_CLOSE_CAN_LIN_EVENT)  //Full Close mode input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_FULL_CLOSE_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;		
				}
				else if(DeviceEvent & DEVICE_CTS_FULL_CLOSE_CAN_LIN_EVENT) //Full Close input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CTS_FULL_CLOSE_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_P_OIP_FULL_CLOSE_LIN_CAN_EVENT)  //Full Close mode input LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_FULL_CLOSE_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;		
				}
				else if(DeviceEvent & DEVICE_CTS_FULL_CLOSE_LIN_CAN_EVENT) //Full Close input LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CTS_FULL_CLOSE_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_P_OIP_FOCUS_CAN_LIN_EVENT)  //Focus mode input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_FOCUS_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_FOCUS_MODE_INIT;		
				}
				else if(DeviceEvent & DEVICE_CTS_FOCUS_CAN_LIN_EVENT) //Focus input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CTS_FOCUS_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_FOCUS_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_P_OIP_FOCUS_LIN_CAN_EVENT)  //Focus mode input LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_FOCUS_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_FOCUS_MODE_INIT;		
				}
				else if(DeviceEvent & DEVICE_CTS_FOCUS_LIN_CAN_EVENT) //Focus input LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CTS_FOCUS_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_FOCUS_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_P_OIP_SPREAD_CAN_LIN_EVENT)  //SPREAD mode input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_SPREAD_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_SPREAD_MODE_INIT;		
				}
				else if(DeviceEvent & DEVICE_CTS_SPREAD_CAN_LIN_EVENT) //SPREAD input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CTS_SPREAD_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_SPREAD_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_CTS_FOCUS_LIN_CAN_EVENT)  //SPREAD mode input LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CTS_FOCUS_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_SPREAD_MODE_INIT;		
				}
				else if(DeviceEvent & DEVICE_STICK_FOCUS_LIN_CAN_EVENT) //SPREAD input LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_STICK_FOCUS_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_SPREAD_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_P_OIP_CYCLE_CAN_LIN_EVENT)  //Cycle mode input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_CYCLE_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_CYCLE_MODE_INIT;		
				}
				else if(DeviceEvent & DEVICE_CTS_CYCLE_CAN_LIN_EVENT) //Cycle input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CTS_CYCLE_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_CYCLE_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_P_OIP_CYCLE_LIN_CAN_EVENT)  //Cycle mode input LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_CYCLE_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_CYCLE_MODE_INIT;		
				}
				else if(DeviceEvent & DEVICE_CTS_CYCLE_LIN_CAN_EVENT) //Cycle input LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CTS_CYCLE_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_CYCLE_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_ACTUATOR_RECOVERY_EVENT) //Recovery mode
				{
					DeviceEvent = DeviceEvent & ~DEVICE_ACTUATOR_RECOVERY_EVENT;

					Board_state = DEVICE_STATE_RECOVERY_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_ACTUATOR_RECOVERY_RELEASE_EVENT) //Recovery release
				{
					DeviceEvent = DeviceEvent & ~DEVICE_ACTUATOR_RECOVERY_RELEASE_EVENT;

					Board_state = DEVICE_STATE_RECOVERY_MAIN;
				}
				else if(DeviceEvent & DEVICE_ACTUATOR_SECURE_EVENT) //Recovery mode
				{
					DeviceEvent = DeviceEvent & ~DEVICE_ACTUATOR_SECURE_EVENT;

					Board_state = DEVICE_STATE_SECURE_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_ACTUATOR_SECURE_RELEASE_EVENT) //Recovery release
				{
					DeviceEvent = DeviceEvent & ~DEVICE_ACTUATOR_SECURE_RELEASE_EVENT;

					Board_state = DEVICE_STATE_SECURE_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_CAN_EVNT_UTILITY_MODE_EVENT) //Recovery release
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CAN_EVNT_UTILITY_MODE_EVENT;

					Board_state = DEVICE_STATE_UTILITY_MODE_INIT;
				}				
				else if(DeviceEvent & DEVICE_CAN_RXDATA_TIMEOUT_EVENT)  //RX Timeout
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CAN_RXDATA_TIMEOUT_EVENT;

					Board_state = DEVICE_RX_TIMEOUT_INIT;
				}
				else if(DeviceEvent & DEVICE_CAN_RXDATA_RECOVERY_EVENT)  //RX Recovery
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CAN_RXDATA_RECOVERY_EVENT;

					Board_state = DEVICE_RX_RECOVERY_INIT;
				}			
				else if(DeviceEvent & DEVICE_CAN_EVNT_ATCU_SEND_EVENT)  
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CAN_EVNT_ATCU_SEND_EVENT;

					Board_state = DEVICE_EVNT_ATCU_SEND_INIT;
				}					
				else
				{
					Board_state = DEVICE_STATE_SLEEP_INIT;
				}
				
				break;
			}
			case DEVICE_STATE_KEY_INIT:
			{
				Board_state = DEVICE_STATE_KEY_MAIN;
				break;
			}
			case DEVICE_STATE_KEY_MAIN:
			{
				Board_state = DEVICE_STATE_MAIN_IDLE;	
				break;
			}
			case DEVICE_FAULT_CHECK_INIT:
			{
				Board_state = DEVICE_FAULT_CHECK_MAIN;
				break;
			}
			case DEVICE_FAULT_CHECK_MAIN:
			{
				Board_state = DEVICE_STATE_MAIN_IDLE;	
				break;
			}
			case DEVICE_STATE_SECURE_MODE_INIT:
			{
				Board_state = DEVICE_STATE_SECURE_MAIN;
				break;
			}
			case DEVICE_STATE_SECURE_MAIN:
			{
				Board_state = DEVICE_STATE_MAIN_IDLE;	
				break;
			}			
			case DEVICE_STATE_UTILITY_MODE_INIT:
			{
				Board_state = DEVICE_STATE_UTILITY_MODE_MAIN;	
				break;
			}		
			case DEVICE_STATE_UTILITY_MODE_MAIN:
			{
				Board_state = DEVICE_STATE_MAIN_IDLE;	
				break;
			}				
			case DEVICE_RX_TIMEOUT_INIT:
			{
				Board_state = DEVICE_RX_TIMEOUT_MAIN;
				break;
			}
			case DEVICE_RX_TIMEOUT_MAIN:
			{
				//Actuator stop !!!
				//Actuator stop !!!
				Can_Rx_TimeOut();
				//printf("IGN2 ADC Value = %s adcValue = %f adcRawValue = %d adcMax = %d \r\n", Ign2_adc_msg, Ign2_adcValue, Ign2_adcRawValue, adcMax);
				//printf("IGN3 ADC Value = %s adcValue = %f adcRawValue = %d adcMax = %d \r\n", Ign3_adc_msg, Ign3_adcValue, Ign3_adcRawValue, adcMax);
				//printf("VATT ADC Value  = %s adcValue = %f adcRawValue = %d adcMax = %d \r\n", Vatt_adc_msg, Vatt_adcValue, Vatt_adc_RawValue, adcMax);
				//printf("can_error_status = 0x%x \r\n", can_error_data);
				//printf("Evnt_IGN2_Onoff = %d Vatt_LimitValue = %d Ign2_AdcMinValue = %d \r\n", Evnt_IGN2_Onoff, Vatt_LimitValue, Ign2_AdcMinValue);

				if(can_error_data & FLEXCAN_BusOffFlag)
				{
					printf("Can bus-off error \r\n");
				}
				if(can_error_data & FLEXCAN_ErrorPassiveFlag)
				{
					printf("Can error passive \r\n");
				}
				if(can_error_data & FLEXCAN_StuffingErrorFlag)
				{
					printf("Can Stuffing error \r\n");
				}
				if(can_error_data & FLEXCAN_CrcErrorFlag)
				{
					printf("Can crc error \r\n");
				}
				if(can_error_data & FLEXCAN_AckErrorFlag)
				{
					printf("Can AckErrorFlag error \r\n");
				}
				if(can_error_data & FLEXCAN_FormErrorFlag)
				{
					printf("Can Form error \r\n");
				}
				if(can_error_data & FLEXCAN_BitErrorFlag)
				{
					printf("Can Bit error \r\n");
				}
				
				Board_state = DEVICE_STATE_MAIN_IDLE;	
				break;
			}
			case DEVICE_RX_RECOVERY_INIT:
			{
				Board_state = DEVICE_RX_RECOVERY_MAIN;
				break;
			}
			case DEVICE_RX_RECOVERY_MAIN:
			{
				//ACTUATOR RECOVERY !!!
				//ACTUATOR RECOVERY !!!
				printf("recovery main \r\n");
				CAN_Recovery_TX();					
				Board_state = DEVICE_STATE_MAIN_IDLE;	
				break;
			}			
			case DEVICE_EVNT_ATCU_SEND_INIT:
			{
				Board_state = DEVICE_EVNT_ATCU_SEND_MAIN;
				break;
			}
			case DEVICE_EVNT_ATCU_SEND_MAIN:
			{
				Evnt_Atcu_tx_data();
				Board_state = DEVICE_STATE_MAIN_IDLE;	
				break;
			}				
			case DEVICE_STATE_SLEEP_INIT:
			{
				Board_state = DEVICE_STATE_SLEEP_MAIN;
				break;
			}

			case DEVICE_STATE_SLEEP_MAIN:
			{
				//POWER_SYS_SetMode(VLPS, POWER_MANAGER_POLICY_AGREEMENT);

				if((!Evnt_IGN2_Onoff) && (!Evnt_IGN3_Onoff))
				{
					//Evnt_Sleep();
				}
				Board_state = DEVICE_STATE_MAIN_IDLE;
				break;
			}
			
			default:
			break;
		}

		Flexio_Uart_GetReceive();
//		Check_Evnt_StatusError_Flag();
//		lin_sleep_task();

	}

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the NXP S32K series of microcontrollers.
**
** ###################################################################
*/

