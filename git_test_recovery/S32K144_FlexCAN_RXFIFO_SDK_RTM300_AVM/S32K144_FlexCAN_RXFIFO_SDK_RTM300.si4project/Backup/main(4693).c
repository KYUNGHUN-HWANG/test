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
#include "Amo_Mode_Setting.h"
#include "ewl_misra_types.h"
#include "Amo_STATE_Event.h"
#include "Amo_Timer.h"
#include "Amo_main.h"
#include "Amo_Adc.h"
#include "Amo_GPIO.h"
#include "Amo_Flash.h"
#include "Amo_Nvm.h"
#include "Amo_Sleep.h"
#include "Amo_Cycle.h"

volatile int exit_code = 0;

static DeviceStateType Board_state = DEVICE_STATE_INIT;
unsigned long long DeviceEvent = 0;
extern flash_ssd_config_t flashSSDConfig;
#ifdef AMO_DTC_SETTING
extern uint8_t Dtc_data[200];
#endif

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
extern uint16_t batt_data;
extern uint16_t ign_data;
	
extern uint16_t adcMax;
extern float Ign2_adcValue;
extern float Ign3_adcValue;
extern float Vatt_adcValue;
extern uint16_t Ign2_AdcMinValue;
extern uint16_t Ign3_AdcMinValue;
extern uint16_t Vatt_LimitValue;
extern uint8_t Evnt_VBATT_Onoff;
extern uint8_t Evnt_IGN2_Onoff;
extern uint8_t Evnt_IGN3_Onoff;
extern uint8_t Evnt_Network_Release;
extern uint8_t Evnt_VBATT_Over;
extern uint8_t Evnt_VBATT_Under;
extern uint8_t Evnt_COM_Onoff;
extern uint8_t Evnt_COM_Over;
extern uint8_t Evnt_COM_Under;
//CAN DATA

/******************************************************************************
 * Global Can value parameter 
 ******************************************************************************/
extern TaskProiority P_Oip_Cts_Select;	
extern uint8_t P_Oip_Cts_priority;
extern Touch_mode_Dr Touch_mode_Dr_set;
extern Touch_mode_Ps Touch_mode_Ps_set;
extern Touch_mode_Rr Touch_mode_Rr_set;
extern flexio_uart_state_t   uartStateTX;
//extern flexio_uart_state_t   uartStateRX;

//extern Atcu_9_data Can_data_9_Info;

//extern uint8_t ATCU_DetentOutStatus;
//extern uint8_t ATCU_AutoBrightStatus;
//extern uint8_t ATCU_NotMinimumModeBrightStatus;
extern uint8_t L_ATCU_IAUProfileValue;
extern uint8_t L_ATCU_AVNProfileValue;

extern Drivermode ATCU_Fr_Dr_Mode_display;
extern Drivermode ATCU_Fr_Ps_Mode_display;

//extern Atcu_22_data Can_data_22_Info;
extern Evnt_mode Hu_Rr_Dr_Vent_Mode;
extern Evnt_mode Hu_Rr_Ps_Vent_Mode;
//extern uint8_t Hu_Rr_Dr_Vent_signal;
//extern uint8_t Hu_Rr_Ps_Vent_signal;
//extern uint16_t Hu_RrDrEVntConsPt_X;
//extern uint16_t Hu_RrDrEVntConsPt_Y;
//extern uint16_t Hu_RrPsEVntConsPt_X;
//extern uint16_t Hu_RrPsEVntConsPt_Y;

#if 0
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
#endif

#ifdef AMO_GN7_PE_SETTING_NONE
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
#endif

extern Atcu_32_data Can_data_32_Info;
extern Evnt_mode Hu_Fr_Dr_Vent_Mode;
extern Evnt_mode Hu_Fr_Ps_Vent_Mode;
extern uint8_t Hu_Fr_dr_Vent_Side_signal;
extern uint8_t Hu_Fr_dr_Vent_Center_signal;
extern uint8_t Hu_Fr_Ps_Vent_Side_signal;
extern uint8_t Hu_Fr_Ps_Vent_Center_signal;

#ifdef AMO_GN7_PE_SETTING_NONE
extern Atcu_33_data Can_data_33_Info;
extern Evnt_mode Cts_Fr_Dr_Vent_Mode;
extern Evnt_mode Cts_Fr_Ps_Vent_Mode;
extern uint8_t Cts_Fr_dr_Vent_Side_siganl;
extern uint8_t Cts_Fr_dr_Vent_Center_signal;
extern uint8_t Cts_Fr_Ps_Vent_Side_signal;
extern uint8_t Cts_Fr_Ps_Vent_Center_signal;
#endif
/******************************************************************************
 * Global Can Rx pending 
 ******************************************************************************/
#if 0
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
#endif
/******************************************************************************
 * Global Can Tx 
 ******************************************************************************/

//extern Lvnt_1_data Can_Tx_Evnt_1;
//extern Lvnt_2_data Can_Tx_Evnt_2;
//extern Lvnt_3_data Can_Tx_Evnt_3;
extern Lvnt_4_data Can_Tx_Evnt_4;
//extern Lvnt_5_data Can_Tx_Evnt_5;
//extern Lvnt_6_data Can_Tx_Evnt_6;
//extern Lvnt_7_data Can_Tx_Evnt_7;
//extern Lvnt_8_data Can_Tx_Evnt_8;
//extern Lvnt_9_data Can_Tx_Evnt_9;

extern uint8_t Tx_candata[8];

extern uint8_t FR_SLLR_FailSafe_Flag;
extern uint8_t FR_SLUD_FailSafe_Flag;
extern uint8_t FR_CLLR_FailSafe_Flag;
extern uint8_t FR_CLUD_FailSafe_Flag;

extern uint8_t FR_CRLR_FailSafe_Flag;
extern uint8_t FR_CRUD_FailSafe_Flag;
extern uint8_t FR_SRLR_FailSafe_Flag;
extern uint8_t FR_SRUD_FailSafe_Flag;

/******************************************************************************
 * Variable prototypes
 ******************************************************************************/
uint8_t registration_check = 0;
Profile_mode last_mode_save = 0;

uint8_t rxMBdone=0;
uint8_t rxFIFOdone=0;

uint8_t sleepFlag = 0;
uint8_t cycleModeFlag_cllr = FALSE;
uint8_t cycleModeFlag_clud = FALSE;
uint8_t cycleModeFlag_sllr = FALSE;
uint8_t cycleModeFlag_slud = FALSE;

uint8_t cycleModeFlag_crlr = FALSE;
uint8_t cycleModeFlag_crud = FALSE;
uint8_t cycleModeFlag_srlr = FALSE;
uint8_t cycleModeFlag_srud = FALSE;

uint8_t fullcloseModeFlag_cllr = FALSE;
uint8_t fullcloseModeFlag_clud = FALSE;
uint8_t fullcloseModeFlag_sllr = FALSE;
uint8_t fullcloseModeFlag_slud = FALSE;

uint8_t fullcloseModeFlag_crlr = FALSE;
uint8_t fullcloseModeFlag_crud = FALSE;
uint8_t fullcloseModeFlag_srlr = FALSE;
uint8_t fullcloseModeFlag_srud = FALSE;


uint8_t secureMainFlag = FALSE;
uint8_t rx_recoveryMainFlag = FALSE;

uint8_t secureSpecialCmd_Count = 0;
uint8_t recoveryCmd_Count = 0;
uint8_t rx_Recovery_CurrentSet_Count = 0;


/* Define user receive buffer */
flexcan_msgbuff_t recvBuff1, recvBuff2;
extern unsigned int can_error_data;

#ifdef KEY_ENABLE
extern uint32_t buttonsPressed;

extern uint8_t button_LH_cycle_check;
extern uint8_t button_RH_cycle_check;

uint8_t key_full_left_close = 0;
uint8_t key_full_right_close = 0;
#endif

extern uint32_t set_address;
uint32_t nv_max_cnt = 0;
extern uint8_t nv_page_cnt;
uint16_t test_counter = 0;
extern Drv_type ATCU_DriverSideType;

extern Drv_type ATCU_DriverSideType;

uint8_t Sleep_5second = 0;

extern uint8_t Hu_Fr_dr_Vent_Side_signal_backup;
extern uint8_t Hu_Fr_dr_Vent_Center_signal_backup;
extern uint8_t Hu_Fr_Ps_Vent_Side_signal_backup;
extern uint8_t Hu_Fr_Ps_Vent_Center_signal_backup;

extern uint16_t Hu_FrDrEVntSidePt_X_invalid;
extern uint16_t Hu_FrDrEVntSidePt_Y_invalid;
extern uint16_t Hu_FrDrEVntCtrPt_X_invalid;
extern uint16_t Hu_FrDrEVntCtrPt_Y_invalid;

extern uint8_t gotoZeroPtFlag_cllr;
extern uint8_t lastTryFlag_cllr;

extern uint8_t gotoZeroPtFlag_clud;
extern uint8_t lastTryFlag_clud;

extern uint8_t gotoZeroPtFlag_sllr;
extern uint8_t lastTryFlag_sllr;

extern uint8_t gotoZeroPtFlag_slud;
extern uint8_t lastTryFlag_slud;

extern uint8_t gotoZeroPtFlag_crlr;
extern uint8_t lastTryFlag_crlr;

extern uint8_t gotoZeroPtFlag_crud;
extern uint8_t lastTryFlag_crud;

extern uint8_t gotoZeroPtFlag_srlr;
extern uint8_t lastTryFlag_srlr;

extern uint8_t gotoZeroPtFlag_srud;
extern uint8_t lastTryFlag_srud;

extern uint8_t recoveryPattern_cllr;

extern Recovery_flag_t	recoveryFlags;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
 #ifdef KEY_ENABLE
void buttonISR(void);
 #endif
void BoardInit(void);
void GPIOInit(void);
/******************************************************************************
 * Functions
 ******************************************************************************/
#ifdef FLEX_UART_ON
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
#endif
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
    //POWER_SYS_SetMode(HSRUN, POWER_MANAGER_POLICY_AGREEMENT);
		POWER_SYS_SetMode(RUN, POWER_MANAGER_POLICY_AGREEMENT);

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
	//uint16_t test_counter = 0;
#ifdef AMO_NVM_SETTING
	nvmem_data_type nv;
#endif	
	/*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
	#ifdef PEX_RTOS_INIT
	PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
	#endif
	/*** End of Processor Expert internal initialization.                    ***/

	/* Do the initializations required for this application */
	BoardInit();
	Amo_Gpio_Init();
#ifdef FLEX_UART_ON
	Flexio_Uart_Init();
#endif	
#ifdef AMO_NVM_SETTING
	Amo_Flash_Init();
#endif
	Lptmr_Init();
	Lin_Init();

	/* Initialize Power Manager
	 * -	 See PowerSettings component for more info
	 */
	POWER_SYS_Init(&powerConfigsArr, POWER_MANAGER_CONFIG_CNT, &powerStaticCallbacksConfigsArr, POWER_MANAGER_CALLBACK_CNT);

	Amo_Timer_Init();
	/*Initialize eDMA driver */
	EDMA_DRV_Init(&dmaController1_State, &dmaController1_InitConfig0, edmaChnStateArray, edmaChnConfigArray, EDMA_CONFIGURED_CHANNELS_COUNT);

	/* Install IRQ handlers for WDOG */
	INT_SYS_InstallHandler(WDOG_EWM_IRQn, WDOG_ISR, (isr_t *)0);
	/* Enable Watchdog IRQ */
	INT_SYS_EnableIRQ(WDOG_EWM_IRQn);

	#ifdef DEBUG_MODE
	printf("Air Vent main start\r\n");
	#endif
	Evt_queue_init();

	Start_Evnt_StatusCheck();

#ifdef AMO_NVM_SETTING
	nvmem_init();
#endif
	Amo_Can_Init();
	Amo_Adc_Init(); 
	/* Initialize WDOG */
	WDOG_DRV_Init(INST_WATCHDOG1, &watchdog1_Config0);
	Adc_Value();
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

#if 0 	//def AMO_NVM_SETTING
				nvmem_data_type nv;
				nvmem_read(NVMEM_NV_INIT_IDX,&nv);		
				registration_check = nv.registration;

				nvmem_read(NVMEM_PROFILE_ID_IDX,&nv); 	
				last_mode_save = nv.profile;
#endif

					#if 0
					nvmem_data_type nv;

					nvmem_read(NVMEM_NV_INIT_IDX,&nv);		
					
					registration_check = nv.registration;
					
					if(!registration_check)
					{
						Board_state = DEVICE_STATE_DEFAULT_MODE_INIT;
					}
					else
					{
						nvmem_read(NVMEM_PROFILE_ID_IDX,&nv);		
						last_mode_save = nv.profile;
					
						Board_state = DEVICE_STATE_LAST_MODE_INIT;							
					}
					#endif
					Board_state = DEVICE_STATE_DEFAULT_MODE_INIT;
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
#ifdef AMO_NVM_SETTING
			case DEVICE_STATE_LAST_MAIN:
			{
				//Adc_Value();
				//Last mode action
				/////////////////////////////////////////////////////////////////////////// 
				nvmem_read(&nv);
				//nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
				//nv.nv_item.drive_type = ATCU_DriverSideType;
				//nv.nv_item.profile = last_mode_save;																	//20241217_temp apply delete code
				//#ifdef AMO_DTC_SETTING	
				//nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 637);
				//#else
				//nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 437);
				//#endif
				//nvmem_write(nv.nv_data,PAGE_SIZE);
				////////////////////////////////////////////////////////////////////////////	
				if(Evnt_VBATT_Onoff)
				{
				switch(last_mode_save)
				{
					case DEFAULT_PROFILE:
					{	
						//Last_Setting_Setup(DEFAULT_PROFILE);
						Default_Setting_Setup(DEFAULT_PROFILE);
						break;
					}

					case GUEST:
					{
						if(nv.nv_item.profile_active[GUEST] != 0)
						{
							Last_Setting_Setup(GUEST);
						}
						else
						{
							Default_Setting_Setup(GUEST);
						}
						break;
					}
					case PROFILE_1:
					{			
						if(nv.nv_item.profile_active[PROFILE_1] != 0)
						{
							Last_Setting_Setup(PROFILE_1);
						}
						else
						{
							Default_Setting_Setup(PROFILE_1);
						}
						break;
					}
					case PROFILE_2:
					{
						if(nv.nv_item.profile_active[PROFILE_2] != 0)
						{
							Last_Setting_Setup(PROFILE_2);
						}
						else
						{
							Default_Setting_Setup(PROFILE_2);
						}
						break;
					}
					case PROFILE_3:
					{
						if(nv.nv_item.profile_active[PROFILE_3] != 0)
						{
							Last_Setting_Setup(PROFILE_3);
						}
						else
						{
							Default_Setting_Setup(PROFILE_3);
						}
						break;
					}
					case PROFILE_4:
					{
						if(nv.nv_item.profile_active[PROFILE_4] != 0)
						{
							Last_Setting_Setup(PROFILE_4);
						}					
						else
						{
							Default_Setting_Setup(PROFILE_4);
						}
						break;
					}
					case PROFILE_5:
					{
						if(nv.nv_item.profile_active[PROFILE_5] != 0)
						{
							Last_Setting_Setup(PROFILE_5);
						}					
						else
						{
							Default_Setting_Setup(PROFILE_5);
						}
						break;
					}
					case PROFILE_6:
					{
						if(nv.nv_item.profile_active[PROFILE_6] != 0)
						{
							Last_Setting_Setup(PROFILE_6);
						}					
						else
						{
							Default_Setting_Setup(PROFILE_6);
						}
						break;
					}
					case PROFILE_7:
					{
						if(nv.nv_item.profile_active[PROFILE_7] != 0)
						{
							Last_Setting_Setup(PROFILE_7);
						}					
						else
						{
							Default_Setting_Setup(PROFILE_7);
						}
						break;
					}
					case PROFILE_8:
					{
						if(nv.nv_item.profile_active[PROFILE_8] != 0)
						{
							Last_Setting_Setup(PROFILE_8);
						}					
						else
						{
							Default_Setting_Setup(PROFILE_8);
						}
						break;
					}
					case PROFILE_9:
					{
						if(nv.nv_item.profile_active[PROFILE_9] != 0)
						{
							Last_Setting_Setup(PROFILE_9);
						}					
						else
						{
							Default_Setting_Setup(PROFILE_9);
						}
						break;
					}
					default:
					break;
				}
				}
				Board_state = DEVICE_STATE_MAIN_IDLE;			
				break;
			}
#endif
			case DEVICE_STATE_MANUAL_MODE_INIT:
			{
			
				Board_state = DEVICE_STATE_MANUAL_MAIN;			
				break;
			}

					case DEVICE_STATE_MANUAL_MAIN:
						{
							//Manual mode action
							//Adc_Value();
							#ifdef DEBUG_MODE
							printf("Manual mode \r\n");
							#endif
							cycleModeFlag_sllr = FALSE;
							cycleModeFlag_slud = FALSE;
							cycleModeFlag_cllr = FALSE;
							cycleModeFlag_clud = FALSE;
							cycleModeFlag_crlr = FALSE;
							cycleModeFlag_crud = FALSE;
							cycleModeFlag_srlr = FALSE;
							cycleModeFlag_srud = FALSE;

							fullcloseModeFlag_clud = FALSE;
							fullcloseModeFlag_slud = FALSE;
							fullcloseModeFlag_crud = FALSE;
							fullcloseModeFlag_srud = FALSE;
				

							if(Evnt_VBATT_Onoff)
							{
							if(P_Oip_Cts_Select == P_OIP_SELECT)
							{
								//if((Hu_Fr_Dr_Vent_Mode == FREE_MODE) && (Touch_mode_Dr_set == HU_DRIVER_TOUCH))
								if(Touch_mode_Dr_set == HU_DRIVER_TOUCH)
								{
									Touch_mode_Dr_set = 0;
#ifdef KEY_ENABLE
									PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
									PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif									
	//							lin_FrDrCycleMode_TimerStop();
	//							lin_CycleMode_CanDisp_TimerStop();

									Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
									//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
									if((Hu_FrDrEVntSidePt_X_invalid != 2047) || (Hu_FrDrEVntSidePt_Y_invalid != 2047))
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
										Hu_Fr_dr_Vent_Side_signal_backup = 0x01;
									}

									if((Hu_FrDrEVntCtrPt_X_invalid != 2047) || (Hu_FrDrEVntCtrPt_Y_invalid != 2047))
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Hu_Fr_dr_Vent_Center_signal_backup = 0x01;
									}

									if(ATCU_DriverSideType == LHD)
									{
			//						CanData_Update_Check_FrDrPt(&Can_data_20_Info);
										FrDrLinData_Parsing(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
										lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);
									}
									else if(ATCU_DriverSideType == RHD)
									{
										RHD_FrPsLinData_Parsing(&LIN_RH_EVNT_MASTER_CMD);
										lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);
									}
									else
									{
									}
								}
								
								//if((Hu_Fr_Ps_Vent_Mode == FREE_MODE) && (Touch_mode_Ps_set == HU_PASSENGER_TOUCH))
								if(Touch_mode_Ps_set == HU_PASSENGER_TOUCH)
								{
									Touch_mode_Ps_set = 0;
#ifdef KEY_ENABLE
									PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
									 PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif				
	//									lin_FrPsCycleMode_TimerStop();
	//									lin_CycleMode_CanDisp_TimerStop();

										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x04; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;

									if(ATCU_DriverSideType == LHD)
									{
			//						CanData_Update_Check_FrPsPt(&Can_data_21_Info);
										FrPsLinData_Parsing(&LIN_RH_EVNT_MASTER_CMD);
										lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);
									}
									else if(ATCU_DriverSideType == RHD)
									{
										RHD_FrDrLinData_Parsing(&LIN_LH_EVNT_MASTER_CMD);
										lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);
									}
									else
									{
									}
								}
								
								if(((Hu_Rr_Dr_Vent_Mode == FREE_MODE) || (Hu_Rr_Ps_Vent_Mode == FREE_MODE)) &&(Touch_mode_Rr_set == HU_REAR_TOUCH))
								{			
									lin_RrCycleMode_TimerStop();
									lin_CycleMode_CanDisp_TimerStop();
			
			//						CanData_Update_Check_RrPt(&Can_data_22_Info);
									RrLinData_Parsing(&LIN_RC_EVNT_MASTER_CMD);
									lin_RrmanualMode_task(&LIN_RC_EVNT_MASTER_CMD);
								}
								else
								{
									
								}
								////////////////////////////////////////////////////////////////////////////////////////////
#if 0 //def AMO_NVM_SETTING						
								last_mode_save = L_ATCU_IAUProfileValue; 		//20241217_temp apply delete code
								last_mode_save = L_ATCU_AVNProfileValue; 

								nvmem_read(&nv);
								nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
								nv.nv_item.drive_type = ATCU_DriverSideType;
								nv.nv_item.profile = last_mode_save;

								nv.nv_item.profile_active[last_mode_save] = last_mode_save;
								nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 437);
								nvmem_write(nv.nv_data,PAGE_SIZE);		
								Last_Setting_Save(last_mode_save);
#endif						
								/////////////////////////////////////////////////////////////////////////////////////////////								
							}
							else
							{
								if(Touch_mode_Dr_set == CTS_DRIVER_TOUCH)
								{
									
								}
								else if(Touch_mode_Ps_set == CTS_PASSENGER_TOUCH)
								{
									
								}
								else if(Touch_mode_Rr_set == CTS_REAR_TOUCH)
								{
									
								}
								else
								{
									
								}
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
				//FULL Close mode action
				//Adc_Value();
				#ifdef DEBUG_MODE
				printf("Full close mode \r\n");
				#endif
				cycleModeFlag_sllr = FALSE;
				cycleModeFlag_slud = FALSE;
				cycleModeFlag_cllr = FALSE;
				cycleModeFlag_clud = FALSE;
				cycleModeFlag_crlr = FALSE;
				cycleModeFlag_crud = FALSE;
				cycleModeFlag_srlr = FALSE;
				cycleModeFlag_srud = FALSE;

				fullcloseModeFlag_slud = TRUE;
				fullcloseModeFlag_clud = TRUE;
				fullcloseModeFlag_crud = TRUE;
				fullcloseModeFlag_srud = TRUE;
				
				if(Evnt_VBATT_Onoff)
				{
				if(P_Oip_Cts_Select == P_OIP_SELECT)
				{						
				 if(ATCU_DriverSideType == LHD)
				 {
					Hu_Front_mode_FullClose_setting();
				 }
				 else if(ATCU_DriverSideType == RHD)
				 {
				 	RHD_Hu_Front_mode_FullClose_setting();
				 }
				 else
				 {
				 }
#ifdef AMO_GN7_PE_SETTING_NONE
					HU_Rear_mode_setting();
#endif
					////////////////////////////////////////////////////////////////////////////////////////////
#if 0 //def AMO_NVM_SETTING					
					last_mode_save = L_ATCU_IAUProfileValue;		//20241217_temp apply delete code
					last_mode_save = L_ATCU_AVNProfileValue; 

					nvmem_read(&nv);
					nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
					nv.nv_item.drive_type = ATCU_DriverSideType;
					nv.nv_item.profile = last_mode_save;

					nv.nv_item.profile_active[last_mode_save] = last_mode_save;
					nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 437);
					nvmem_write(nv.nv_data,PAGE_SIZE);		
					Last_Setting_Save(last_mode_save);
#endif					
					/////////////////////////////////////////////////////////////////////////////////////////////
				}
				else
				{
					#ifdef AMO_GN7_PE_SETTING_NONE
					Cts_Front_mode_setting();
					Cts_Rear_mode_setting();
					#endif
				}
				}
				P_Oip_Cts_priority = 0;
				Board_state = DEVICE_STATE_MAIN_IDLE;			
				break;
			}

#ifdef KEY_ENABLE
			case DEVICE_STATE_KEY_FULL_CLOSE_MODE_INIT:
			{
			
				Board_state = DEVICE_STATE_KEY_FULL_CLOSE_MAIN; 		
				break;
			}

			case DEVICE_STATE_KEY_FULL_CLOSE_MAIN:
			{
				#ifdef DEBUG_MODE
				printf("Full key close mode %ld \r\n", buttonsPressed); 
				#endif
				if(buttonsPressed == 1)
				{
					key_full_left_close = 1;
					Hu_Fr_dr_Vent_Side_signal = 0x02;
					Hu_Fr_dr_Vent_Center_signal = 0x02;
				}
				else if(buttonsPressed == 2)
				{
					key_full_right_close = 1;
					Hu_Fr_Ps_Vent_Side_signal = 0x02;
					Hu_Fr_Ps_Vent_Center_signal = 0x02;
				}
				else
				{				
				}
				
				Hu_Front_mode_FullClose_setting();
				#ifdef AMO_GN7_PE_SETTING_NONE
				HU_Rear_mode_setting();
				#endif
				
				Board_state = DEVICE_STATE_MAIN_IDLE; 		
				break;
			}
#endif

			case DEVICE_STATE_FOCUS_MODE_INIT:
			{
			
				Board_state = DEVICE_STATE_FOCUS_MAIN;			
				break;
			}
			case DEVICE_STATE_FOCUS_MAIN:
			{
				//Focus mode action
				//Adc_Value();
				#ifdef DEBUG_MODE
				printf("Focus mode \r\n");
				#endif
				cycleModeFlag_sllr = FALSE;
				cycleModeFlag_slud = FALSE;
				cycleModeFlag_cllr = FALSE;
				cycleModeFlag_clud = FALSE;
				cycleModeFlag_crlr = FALSE;
				cycleModeFlag_crud = FALSE;
				cycleModeFlag_srlr = FALSE;
				cycleModeFlag_srud = FALSE;
				
				fullcloseModeFlag_clud = FALSE;
				fullcloseModeFlag_slud = FALSE;
				fullcloseModeFlag_crud = FALSE;
				fullcloseModeFlag_srud = FALSE;


				if(Evnt_VBATT_Onoff)
				{
				if(P_Oip_Cts_Select == P_OIP_SELECT)
				{	
					if(ATCU_DriverSideType == LHD)
					{
						Hu_Front_Focus_mode_setting();
					}
					else if(ATCU_DriverSideType == RHD)
					{
						RHD_Hu_Front_Focus_mode_setting();
					}
					else
				 {
				 }
#ifdef AMO_GN7_PE_SETTING_NONE
					HU_Rear_mode_setting();
#endif
					////////////////////////////////////////////////////////////////////////////////////////////
#if 0 //def AMO_NVM_SETTING					
					last_mode_save = L_ATCU_IAUProfileValue;		//20241217_temp apply delete code
					last_mode_save = L_ATCU_AVNProfileValue; 

					nvmem_read(&nv);
					nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
					nv.nv_item.drive_type = ATCU_DriverSideType;
					nv.nv_item.profile = last_mode_save;

					nv.nv_item.profile_active[last_mode_save] = last_mode_save;
					nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 437);
					nvmem_write(nv.nv_data,PAGE_SIZE); 
					Last_Setting_Save(last_mode_save);
#endif					
					/////////////////////////////////////////////////////////////////////////////////////////////
				}
				else
				{
					#ifdef AMO_GN7_PE_SETTING_NONE
					Cts_Front_mode_setting();
					Cts_Rear_mode_setting();
					#endif
				}
				}
				P_Oip_Cts_priority = 0;		
				Board_state = DEVICE_STATE_MAIN_IDLE;			
				break;
			}
			
#ifdef KEY_ENABLE
			case DEVICE_STATE_KEY_FOCUS_MODE_INIT:
			{			
				Board_state = DEVICE_STATE_KEY_FOCUS_MAIN;			
				break;
			}
			case DEVICE_STATE_KEY_FOCUS_MAIN:
			{
				#ifdef DEBUG_MODE
				printf("Key Focus mode %ld\r\n", buttonsPressed);
				#endif
				if(buttonsPressed == 1)
				{
					key_full_left_close = 0;
					Hu_Fr_Dr_Vent_Mode = FACING_MODE;
					Hu_Fr_dr_Vent_Side_signal = 0x01;
					Hu_Fr_dr_Vent_Center_signal = 0x01;
				}
				else if(buttonsPressed == 2)
				{
					key_full_right_close = 0;
					Hu_Fr_Ps_Vent_Mode = FACING_MODE;
					Hu_Fr_Ps_Vent_Side_signal = 0x01;
					Hu_Fr_Ps_Vent_Center_signal = 0x01;
				}
				else
				{	
				}
			
				Hu_Front_mode_setting();
				#ifdef AMO_GN7_PE_SETTING_NONE
				HU_Rear_mode_setting();
				#endif
	
				Board_state = DEVICE_STATE_MAIN_IDLE; 		
				break;
			}
#endif	

			case DEVICE_STATE_SPREAD_MODE_INIT:
			{
			
				Board_state = DEVICE_STATE_SPREAD_MAIN;			
				break;
			}
			case DEVICE_STATE_SPREAD_MAIN:
			{
				//Spread mode action
				//Adc_Value();
				#ifdef DEBUG_MODE
				printf("Spread mode \r\n");
				#endif
				cycleModeFlag_sllr = FALSE;
				cycleModeFlag_slud = FALSE;
				cycleModeFlag_cllr = FALSE;
				cycleModeFlag_clud = FALSE;
				cycleModeFlag_crlr = FALSE;
				cycleModeFlag_crud = FALSE;
				cycleModeFlag_srlr = FALSE;
				cycleModeFlag_srud = FALSE;
				
				fullcloseModeFlag_clud = FALSE;
				fullcloseModeFlag_slud = FALSE;
				fullcloseModeFlag_crud = FALSE;
				fullcloseModeFlag_srud = FALSE;



				if(Evnt_VBATT_Onoff)
				{
				if(P_Oip_Cts_Select == P_OIP_SELECT)
				{					
					if(ATCU_DriverSideType == LHD)
					{
						Hu_Front_Spread_mode_setting();
					}
					else if(ATCU_DriverSideType == RHD)
					{
						RHD_Hu_Front_Spread_mode_setting();
					}
					else
				 {
				 }
#ifdef AMO_GN7_PE_SETTING_NONE
					HU_Rear_mode_setting();
#endif
					////////////////////////////////////////////////////////////////////////////////////////////
#if 0 //def AMO_NVM_SETTING					
					last_mode_save = L_ATCU_IAUProfileValue;		//20241217_temp apply delete code
					last_mode_save = L_ATCU_AVNProfileValue; 

					nvmem_read(&nv);
					nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
					nv.nv_item.drive_type = ATCU_DriverSideType;
					nv.nv_item.profile = last_mode_save;

					nv.nv_item.profile_active[last_mode_save] = last_mode_save;
					nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 437);
					nvmem_write(nv.nv_data,PAGE_SIZE);
					Last_Setting_Save(last_mode_save);
#endif					
					/////////////////////////////////////////////////////////////////////////////////////////////
				}
				else
				{
#ifdef AMO_GN7_PE_SETTING_NONE
					Cts_Front_mode_setting();
					Cts_Rear_mode_setting();
#endif
				}				
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
				//Adc_Value();
				#ifdef DEBUG_MODE
				printf("CYCLE mode %d \r\n", last_mode_save);
				#endif
				cycleModeFlag_sllr = TRUE;
				cycleModeFlag_slud = TRUE;
				cycleModeFlag_cllr = TRUE;
				cycleModeFlag_clud = TRUE;
				cycleModeFlag_crlr = TRUE;
				cycleModeFlag_crud = TRUE;
				cycleModeFlag_srlr = TRUE;
				cycleModeFlag_srud = TRUE;
				
				fullcloseModeFlag_clud = FALSE;
				fullcloseModeFlag_slud = FALSE;
				fullcloseModeFlag_crud = FALSE;
				fullcloseModeFlag_srud = FALSE;

				if(Evnt_VBATT_Onoff)
				{
				if(P_Oip_Cts_Select == P_OIP_SELECT)
				{			
					if(ATCU_DriverSideType == LHD)
					{
						Hu_Front_Cycle_mode_setting();
					}
					else if(ATCU_DriverSideType == RHD)
					{
						RHD_Hu_Front_Cycle_mode_setting();
					}
					else
				 {
				 }
#ifdef AMO_GN7_PE_SETTING_NONE
					HU_Rear_mode_setting();
#endif
					////////////////////////////////////////////////////////////////////////////////////////////
#if 0 //def AMO_NVM_SETTING					
					last_mode_save = L_ATCU_IAUProfileValue;		//20241217_temp apply delete code
					last_mode_save = L_ATCU_AVNProfileValue; 

					nvmem_read(&nv);
					nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
					nv.nv_item.drive_type = ATCU_DriverSideType;
					nv.nv_item.profile = last_mode_save;

					nv.nv_item.profile_active[last_mode_save] = last_mode_save;
					nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 437);
					nvmem_write(nv.nv_data,PAGE_SIZE); 
					Last_Setting_Save(last_mode_save);
#endif					
					/////////////////////////////////////////////////////////////////////////////////////////////
				}
				else
				{
#ifdef AMO_GN7_PE_SETTING_NONE
					Cts_Front_mode_setting();
					Cts_Rear_mode_setting();
#endif
				}					
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
				
//				printf("Recovery_main \r\n");

				Lin_Scheduler_SpecialCmd_Start();

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

				if(DeviceEvent & DEVICE_CAN_EVNT_LAST_MODE_EVENT)
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CAN_EVNT_LAST_MODE_EVENT;

					Board_state = DEVICE_STATE_LAST_MODE_INIT;
				}								

#ifdef KEY_ENABLE
				else if(DeviceEvent & DEVICE_LH_KEY_PRESS_EVENT)
				{
					DeviceEvent = DeviceEvent & ~DEVICE_LH_KEY_PRESS_EVENT;

					Board_state = DEVICE_STATE_LH_KEY_INIT;
				}
				else if(DeviceEvent & DEVICE_RH_KEY_PRESS_EVENT)
				{
					DeviceEvent = DeviceEvent & ~DEVICE_RH_KEY_PRESS_EVENT;

					Board_state = DEVICE_STATE_RH_KEY_INIT;
				}
				else if(DeviceEvent & DEVICE_LH_LONG_PRESS_EVENT)
				{
					DeviceEvent = DeviceEvent & ~DEVICE_LH_LONG_PRESS_EVENT;

					Board_state = DEVICE_STATE_KEY_FULL_CLOSE_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_RH_LONG_PRESS_EVENT)
				{
					DeviceEvent = DeviceEvent & ~DEVICE_RH_LONG_PRESS_EVENT;

					Board_state = DEVICE_STATE_KEY_FULL_CLOSE_MODE_INIT;
				}
				else if(DeviceEvent & DEVICE_KEY_FOCUS_PRESS_EVENT)
				{
					DeviceEvent = DeviceEvent & ~DEVICE_KEY_FOCUS_PRESS_EVENT;

					Board_state = DEVICE_STATE_KEY_FOCUS_MODE_INIT;
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
#endif

				else if(DeviceEvent & DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT)  //Manual mode input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_MANUAL_MODE_INIT;		
				}
				#ifdef AMO_GN7_PE_SETTING_NONE
				else if(DeviceEvent & DEVICE_CTS_MANUAL_CAN_LIN_EVENT) //Manual mode input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CTS_MANUAL_CAN_LIN_EVENT;
				
					Board_state = DEVICE_STATE_MANUAL_MODE_INIT;
				}
				#endif
				else if(DeviceEvent & DEVICE_P_OIP_MANUAL_LIN_CAN_EVENT)  //Manual mode input LIN->CAN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_MANUAL_LIN_CAN_EVENT; //Manual mode input LIN->CAN

					Board_state = DEVICE_STATE_MANUAL_MODE_INIT;
				}
				#ifdef AMO_GN7_PE_SETTING_NONE
				else if(DeviceEvent & DEVICE_CTS_MANUAL_LIN_CAN_EVENT)
				{
					DeviceEvent = DeviceEvent & ~DEVICE_CTS_MANUAL_LIN_CAN_EVENT;

					Board_state = DEVICE_STATE_MANUAL_MODE_INIT;
				}
				#endif
				else if(DeviceEvent & DEVICE_P_OIP_FULL_CLOSE_CAN_LIN_EVENT)  //Full Close mode input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_FULL_CLOSE_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;		
				}
				#ifdef AMO_GN7_PE_SETTING_NONE
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
				#endif
				else if(DeviceEvent & DEVICE_P_OIP_FOCUS_CAN_LIN_EVENT)  //Focus mode input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_FOCUS_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_FOCUS_MODE_INIT;		
				}
				#ifdef AMO_GN7_PE_SETTING_NONE
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
				#endif
				else if(DeviceEvent & DEVICE_P_OIP_SPREAD_CAN_LIN_EVENT)  //SPREAD mode input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_SPREAD_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_SPREAD_MODE_INIT;		
				}
				#ifdef AMO_GN7_PE_SETTING_NONE
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
				#endif
				else if(DeviceEvent & DEVICE_P_OIP_CYCLE_CAN_LIN_EVENT)  //Cycle mode input CAN->LIN
				{
					DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_CYCLE_CAN_LIN_EVENT;

					Board_state = DEVICE_STATE_CYCLE_MODE_INIT;		
				}
				#ifdef AMO_GN7_PE_SETTING_NONE
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
				#endif
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
				else if(DeviceEvent & DEVICE_EVNT_DTC_EVENT)  
				{
					DeviceEvent = DeviceEvent & ~DEVICE_EVNT_DTC_EVENT;

					Board_state = DEVICE_STATC_DTC_MAIN;
				}	
				else if(DeviceEvent & DEVICE_EVNT_OTA_EVENT)  
				{
					DeviceEvent = DeviceEvent & ~DEVICE_EVNT_OTA_EVENT;

					Board_state = DEVICE_STATC_FW_OTA_MAIN;
				}	
				else if(DeviceEvent & DEVICE_SLEEP_ENTER_EVENT)  
				{
					DeviceEvent = DeviceEvent & ~DEVICE_SLEEP_ENTER_EVENT;

					Board_state = DEVICE_STATE_SLEEP_ENTER;
				}					
				else
				{
					Board_state = DEVICE_STATE_SLEEP_INIT;
				}
				
				break;
			}

#ifdef KEY_ENABLE
			case DEVICE_STATE_LH_KEY_INIT:
			{
				Board_state = DEVICE_STATE_LH_KEY_MAIN;
				break;
			}
			case DEVICE_STATE_LH_KEY_MAIN:
			{
				#ifdef DEBUG_MODE
				printf("DEVICE_STATE_LH_KEY_MAIN %ld\r\n", buttonsPressed);
				#endif
				if(key_full_left_close)
				{
					Board_state = DEVICE_STATE_KEY_FOCUS_MODE_INIT;
				}
				else
				{
					if(Hu_Fr_Dr_Vent_Mode == DEFAULT_MODE ||Hu_Fr_Dr_Vent_Mode == FREE_MODE || Hu_Fr_Dr_Vent_Mode == SWING_MODE)
					{
					Hu_Fr_Dr_Vent_Mode = FACING_MODE;
					Hu_Fr_dr_Vent_Center_signal = 0x01;
					Hu_Fr_dr_Vent_Side_signal = 0x01;
					Hu_Front_mode_setting();
					HU_Rear_mode_setting();
					}
					else if(Hu_Fr_Dr_Vent_Mode == FACING_MODE)
					{
					Hu_Fr_Dr_Vent_Mode = AVOID_MODE;
					Hu_Fr_dr_Vent_Center_signal = 0x01;
					Hu_Fr_dr_Vent_Side_signal = 0x01;
					Hu_Front_mode_setting();
					HU_Rear_mode_setting();
					}
					else if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE)
					{
					button_idx_1 = 0;
					Hu_Fr_Dr_Vent_Mode = FREE_MODE;
					Hu_Fr_dr_Vent_Center_signal = 0x01;
					Hu_Fr_dr_Vent_Side_signal = 0x01;
					Hu_Front_mode_manual_key_setting();
					HU_Rear_mode_setting();
					}

					Board_state = DEVICE_STATE_MAIN_IDLE;
				}
				break;
			}
			case DEVICE_STATE_RH_KEY_INIT:
			{
				Board_state = DEVICE_STATE_RH_KEY_MAIN;
				break;
			}
			case DEVICE_STATE_RH_KEY_MAIN:
			{
				#ifdef DEBUG_MODE
				printf("DEVICE_STATE_LH_KEY_MAIN %ld\r\n", buttonsPressed);
				#endif
				if(key_full_right_close)
				{
					Board_state = DEVICE_STATE_KEY_FOCUS_MODE_INIT;
				}
				else
				{
					if(Hu_Fr_Ps_Vent_Mode == DEFAULT_MODE ||Hu_Fr_Ps_Vent_Mode == FREE_MODE || Hu_Fr_Ps_Vent_Mode == SWING_MODE)
					{				
					Hu_Fr_Ps_Vent_Mode = FACING_MODE;
					Hu_Fr_Ps_Vent_Center_signal = 0x01;
					Hu_Fr_Ps_Vent_Side_signal = 0x01;		
					Hu_Front_mode_setting();
					HU_Rear_mode_setting();
					}
					else if(Hu_Fr_Ps_Vent_Mode == FACING_MODE)
					{
					Hu_Fr_Ps_Vent_Mode = AVOID_MODE;
					Hu_Fr_Ps_Vent_Center_signal = 0x01;
					Hu_Fr_Ps_Vent_Side_signal = 0x01;
					Hu_Front_mode_setting();
					HU_Rear_mode_setting();
					}
					else if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE)
					{
					button_idx_2 = 0;
					Hu_Fr_Ps_Vent_Mode = FREE_MODE;
					Hu_Fr_Ps_Vent_Center_signal = 0x01;
					Hu_Fr_Ps_Vent_Side_signal = 0x01;
					Hu_Front_mode_manual_key_setting();
					HU_Rear_mode_setting();
					}
					Board_state = DEVICE_STATE_MAIN_IDLE;	
				}
				break;
			}
#endif

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
				#ifdef DEBUG_MODE
				printf("Acturator Stop !!! \r\n");
				#endif
				FR_SLLR_FailSafe_Flag = TRUE;
				FR_SLUD_FailSafe_Flag = TRUE;
				FR_CLLR_FailSafe_Flag = TRUE;
				FR_CLUD_FailSafe_Flag = TRUE;

				FR_CRLR_FailSafe_Flag = TRUE;
				FR_CRUD_FailSafe_Flag = TRUE;
				FR_SRLR_FailSafe_Flag = TRUE;
				FR_SRUD_FailSafe_Flag = TRUE;

				recoveryFlags.FR_SLLR_Recov_Flag = TRUE;
				recoveryFlags.FR_SLUD_Recov_Flag = TRUE;
				recoveryFlags.FR_CLLR_Recov_Flag = TRUE;
				recoveryFlags.FR_CLUD_Recov_Flag = TRUE;
				
				recoveryFlags.FR_SRLR_Recov_Flag = TRUE;
				recoveryFlags.FR_SRUD_Recov_Flag = TRUE;
				recoveryFlags.FR_CRLR_Recov_Flag = TRUE;
				recoveryFlags.FR_CRUD_Recov_Flag = TRUE;

				gotoZeroPtFlag_cllr = FALSE;
				lastTryFlag_cllr = TRUE;
				gotoZeroPtFlag_clud = FALSE;
				lastTryFlag_clud = TRUE;
				gotoZeroPtFlag_sllr = FALSE;
				lastTryFlag_sllr = TRUE;
				gotoZeroPtFlag_slud = FALSE;
				lastTryFlag_slud = TRUE;
				
				gotoZeroPtFlag_crlr = FALSE;
				lastTryFlag_crlr = TRUE;
				gotoZeroPtFlag_crud = FALSE;
				lastTryFlag_crud = TRUE;
				gotoZeroPtFlag_srlr = FALSE;
				lastTryFlag_srlr = TRUE;
				gotoZeroPtFlag_srud = FALSE;
				lastTryFlag_srud = TRUE;

				secureMainFlag = TRUE;
				secureSpecialCmd_Count++;

				rx_Recovery_CurrentSet_Count = 0;
				
				//Last_Setting_Save(current_mode_save);
					
//				Lin_Scheduler_SpecialCmd_Start();

				if(secureSpecialCmd_Count == 1)
				{
					Lin_Scheduler_SpecialCmd_Start_1();

					if(cycleModeFlag_cllr == TRUE) { lin_FrDrCycleMode_TimerStop(); }
					if(cycleModeFlag_clud == TRUE) { lin_FrDrCycleMode_TimerStop(); }
					if(cycleModeFlag_sllr == TRUE) { lin_FrDrCycleMode_TimerStop(); }
					if(cycleModeFlag_slud == TRUE) { lin_FrDrCycleMode_TimerStop(); }

					if(cycleModeFlag_crlr == TRUE) { lin_FrPsCycleMode_TimerStop(); }
					if(cycleModeFlag_crud == TRUE) { lin_FrPsCycleMode_TimerStop(); }
					if(cycleModeFlag_srlr == TRUE) { lin_FrPsCycleMode_TimerStop(); }
					if(cycleModeFlag_srud == TRUE) { lin_FrPsCycleMode_TimerStop(); }
				}
				
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
				//Can_Rx_TimeOut();
				#ifdef DEBUG_MODE
				//printf("Evnt_IGN2_Onoff = %d IGN2 ADC Value = %s adcValue = %f adcRawValue = %d adcMax = %d \r\n", Evnt_IGN2_Onoff, Ign2_adc_msg, Ign2_adcValue, Ign2_adcRawValue, adcMax);
				//printf("IGN3 ADC Value = %s adcValue = %f adcRawValue = %d adcMax = %d \r\n", Ign3_adc_msg, Ign3_adcValue, Ign3_adcRawValue, adcMax);
				//printf("VATT adcValue = %f adcRawValue = %d adcMax = %d Vatt_LimitValue = %d Evnt_VBATT_Onoff = %d \r\n", Vatt_adcValue, Vatt_adc_RawValue, adcMax, Vatt_LimitValue, Evnt_VBATT_Onoff);
				//printf("Evnt_VBATT_Over = %d Evnt_VBATT_Under = %d \r\n", Evnt_VBATT_Over, Evnt_VBATT_Under);
				//printf("Evnt_COM_Onoff = %d Evnt_COM_Over = %d Evnt_COM_Under = %d \r\n", Evnt_COM_Onoff, Evnt_COM_Over, Evnt_COM_Under);
				//printf("can_error_status = 0x%x \r\n", can_error_data);
				//printf("Evnt_IGN2_Onoff = %d Vatt_LimitValue = %d Ign2_AdcMinValue = %d \r\n", Evnt_IGN2_Onoff, Vatt_LimitValue, Ign2_AdcMinValue);
				//printf("batt_cal_data = %d ign2_cal_data = %d \r\n", batt_data, ign_data);

#if 0//def AMO_NVM_SETTING
				nvmem_read(&nv);
				nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
				nv_max_cnt = nv.nv_item.write_cnt;
				nv.nv_item.drive_type = ATCU_DriverSideType;
				nv.nv_item.profile = last_mode_save;

				nv.nv_item.profile_active[last_mode_save] = last_mode_save;
				nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 437);
				nvmem_write(nv.nv_data,PAGE_SIZE); 
				printf("1st nv_max_cnt = %ld nv_page_cnt = %d address = %X \r\n", nv_max_cnt, nv_page_cnt,(int)set_address);
				//Last_Setting_Save(last_mode_save);

				nv.nv_item.drive_type = ATCU_DriverSideType;
				nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
				nv_max_cnt = nv.nv_item.write_cnt;
				nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 437);
				nvmem_write(nv.nv_data, PAGE_SIZE);
				printf("2st nv_max_cnt = %ld nv_page_cnt = %d address = %X \r\n", nv_max_cnt, nv_page_cnt,(int)set_address);
#endif

				//printf("Evnt_IGN2_Onoff = %d Evnt_Network_Release = %d \r\n", Evnt_IGN2_Onoff, Evnt_Network_Release);
				//printf(" %d  %d  %d  %d \r\n", Hu_Fr_dr_Vent_Side_signal_backup,Hu_Fr_dr_Vent_Center_signal_backup, Hu_Fr_Ps_Vent_Side_signal_backup,Hu_Fr_Ps_Vent_Center_signal_backup );
				#endif

				#if 0
				if(can_error_data & CAN_ESR1_BOFFINT_MASK)
				{
					printf("Can bus-off error \r\n");
				}
				if(can_error_data & CAN_ESR1_FLTCONF(2))
				{
					printf("Can error passive \r\n");
				}
				if(can_error_data & CAN_ESR1_STFERR_MASK)
				{
					printf("Can Stuffing error \r\n");
				}
				if(can_error_data & CAN_ESR1_CRCERR_MASK)
				{
					printf("Can crc error \r\n");
				}
				if(can_error_data & CAN_ESR1_ACKERR_MASK)
				{
					printf("Can AckErrorFlag error \r\n");
				}
				if(can_error_data & CAN_ESR1_FRMERR_MASK)
				{
					printf("Can Form error \r\n");
				}
				if(can_error_data & CAN_ESR1_BIT0ERR_MASK)
				{
					printf("Can Bit 0 error \r\n");
				}

				if(can_error_data & CAN_ESR1_BIT1ERR_MASK)
				{
					printf("Can Bit 1 error \r\n");
				}				

				can_error_data = FLEXCAN_DRV_GetErrorStatus(INST_CANCOM1);
				#endif
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
				#ifdef DEBUG_MODE
				printf("recovery main \r\n");
				#endif				
				//CAN_Recovery_TX();					

				secureMainFlag = FALSE;
				rx_recoveryMainFlag = TRUE;

				recoveryPattern_cllr = Recovery_None;
				
				secureSpecialCmd_Count = 0;
				rx_Recovery_CurrentSet_Count++;

				FR_DTC_Flt_flag.FR_EXT_FLT.fltByte = false;

				if(rx_Recovery_CurrentSet_Count == 1)
				{
					Current_Setting_Setup();
				}
				
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
				//Adc_Value();
				if(Evnt_COM_Onoff)
				{
				Evnt_Atcu_tx_data();
				}
				Board_state = DEVICE_STATE_MAIN_IDLE;	
				break;
			}
			case DEVICE_STATC_DTC_MAIN:
			{
				DTC_Send_event();
				Board_state = DEVICE_STATE_MAIN_IDLE;	
				break;
			}
			case DEVICE_STATC_FW_OTA_MAIN:
			{
				#ifdef DEBUG_ON
				//printf("FW OTA \r\n");
				#endif
				DTC_OTA_event();
				Board_state = DEVICE_STATE_MAIN_IDLE;	
				break;
			}					
			case DEVICE_STATE_SLEEP_INIT:
			{
				Board_state = DEVICE_STATE_SLEEP_MAIN;
				break;
			}
			case DEVICE_STATE_SLEEP_ENTER:
			{
				nvmem_read(&nv);
				nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
				nv_max_cnt = nv.nv_item.write_cnt;
				nv.nv_item.drive_type = ATCU_DriverSideType;
				nv.nv_item.profile = last_mode_save;
				
				nv.nv_item.profile_active[last_mode_save] = last_mode_save;
#ifdef AMO_DTC_SETTING	
				nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 637);
#else
				nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 441);
#endif
				nvmem_write(nv.nv_data,PAGE_SIZE);			
				Last_Setting_Save(last_mode_save);
			
				PINS_DRV_ClearPins(GPIO_PORTC, CAN_STB_MASK);
				Board_state = DEVICE_STATE_MAIN_IDLE;
				break;
			}			
			case DEVICE_STATE_SLEEP_MAIN:
			{
#ifdef AMO_GN7_PE_SETTING_NONE
			if(((!Evnt_IGN2_Onoff) && (!Evnt_IGN3_Onoff)) && (!Evnt_Network_Release))
#else
			if((!Evnt_IGN2_Onoff) && (!Evnt_Network_Release))
#endif
			{
				if(!Sleep_5second)
				{
					if(L_ATCU_IAUProfileValue == L_ATCU_AVNProfileValue)
					{
						last_mode_save = L_ATCU_AVNProfileValue;
					}
					else
					{
						last_mode_save = L_ATCU_AVNProfileValue;	
					}
					
					lin_FrDrCycleMode_TimerStop();
					lin_CycleMode_CanDisp_TimerStop();
					lin_FrPsCycleMode_TimerStop();

					Sleep_5second = 1;					
					Lin_GoTo_Sleep();

					Amo_timer_Stop(timer_82);
					Amo_timer_Start(timer_82, 5000, false, Amo_Sleep);				
					//Amo_Sleep();
				}
			}
			else
			{
				Sleep_5second = 0;
				Amo_timer_Stop(timer_82);
			}
				Board_state = DEVICE_STATE_MAIN_IDLE;
				break;
			}
			
			default:
			break;
		}
//		Flexio_Uart_GetReceive();

		Handle_RecoveryMode_Steploss();

//		Steploss_TimeOut_Stop(&LIN_RH_FR_STATUS, &LIN_EVNT_PT);

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

