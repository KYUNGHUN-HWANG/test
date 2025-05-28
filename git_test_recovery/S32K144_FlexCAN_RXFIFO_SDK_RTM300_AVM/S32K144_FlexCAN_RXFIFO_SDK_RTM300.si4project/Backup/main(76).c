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

/******************************************************************************
 * Global Can value parameter 
 ******************************************************************************/
flexcan_msgbuff_t recvBuff1, recvBuff2;
extern unsigned int can_error_data;
extern TaskProiority P_Oip_Cts_Select;	
extern uint8_t P_Oip_Cts_priority;
extern Touch_mode_Dr Touch_mode_Dr_set;
extern Touch_mode_Ps Touch_mode_Ps_set;
extern Touch_mode_Rr Touch_mode_Rr_set;
extern flexio_uart_state_t   uartStateTX;
extern uint8_t Evnt_VBATT_Onoff, Evnt_IGN2_Onoff,Evnt_IGN3_Onoff,Evnt_Network_Release,Evnt_VBATT_Over,Evnt_VBATT_Under,Evnt_COM_Onoff,Evnt_COM_Over,Evnt_COM_Under;
extern uint8_t L_ATCU_IAUProfileValue,L_ATCU_AVNProfileValue;
//extern uint16_t Ign2_adcRawValue, Ign3_adcRawValue,Vatt_adc_RawValue,batt_data,ign_data, adcMax,Ign2_AdcMinValue,Ign3_AdcMinValue,Vatt_LimitValue;
extern float Ign2_adcValue, Ign3_adcValue, Vatt_adcValue;
extern Drivermode ATCU_Fr_Dr_Mode_display,ATCU_Fr_Ps_Mode_display;
extern Evnt_mode Hu_Rr_Dr_Vent_Mode,Hu_Rr_Ps_Vent_Mode;
#ifdef AMO_GN7_PE_SETTING_NONE
extern Atcu_26_data Can_data_26_Info;
extern uint16_t Cts_FrDrEVntSidePt_X,Cts_FrDrEVntSidePt_Y,Cts_FrDrEVntCtrPt_X,Cts_FrDrEVntCtrPt_Y;
extern Atcu_27_data Can_data_27_Info;
extern uint16_t Cts_FrPsEVntSidePt_X,Cts_FrPsEVntSidePt_Y,Cts_FrPsEVntCtrPt_X,Cts_FrPsEVntCtrPt_Y;
extern Atcu_28_data Can_data_28_Info; 
extern Evnt_mode Cts_Rr_Dr_Vent_Mode;
extern Evnt_mode Cts_Rr_Ps_Vent_Mode;
extern uint8_t Cts_Rr_Dr_Vent_siganl,Cts_Rr_Ps_Vent_siganl;
extern uint16_t Cts_RrDrEVntConsPt_X,Cts_RrDrEVntConsPt_Y,Cts_RrPsEVntConsPt_X,Cts_RrPsEVntConsPt_Y;
#endif
extern Atcu_32_data Can_data_32_Info;
extern Evnt_mode Hu_Fr_Dr_Vent_Mode,Hu_Fr_Ps_Vent_Mode;
extern uint8_t Hu_Fr_dr_Vent_Side_signal,Hu_Fr_dr_Vent_Center_signal,Hu_Fr_Ps_Vent_Side_signal,Hu_Fr_Ps_Vent_Center_signal;
#ifdef AMO_GN7_PE_SETTING_NONE
extern Atcu_33_data Can_data_33_Info;
extern Evnt_mode Cts_Fr_Dr_Vent_Mode;
extern Evnt_mode Cts_Fr_Ps_Vent_Mode;
extern uint8_t Cts_Fr_dr_Vent_Side_siganl,Cts_Fr_dr_Vent_Center_signal,Cts_Fr_Ps_Vent_Side_signal,Cts_Fr_Ps_Vent_Center_signal;
#endif
extern Drv_type ATCU_DriverSideType,ATCU_DriverSideType;
extern uint32_t set_address;
extern uint8_t nv_page_cnt;
extern uint8_t Hu_Fr_dr_Vent_Side_signal_backup,Hu_Fr_dr_Vent_Center_signal_backup,Hu_Fr_Ps_Vent_Side_signal_backup,Hu_Fr_Ps_Vent_Center_signal_backup;
extern uint16_t Hu_FrDrEVntSidePt_X_invalid,Hu_FrDrEVntSidePt_Y_invalid,Hu_FrDrEVntCtrPt_X_invalid,Hu_FrDrEVntCtrPt_Y_invalid;
extern uint8_t gotoZeroPtFlag_cllr,lastTryFlag_cllr,gotoZeroPtFlag_clud,lastTryFlag_clud,gotoZeroPtFlag_sllr,lastTryFlag_sllr,gotoZeroPtFlag_slud,lastTryFlag_slud;
extern uint8_t gotoZeroPtFlag_crlr,lastTryFlag_crlr,gotoZeroPtFlag_crud,lastTryFlag_crud,gotoZeroPtFlag_srlr,lastTryFlag_srlr,gotoZeroPtFlag_srud,lastTryFlag_srud;
extern uint8_t recoveryPattern_sllr,recoveryPattern_slud,recoveryPattern_cllr,recoveryPattern_clud,recoveryPattern_crlr,recoveryPattern_crud,recoveryPattern_srlr,recoveryPattern_srud;
extern Recovery_flag_t	recoveryFlags;
/******************************************************************************
 * Global Can Tx 
 ******************************************************************************/
extern Lvnt_4_data Can_Tx_Evnt_4;
extern uint8_t Tx_candata[8],FR_SLLR_FailSafe_Flag,FR_SLUD_FailSafe_Flag,FR_CLLR_FailSafe_Flag,FR_CLUD_FailSafe_Flag;
extern uint8_t FR_CRLR_FailSafe_Flag,FR_CRUD_FailSafe_Flag,FR_SRLR_FailSafe_Flag,FR_SRUD_FailSafe_Flag;
/******************************************************************************
 * Variable prototypes
 ******************************************************************************/
Profile_mode last_mode_save = 0;
uint8_t rxMBdone=0,rxFIFOdone=0,Sleep_5second = 0,registration_check = 0;
uint8_t sleepFlag = 0,cycleModeFlag_cllr = FALSE,cycleModeFlag_clud = FALSE,cycleModeFlag_sllr = FALSE,cycleModeFlag_slud = FALSE;
uint8_t cycleModeFlag_crlr = FALSE,cycleModeFlag_crud = FALSE,cycleModeFlag_srlr = FALSE,cycleModeFlag_srud = FALSE;
uint8_t fullcloseModeFlag_cllr = FALSE,fullcloseModeFlag_clud = FALSE,fullcloseModeFlag_sllr = FALSE,fullcloseModeFlag_slud = FALSE;
uint8_t fullcloseModeFlag_crlr = FALSE,fullcloseModeFlag_crud = FALSE,fullcloseModeFlag_srlr = FALSE,fullcloseModeFlag_srud = FALSE;
uint8_t secureMainFlag = FALSE,secureSpecialCmd_Count = 0,recoveryCmd_Count = 0,rx_Recovery_CurrentSet_Count = 0;
#ifdef KEY_ENABLE
extern uint32_t buttonsPressed;
extern uint8_t button_LH_cycle_check,button_RH_cycle_check;
uint8_t key_full_left_close = 0,key_full_right_close = 0;
#endif
uint32_t nv_max_cnt = 0;
uint16_t test_counter = 0;
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
    uint32_t bytes=*count;
    FLEXIO_UART_DRV_SendDataBlocking(&uartStateTX, (uint8_t *)buffer, bytes, TIMEOUT);
    return 0;
}
#endif
/* WatchDog IRQ handler */
void WDOG_ISR(void)
{
}

void LVD_LVW_ISR(void)
{
	if(PMC->LVDSC1 & PMC_LVDSC1_LVDF_MASK)
	{ 
		PMC->LVDSC1 |= PMC_LVDSC1_LVDF_MASK; 
	}
}

void LVD_SET(void)
{
	PMC->LVDSC1 |= PMC_LVDSC1_LVDRE_MASK; //LVD Reset Enable
	PMC->LVDSC1 |= PMC_LVDSC1_LVDIE_MASK;
	INT_SYS_InstallHandler(LVD_LVW_IRQn, LVD_LVW_ISR, (isr_t *)0);	
	INT_SYS_EnableIRQ(LVD_LVW_IRQn);
}

/*
 * @brief : Initialize clocks, pins and power modes
 */
void BoardInit(void)
{
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT, g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);
    POWER_SYS_Init(&powerConfigsArr, POWER_MANAGER_CONFIG_CNT, &powerStaticCallbacksConfigsArr, POWER_MANAGER_CALLBACK_CNT);
		POWER_SYS_SetMode(RUN, POWER_MANAGER_POLICY_AGREEMENT);
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
}

int main(void)
{
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
	OSIF_TimeDelay(5);
	Amo_Gpio_Init();
	
#ifdef FLEX_UART_ON
	Flexio_Uart_Init();
#endif	
	Lptmr_Init();
	Amo_Timer_Init();
	Lin_Init();
	EDMA_DRV_Init(&dmaController1_State, &dmaController1_InitConfig0, edmaChnStateArray, edmaChnConfigArray, EDMA_CONFIGURED_CHANNELS_COUNT);
	INT_SYS_InstallHandler(WDOG_EWM_IRQn, WDOG_ISR, (isr_t *)0);
	INT_SYS_EnableIRQ(WDOG_EWM_IRQn);
	#ifdef DEBUG_MODE
	printf("Air Vent main start\r\n");
	#endif
#ifdef AMO_NVM_SETTING
	Amo_Flash_Init();
#endif	
	Evt_queue_init();
	Start_Evnt_StatusCheck();
#ifdef AMO_NVM_SETTING
	nvmem_init();
#endif
	Amo_Can_Init();
	Amo_Adc_Init(); 
	WDOG_DRV_Init(INST_WATCHDOG1, &watchdog1_Config0);
	Adc_Value();
	LVD_SET();

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
			FLEXCAN_DRV_RxFifo(INST_CANCOM1,&recvBuff2);
			test_counter = 0;
		}

		switch(Board_state)
		{
			case DEVICE_STATE_INIT:		//Default mode check
			{
				Board_state = DEVICE_STATE_DEFAULT_MODE_INIT;
				break;
			}
			case DEVICE_STATE_MAIN_IDLE:
			{
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
				nvmem_read(&nv);
				if(Evnt_VBATT_Onoff)
				{
				switch(last_mode_save)
				{
					case DEFAULT_PROFILE:
					{	
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
					cycleModeFlag_sllr = FALSE,cycleModeFlag_slud = FALSE,cycleModeFlag_cllr = FALSE,cycleModeFlag_clud = FALSE,cycleModeFlag_crlr = FALSE;
					cycleModeFlag_crud = FALSE,cycleModeFlag_srlr = FALSE,cycleModeFlag_srud = FALSE;
					fullcloseModeFlag_clud = FALSE,fullcloseModeFlag_slud = FALSE,fullcloseModeFlag_crud = FALSE,fullcloseModeFlag_srud = FALSE;		
					if(Evnt_VBATT_Onoff)
					{
					if(P_Oip_Cts_Select == P_OIP_SELECT)
					{
						if(Touch_mode_Dr_set == HU_DRIVER_TOUCH)
						{
							Touch_mode_Dr_set = 0;
#ifdef KEY_ENABLE
							PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
							PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif									
							Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
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
						if(Touch_mode_Ps_set == HU_PASSENGER_TOUCH)
						{
							Touch_mode_Ps_set = 0;
#ifdef KEY_ENABLE
							PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
							PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif				
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
							RrLinData_Parsing(&LIN_RC_EVNT_MASTER_CMD);
							lin_RrmanualMode_task(&LIN_RC_EVNT_MASTER_CMD);
						}
						else
						{
							
						}							
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
				#ifdef DEBUG_MODE
				printf("Full close mode \r\n");
				#endif
				cycleModeFlag_sllr = FALSE,cycleModeFlag_slud = FALSE,cycleModeFlag_cllr = FALSE,cycleModeFlag_clud = FALSE,cycleModeFlag_crlr = FALSE,cycleModeFlag_crud = FALSE;
				cycleModeFlag_srlr = FALSE,cycleModeFlag_srud = FALSE,fullcloseModeFlag_slud = TRUE,fullcloseModeFlag_clud = TRUE,fullcloseModeFlag_crud = TRUE,fullcloseModeFlag_srud = TRUE;				
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
					key_full_left_close = 1,Hu_Fr_dr_Vent_Side_signal = 0x02,Hu_Fr_dr_Vent_Center_signal = 0x02;
				}
				else if(buttonsPressed == 2)
				{
					key_full_right_close = 1,Hu_Fr_Ps_Vent_Side_signal = 0x02,Hu_Fr_Ps_Vent_Center_signal = 0x02;
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
				#ifdef DEBUG_MODE
				printf("Focus mode \r\n");
				#endif
				cycleModeFlag_sllr = FALSE,cycleModeFlag_slud = FALSE,cycleModeFlag_cllr = FALSE,cycleModeFlag_clud = FALSE,cycleModeFlag_crlr = FALSE,cycleModeFlag_crud = FALSE;
				cycleModeFlag_srlr = FALSE,cycleModeFlag_srud = FALSE,fullcloseModeFlag_clud = FALSE,fullcloseModeFlag_slud = FALSE,fullcloseModeFlag_crud = FALSE,fullcloseModeFlag_srud = FALSE;
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
					key_full_left_close = 0,Hu_Fr_Dr_Vent_Mode = FACING_MODE,Hu_Fr_dr_Vent_Side_signal = 0x01,Hu_Fr_dr_Vent_Center_signal = 0x01;
				}
				else if(buttonsPressed == 2)
				{
					key_full_right_close = 0,Hu_Fr_Ps_Vent_Mode = FACING_MODE,Hu_Fr_Ps_Vent_Side_signal = 0x01,Hu_Fr_Ps_Vent_Center_signal = 0x01;
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
				#ifdef DEBUG_MODE
				printf("Spread mode \r\n");
				#endif
				cycleModeFlag_sllr = FALSE,cycleModeFlag_slud = FALSE,cycleModeFlag_cllr = FALSE,cycleModeFlag_clud = FALSE,cycleModeFlag_crlr = FALSE,cycleModeFlag_crud = FALSE;
				cycleModeFlag_srlr = FALSE,cycleModeFlag_srud = FALSE,fullcloseModeFlag_clud = FALSE,fullcloseModeFlag_slud = FALSE,fullcloseModeFlag_crud = FALSE,fullcloseModeFlag_srud = FALSE;
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
				#ifdef DEBUG_MODE
				printf("CYCLE mode %d \r\n", last_mode_save);
				#endif
				cycleModeFlag_sllr = TRUE,cycleModeFlag_slud = TRUE,cycleModeFlag_cllr = TRUE,cycleModeFlag_clud = TRUE,cycleModeFlag_crlr = TRUE,cycleModeFlag_crud = TRUE;
				cycleModeFlag_srlr = TRUE,cycleModeFlag_srud = TRUE,fullcloseModeFlag_clud = FALSE,fullcloseModeFlag_slud = FALSE,fullcloseModeFlag_crud = FALSE,fullcloseModeFlag_srud = FALSE;
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
				else if(DeviceEvent & DEVICE_ATCU_TX_EVENT)  
				{
					DeviceEvent = DeviceEvent & ~DEVICE_ATCU_TX_EVENT;

					Board_state = DEVICE_STATC_ATCU_TX_MAIN;
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
				
//				FR_SLLR_FailSafe_Flag = TRUE,FR_SLUD_FailSafe_Flag = TRUE,FR_CLLR_FailSafe_Flag = TRUE,FR_CLUD_FailSafe_Flag = TRUE;
//				FR_CRLR_FailSafe_Flag = TRUE,FR_CRUD_FailSafe_Flag = TRUE,FR_SRLR_FailSafe_Flag = TRUE,FR_SRUD_FailSafe_Flag = TRUE;
//				recoveryFlags.FR_SLLR_Recov_Flag = TRUE,recoveryFlags.FR_SLUD_Recov_Flag = TRUE,recoveryFlags.FR_CLLR_Recov_Flag = TRUE,recoveryFlags.FR_CLUD_Recov_Flag = TRUE;
//				recoveryFlags.FR_SRLR_Recov_Flag = TRUE,recoveryFlags.FR_SRUD_Recov_Flag = TRUE,recoveryFlags.FR_CRLR_Recov_Flag = TRUE,recoveryFlags.FR_CRUD_Recov_Flag = TRUE;

//				gotoZeroPtFlag_cllr = FALSE,lastTryFlag_cllr = TRUE,gotoZeroPtFlag_clud = FALSE,lastTryFlag_clud = TRUE,gotoZeroPtFlag_sllr = FALSE,lastTryFlag_sllr = TRUE,gotoZeroPtFlag_slud = FALSE,lastTryFlag_slud = TRUE;			
//				gotoZeroPtFlag_crlr = FALSE,lastTryFlag_crlr = TRUE,gotoZeroPtFlag_crud = FALSE,lastTryFlag_crud = TRUE,gotoZeroPtFlag_srlr = FALSE,lastTryFlag_srlr = TRUE,gotoZeroPtFlag_srud = FALSE,lastTryFlag_srud = TRUE;

				secureMainFlag = TRUE,rx_Recovery_CurrentSet_Count = 0;
				secureSpecialCmd_Count++;
				
				if(secureSpecialCmd_Count == 1)
				{
/*
					Lin_Scheduler_SpecialCmd_Start_1();

					if(cycleModeFlag_sllr == TRUE) { lin_FrDrCycleMode_TimerStop(); }
					if(cycleModeFlag_slud == TRUE) { lin_FrDrCycleMode_TimerStop(); }
					if(cycleModeFlag_cllr == TRUE) { lin_FrDrCycleMode_TimerStop(); }
					if(cycleModeFlag_clud == TRUE) { lin_FrDrCycleMode_TimerStop(); }

					if(cycleModeFlag_crlr == TRUE) { lin_FrPsCycleMode_TimerStop(); }
					if(cycleModeFlag_crud == TRUE) { lin_FrPsCycleMode_TimerStop(); }
					if(cycleModeFlag_srlr == TRUE) { lin_FrPsCycleMode_TimerStop(); }
					if(cycleModeFlag_srud == TRUE) { lin_FrPsCycleMode_TimerStop(); }
*/

					if((cycleModeFlag_sllr == TRUE) || (cycleModeFlag_slud == TRUE) || (cycleModeFlag_cllr == TRUE) || (cycleModeFlag_clud == TRUE))
					{
						lin_FrDrCycleMode_TimerStop();
					}
					else
					{
//						FR_SLLR_FailSafe_Flag = TRUE,FR_SLUD_FailSafe_Flag = TRUE,FR_CLLR_FailSafe_Flag = TRUE,FR_CLUD_FailSafe_Flag = TRUE;
//						recoveryFlags.FR_SLLR_Recov_Flag = TRUE,recoveryFlags.FR_SLUD_Recov_Flag = TRUE,recoveryFlags.FR_CLLR_Recov_Flag = TRUE,recoveryFlags.FR_CLUD_Recov_Flag = TRUE;
//						gotoZeroPtFlag_cllr = FALSE,lastTryFlag_cllr = TRUE,gotoZeroPtFlag_clud = FALSE,lastTryFlag_clud = TRUE,gotoZeroPtFlag_sllr = FALSE,lastTryFlag_sllr = TRUE,gotoZeroPtFlag_slud = FALSE,lastTryFlag_slud = TRUE;

						Lin_Scheduler_SpecialCmd_Start_1_LH();
					}
						
					if((cycleModeFlag_crlr == TRUE) || (cycleModeFlag_crud == TRUE) || (cycleModeFlag_srlr == TRUE) || (cycleModeFlag_srud == TRUE))
					{
						lin_FrPsCycleMode_TimerStop();
					}
					else
					{
//						FR_CRLR_FailSafe_Flag = TRUE,FR_CRUD_FailSafe_Flag = TRUE,FR_SRLR_FailSafe_Flag = TRUE,FR_SRUD_FailSafe_Flag = TRUE;
//						recoveryFlags.FR_SRLR_Recov_Flag = TRUE,recoveryFlags.FR_SRUD_Recov_Flag = TRUE,recoveryFlags.FR_CRLR_Recov_Flag = TRUE,recoveryFlags.FR_CRUD_Recov_Flag = TRUE;
//						gotoZeroPtFlag_crlr = FALSE,lastTryFlag_crlr = TRUE,gotoZeroPtFlag_crud = FALSE,lastTryFlag_crud = TRUE,gotoZeroPtFlag_srlr = FALSE,lastTryFlag_srlr = TRUE,gotoZeroPtFlag_srud = FALSE,lastTryFlag_srud = TRUE;
					
						Lin_Scheduler_SpecialCmd_Start_1_RH();
					}
	
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
				#ifdef DEBUG_MODE
				//printf("Evnt_IGN2_Onoff = %d IGN2 ADC Value = %s adcValue = %f adcRawValue = %d adcMax = %d \r\n", Evnt_IGN2_Onoff, Ign2_adc_msg, Ign2_adcValue, Ign2_adcRawValue, adcMax);
				//printf("IGN3 ADC Value = %s adcValue = %f adcRawValue = %d adcMax = %d \r\n", Ign3_adc_msg, Ign3_adcValue, Ign3_adcRawValue, adcMax);
				//printf("VATT adcValue = %f adcRawValue = %d adcMax = %d Vatt_LimitValue = %d Evnt_VBATT_Onoff = %d \r\n", Vatt_adcValue, Vatt_adc_RawValue, adcMax, Vatt_LimitValue, Evnt_VBATT_Onoff);
				//printf("Evnt_VBATT_Over = %d Evnt_VBATT_Under = %d \r\n", Evnt_VBATT_Over, Evnt_VBATT_Under);
				//printf("Evnt_COM_Onoff = %d Evnt_COM_Over = %d Evnt_COM_Under = %d \r\n", Evnt_COM_Onoff, Evnt_COM_Over, Evnt_COM_Under);
				//printf("can_error_status = 0x%x \r\n", can_error_data);
				//printf("Evnt_IGN2_Onoff = %d Vatt_LimitValue = %d Ign2_AdcMinValue = %d \r\n", Evnt_IGN2_Onoff, Vatt_LimitValue, Ign2_AdcMinValue);
				//printf("batt_cal_data = %d ign2_cal_data = %d \r\n", batt_data, ign_data);

				printf("Evnt_IGN2_Onoff = %d Evnt_Network_Release = %d \r\n", Evnt_IGN2_Onoff, Evnt_Network_Release);
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
				#ifdef DEBUG_MODE
				printf("recovery main \r\n");
				#endif				
				FR_SLLR_FailSafe_Flag = FALSE,FR_SLUD_FailSafe_Flag = FALSE,FR_CLLR_FailSafe_Flag = FALSE,FR_CLUD_FailSafe_Flag = FALSE;
				FR_CRLR_FailSafe_Flag = FALSE,FR_CRUD_FailSafe_Flag = FALSE,FR_SRLR_FailSafe_Flag = FALSE,FR_SRUD_FailSafe_Flag = FALSE;
				recoveryFlags.FR_SLLR_Recov_Flag = FALSE,recoveryFlags.FR_SLUD_Recov_Flag = FALSE,recoveryFlags.FR_CLLR_Recov_Flag = FALSE,recoveryFlags.FR_CLUD_Recov_Flag = FALSE;			
				recoveryFlags.FR_SRLR_Recov_Flag = FALSE,recoveryFlags.FR_SRUD_Recov_Flag = FALSE,recoveryFlags.FR_CRLR_Recov_Flag = FALSE,recoveryFlags.FR_CRUD_Recov_Flag = FALSE;
				secureMainFlag = FALSE;
				recoveryPattern_sllr = Recovery_None,recoveryPattern_slud = Recovery_None,recoveryPattern_cllr = Recovery_None,recoveryPattern_clud = Recovery_None;
				recoveryPattern_crlr = Recovery_None,recoveryPattern_crud = Recovery_None,recoveryPattern_srlr = Recovery_None,recoveryPattern_srud = Recovery_None;			
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
				DTC_OTA_event();
				Board_state = DEVICE_STATE_MAIN_IDLE;	
				break;
			}
			case DEVICE_STATC_ATCU_TX_MAIN:
			{
				if(Evnt_COM_Onoff)
				{
					Evnt_Atcu_tx_data();
				}
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
					lin_FrDrCycleMode_TimerStop();
					lin_CycleMode_CanDisp_TimerStop();
					lin_FrPsCycleMode_TimerStop();
					if(L_ATCU_IAUProfileValue == L_ATCU_AVNProfileValue)
					{
						last_mode_save = L_ATCU_AVNProfileValue;
					}
					else
					{
						last_mode_save = L_ATCU_AVNProfileValue;	
					}
					Sleep_5second = 1;					
					Lin_GoTo_Sleep();
					Amo_timer_Stop(timer_82);
					Amo_timer_Start(timer_82, 5000, false, Amo_Sleep);				
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
			/* default state */
			break;
		}
		Handle_RecoveryMode_Steploss();
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

