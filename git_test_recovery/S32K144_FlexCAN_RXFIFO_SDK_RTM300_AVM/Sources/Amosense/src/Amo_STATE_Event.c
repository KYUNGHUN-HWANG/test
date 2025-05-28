#ifndef AMO_EVENT_C
#define AMO_EVENT_C

/*===========================================================================*/
/* Project   :  AMOSENSE event function Software                                                                */
/* File name :  Amo_State_Event.c                                                                             */
/*===========================================================================*/
/*                                  COPYRIGHT                                                            */
/*===========================================================================*/
/*                                                                                                                                                    */
/* Copyright (c) 2024 by Amosense co.,Ltd,                                                                   */
/*                                                                                                        */
/*===========================================================================*/
/*                              File Description                                                             */
/*===========================================================================*/
/*                                                                                                                                                   */
/* Header file for event.                                                                               */
/*                                                                                                         */
/*===========================================================================*/
/*                                 History                                                                 */
/*===========================================================================*/
//============================================================================
//Project          : Air Vent    Nifco
//Date : 2024. 11.09
//Hardware Version : 
//Compiler         : S32 Design Studio for ARm  Ver 2.2 Build id:200116
//MCU              : FS32K144HAT0MLHT 64pin 
//           MCU           CAR   SPEC HW_Ver
//#define FS32K144HAT0MLHT_GN7PE_V10_V01  
//Software Version : V0.1      PSM     2024. 11.09        Initialize code     
//
//
//
//
//============================================================================ 

#include "Cpu.h"
#include "clockMan1.h"
#include "dmaController1.h"
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
#include "Amo_Sleep.h"
#include "Amo_Cycle.h"
#include "Amo_System_setting.h"
/******************************************************************************
 * static variable Definitions
 ******************************************************************************/
DeviceStateType Board_state = DEVICE_STATE_INIT;
/* User includes (#include below this line is not maintained by Processor Expert) */

/******************************************************************************
 * Variable prototypes
 ******************************************************************************/
//static uint8_t Sleep_5second = 0;
uint8_t secureSpecialCmd_Count = 0, rx_Recovery_CurrentSet_Count = 0;

/******************************************************************************
 * prototype define
 ******************************************************************************/
static void Mode_select_func(DeviceStateType vent_state);
/*
* @brief Handle_Focus_Mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Mode_select_func(DeviceStateType vent_state)
{
	switch(vent_state)
	{
		case DEVICE_STATE_SPREAD_MAIN:
		{
			Handle_Spread_Mode();
			break;
		}
		case DEVICE_STATE_CYCLE_MAIN:
		{
			Handle_Cycle_Mode();
			break;
		}
		case DEVICE_STATE_RECOVERY_MAIN:
		{
			break;
		}
		default:
		/* default */
		break;
	}
}

/*
* @brief Handle_Focus_Mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Handle_Event_Mode(void)
{
	switch(Board_state)
	{
		case DEVICE_STATE_INIT: 	//Default mode check
		{
			/*Mode state start */
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
			/* Default Mode state start */
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
			/* Last Mode state start */
			Board_state = DEVICE_STATE_LAST_MAIN; 		
			break;
		}
#ifdef AMO_NVM_SETTING
		case DEVICE_STATE_LAST_MAIN:
		{
			Handle_Last_Mode(); 		
			break;
		}
#endif
		case DEVICE_STATE_MANUAL_MODE_INIT:
		{
			/* Manual Mode state start */
			Board_state = DEVICE_STATE_MANUAL_MAIN; 		
			break;
		}

		case DEVICE_STATE_MANUAL_MAIN:
		{
			Handle_Manual_Mode(); 	
				//Mode_select_func(DEVICE_STATE_CYCLE_MAIN);
			break;
		}
		case DEVICE_STATE_FULL_CLOSE_MODE_INIT:
		{
			/* Full close Mode state start */
			Board_state = DEVICE_STATE_FULL_CLOSE_MAIN; 		
			break;
		}
		case DEVICE_STATE_FULL_CLOSE_MAIN:
		{ 			
			Handle_Full_close_Mode();		
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
			/* Foucs Mode state start */
			Board_state = DEVICE_STATE_FOCUS_MAIN;			
			break;
		}
		case DEVICE_STATE_FOCUS_MAIN:
		{
			Handle_Foucs_Mode();	
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
			/* Spread Mode state start */
			Board_state = DEVICE_STATE_SPREAD_MAIN; 		
			break;
		}
		case DEVICE_STATE_SPREAD_MAIN:
		{
			//Handle_Spread_Mode();
			Mode_select_func(DEVICE_STATE_SPREAD_MAIN);
			break;
		}
		case DEVICE_STATE_CYCLE_MODE_INIT:
		{
			/* Cycle Mode state start */
			Board_state = DEVICE_STATE_CYCLE_MAIN;			
			break;
		}
		case DEVICE_STATE_CYCLE_MAIN:
		{
			//Handle_Cycle_Mode();	
			Mode_select_func(DEVICE_STATE_CYCLE_MAIN);
			break;
		}
		case DEVICE_STATE_RECOVERY_MODE_INIT:
		{ 		
			/* Recovery Mode state start */
			Board_state = DEVICE_STATE_RECOVERY_MAIN; 		
			break;
		}
		case DEVICE_STATE_RECOVERY_MAIN:
		{
			Board_state = DEVICE_STATE_MAIN_IDLE;
			break;
		}
		case DEVICE_STATE_EVT_INIT:
		{
			/* event Mode check start */
			Board_state = DEVICE_STATE_EVT_MAIN;	
			break;
		}
		case DEVICE_STATE_EVT_MAIN:
		{
			Handle_Event_Choice();			
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
			/* Secure Mode state start */
			Board_state = DEVICE_STATE_SECURE_MAIN;
			break;
		}
		case DEVICE_STATE_SECURE_MAIN:
		{
			Handle_Secure_Mode();
			//Mode_select_func(DEVICE_STATE_CYCLE_MAIN);
			break;
			} 		
		case DEVICE_STATE_UTILITY_MODE_INIT:
		{
			/* Utility Mode state start */
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
			//printf("Evnt_IGN2_Onoff = %d IGN2 adcValue = %f adcRawValue = %d adcMax = %d \r\n", Evnt_IGN2_Onoff, Ign2_adcValue, Ign2_adcRawValue, adcMax);
			//printf("IGN3 ADC Value = %s adcValue = %f adcRawValue = %d adcMax = %d \r\n", Ign3_adc_msg, Ign3_adcValue, Ign3_adcRawValue, adcMax);
			//printf("VATT adcValue = %f adcRawValue = %d adcMax = %d Vatt_LimitValue = %d Evnt_VBATT_Onoff = %d \r\n", Vatt_adcValue, Vatt_adc_RawValue, adcMax, Vatt_LimitValue, Evnt_VBATT_Onoff);
			//printf("Evnt_VBATT_Over = %d Evnt_VBATT_Under = %d \r\n", Evnt_VBATT_Over, Evnt_VBATT_Under);
			//printf("Evnt_COM_Onoff = %d Evnt_COM_Over = %d Evnt_COM_Under = %d \r\n", Evnt_COM_Onoff, Evnt_COM_Over, Evnt_COM_Under);
			//printf("can_error_status = 0x%x \r\n", can_error_data);
			//printf("Evnt_IGN2_Onoff = %d Vatt_LimitValue = %d Ign2_AdcMinValue = %d \r\n", Evnt_IGN2_Onoff, Vatt_LimitValue, Ign2_AdcMinValue);
			//printf("batt_cal_data = %d ign2_cal_data = %d \r\n", batt_data, ign_data);

			//printf("Evnt_IGN2_Onoff = %d Evnt_Network_Release = %d \r\n", Evnt_IGN2_Onoff, Evnt_Network_Release);
			//printf(" %d  %d  %d  %d \r\n", Hu_Frdr_Vent_Side_sig_backup,Hu_Frdr_Vent_Center_sig_backup, Hu_FrPs_Vent_Side_sig_backup,Hu_FrPs_Vent_Center_sig_backup );
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
			/* Vent Recovery state start */
			Board_state = DEVICE_RX_RECOVERY_MAIN;
			break;
		}
		case DEVICE_RX_RECOVERY_MAIN:
		{
		#ifdef DEBUG_MODE
			printf("recovery main \r\n");
		#endif				
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
			/* Diag message state start */
			Board_state = DEVICE_EVNT_ATCU_SEND_MAIN;
			break;
		}
		case DEVICE_EVNT_ATCU_SEND_MAIN:
		{
			if(Evnt_COM_Onoff == 1)
			{
				Evnt_Atcu_tx_data();
				//Mode_select_func(DEVICE_STATE_CYCLE_MAIN); //20250423_check
			}
			Board_state = DEVICE_STATE_MAIN_IDLE; 
			break;
		}
		case DEVICE_STATC_DTC_MAIN:
		{
			/* DTC state start */
			DTC_Send_event();
			//Mode_select_func(DEVICE_STATE_CYCLE_MAIN);
			Board_state = DEVICE_STATE_MAIN_IDLE; 
			break;
		}
		case DEVICE_STATC_FW_OTA_MAIN:
		{
			/* FW Update state start */
			DTC_OTA_event();
			//Mode_select_func(DEVICE_STATE_CYCLE_MAIN);
			Board_state = DEVICE_STATE_MAIN_IDLE; 
			break;
		}
		case DEVICE_STATC_ATCU_TX_MAIN:
		{
			if(Evnt_COM_Onoff == 1)
			{
				Evnt_Atcu_tx_data();
				//Mode_select_func(DEVICE_STATE_CYCLE_MAIN);
			}
			Board_state = DEVICE_STATE_MAIN_IDLE; 
			break;
		} 			
		case DEVICE_STATE_SLEEP_INIT:
		{
			/* Sleep Mode state start */
			Board_state = DEVICE_STATE_SLEEP_MAIN;
			break;
		}
		case DEVICE_STATE_SLEEP_ENTER:
		{
			Handle_Sleep_Enter();
			//Mode_select_func(DEVICE_STATE_CYCLE_MAIN);
			break;
		} 		
		case DEVICE_STATE_SLEEP_MAIN:
		{
			Handle_Sleep_Main();
			break;
		}

		default:
		/* default state */
		break;
	}

}
#endif

