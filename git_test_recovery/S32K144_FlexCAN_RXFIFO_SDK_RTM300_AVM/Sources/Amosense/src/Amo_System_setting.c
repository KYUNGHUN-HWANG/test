#ifndef AMO_SYSTEM_C
#define AMO_SYSTEM_C

/*===========================================================================*/
/* Project   :  AMOSENSE system function Software                                                                */
/* File name :  Amo_System_setting.c                                                                             */
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
#include "Amo_Vent_mode.h"
/* static variable define */
static unsigned long long DeviceEvent = 0;
static uint8_t Sleep_5second = 0;

/******************************************************************************
 * Prototype Define
 ******************************************************************************/
void Handle_Manual_Mode(void);
void Handle_Spread_Mode(void);
void Handle_Foucs_Mode(void);
void Manual_ps_evt_signal(void);
void LVD_SET(void);
void LVD_LVW_ISR(void);
void Manual_dr_evt_signal(void);
void Handle_Last_Mode(void);
void BoardInit(void);
void Handle_Full_close_Mode(void);
void Handle_Cycle_Mode(void);
void Handle_Secure_Mode(void);
void Handle_Sleep_Main(void);
void Handle_Sleep_Enter(void);
void Handle_Event_Choice(void);
static int read_system_nv(nvmem_data_type *pcBuffer);

/*
* @brief LVD_LVW_ISR function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void LVD_LVW_ISR(void)
{
	/* LVD warning interupt call */
	if((PMC->LVDSC1 & PMC_LVDSC1_LVDF_MASK) != 0)
	{ 
		PMC->LVDSC1 |= PMC_LVDSC1_LVDF_MASK; 
	}
}
/*
* @brief LVD_SET function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void LVD_SET(void)
{
	/* LVD status setting*/
	PMC->LVDSC1 |= PMC_LVDSC1_LVDRE_MASK; //LVD Reset Enable
	PMC->LVDSC1 |= PMC_LVDSC1_LVDIE_MASK;
	INT_SYS_InstallHandler(LVD_LVW_IRQn, LVD_LVW_ISR, (isr_t *)NULL);	
	INT_SYS_EnableIRQ(LVD_LVW_IRQn);
}

/*
* @brief Manual_dr_evt_signal function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Manual_dr_evt_signal(void)
{
	if((Hu_FrDrEVntSidePt_X_invalid != 2047) || (Hu_FrDrEVntSidePt_Y_invalid != 2047))
	{
		/*manual mode drive side init */
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
		Hu_Frdr_Vent_Side_sig_backup = 0x01;
	}
	if((Hu_FrDrEVntCtrPt_X_invalid != 2047) || (Hu_FrDrEVntCtrPt_Y_invalid != 2047))
	{
		/*manual mode drive center init */
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
		Hu_Frdr_Vent_Center_sig_backup = 0x01;
	} 
}
/*
* @brief Manual_ps_evt_signal function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Manual_ps_evt_signal(void)
{
	if((Hu_FrPsEVntSidePt_X_invalid != 2047) || (Hu_FrPsEVntSidePt_Y_invalid != 2047))
	{
		/*manual mode passenger side init */
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
		Hu_FrPs_Vent_Side_sig_backup = 0x01;
	}
	if((Hu_FrPsEVntCtrPt_X_invalid != 2047) || (Hu_FrPsEVntCtrPt_Y_invalid != 2047))
	{
		/*manual mode passenger center init */
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
		Hu_FrPs_Vent_Center_sig_backup = 0x01;
	}
}

/*
* @brief BoardInit function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void BoardInit(void)
{
    CLOCK_SYS_Init(g_clockManConfigsArr, CLOCK_MANAGER_CONFIG_CNT, g_clockManCallbacksArr, CLOCK_MANAGER_CALLBACK_CNT);
    CLOCK_SYS_UpdateConfiguration(0U, CLOCK_MANAGER_POLICY_FORCIBLE);
    POWER_SYS_Init(&powerConfigsArr, POWER_MANAGER_CONFIG_CNT, &powerStaticCallbacksConfigsArr, POWER_MANAGER_CALLBACK_CNT);
		POWER_SYS_SetMode(RUN, POWER_MANAGER_POLICY_AGREEMENT);
    PINS_DRV_Init(NUM_OF_CONFIGURED_PINS, g_pin_mux_InitConfigArr);
}

/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/
/**
 * @brief  read_device_nv function
 * @param  Event: None
 * @retval Ack: Return whether the Event has been managed or not
 * @example  None
 * @note This function polls until conversion is complete.
 */
static int read_system_nv(nvmem_data_type *pcBuffer)
{
	return nvmem_read(pcBuffer);
}

/*
* @brief Handle_Manual_Mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Handle_Last_Mode(void)
{
#ifdef AMO_NVM_SETTING
		nvmem_data_type nv;
#endif	
	//nvmem_read(&nv);
	read_system_nv(&nv);
	/* default/last mode setting */
	if(Evnt_VBATT_Onoff == 1)
	{
		if(last_mode_save == DEFAULT_PROFILE)
		{
			Default_Setting_Setup(DEFAULT_PROFILE);
		}
		else
		{
			if(nv.nv_item.profile_active[last_mode_save] != 0)
			{
				Last_Setting_Setup(last_mode_save);
			}
			else
			{
				Default_Setting_Setup(last_mode_save);
			}
		} 
	}
	Board_state = DEVICE_STATE_MAIN_IDLE; 	
}

/*
* @brief Handle_Manual_Mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Handle_Manual_Mode(void)
{
	//Manual mode action
	//Adc_Value();
#ifdef DEBUG_MODE
	printf("Manual mode \r\n");
#endif	
	if(Evnt_VBATT_Onoff == 1)
	{
	if(P_Oip_Cts_Select == P_OIP_SELECT)
		{
			Manual_side_type();
		#ifdef AMO_GN7_PE_SETTING_NONE
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
		#endif
		}
		else
		{
		#ifdef AMO_GN7_PE_SETTING_NONE
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
		#endif
	}
	}	
	P_Oip_Cts_priority = 0;
	Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x01;	//APPLY status return
	Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_CTS = 0x01;
	Board_state = DEVICE_STATE_MAIN_IDLE; 		
}

/*
* @brief Handle_Full_close_Mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Handle_Full_close_Mode(void)
{
	#ifdef DEBUG_MODE
	printf("Full close mode \r\n");
	#endif				
	if(Evnt_VBATT_Onoff == 1)
	{
	if(P_Oip_Cts_Select == P_OIP_SELECT)
	{						
		Full_close_side_type();
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
}

/*
* @brief Handle_Foucs_Mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Handle_Foucs_Mode(void)
{
	#ifdef DEBUG_MODE
	printf("Focus mode \r\n");
	#endif
	if(Evnt_VBATT_Onoff == 1)
	{
	if(P_Oip_Cts_Select == P_OIP_SELECT)
	{	
		Focus_side_type();
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
}

/*
* @brief Handle_Spread_Mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Handle_Spread_Mode(void)
{
#ifdef DEBUG_MODE
	printf("Spread mode \r\n");
#endif
	if(Evnt_VBATT_Onoff == 1)
	{
	if(P_Oip_Cts_Select == P_OIP_SELECT)
	{ 				
		Spread_side_type();
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
}

/*
* @brief Handle_Cycle_Mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Handle_Cycle_Mode(void)
{
#ifdef DEBUG_MODE
	printf("CYCLE mode %d \r\n", last_mode_save);
#endif
	if(Evnt_VBATT_Onoff == 1)
	{
	if(P_Oip_Cts_Select == P_OIP_SELECT)
	{ 		
		Cycle_side_type();
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
}

/*
* @brief Handle_Secure_Mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Handle_Secure_Mode(void)
{
#ifdef DEBUG_MODE
	printf("Acturator Stop !!! \r\n");
#endif
	
	rx_Recovery_CurrentSet_Count = 0;
	secureSpecialCmd_Count++;
	
	if(secureSpecialCmd_Count == 1)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_FrPsCycleMode_TimerStop();
	}
	
	Board_state = DEVICE_STATE_MAIN_IDLE; 
}

/*
* @brief Handle_Sleep_Main function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Handle_Sleep_Main(void)
{
#ifdef AMO_GN7_PE_SETTING_NONE
	if(((!Evnt_IGN2_Onoff) && (!Evnt_IGN3_Onoff)) && (!Evnt_Network_Release))
#else
	if((Evnt_IGN2_Onoff == 0) && (Evnt_Network_Release == 0))
#endif
	{
		if(Sleep_5second == 0)
		{
			lin_FrDrCycleMode_TimerStop();
			lin_CycleMode_CanDisp_TimerStop();
			lin_FrPsCycleMode_TimerStop();
			uint8_t avn_profile_tmp = L_ATCU_AVNProfileValue;
			uint8_t iau_profile_tmp = L_ATCU_IAUProfileValue; 				
			if(avn_profile_tmp == iau_profile_tmp)
			{
				last_mode_save = (Profile_mode)L_ATCU_AVNProfileValue;
			}
			else
			{
				last_mode_save = (Profile_mode)L_ATCU_AVNProfileValue;	
			}
			Sleep_5second = 1;					
			Lin_GoTo_Sleep();
			Amo_timer_Stop(timer_82);
			Amo_timer_Start(timer_82, 5000, false, Amo_Sleep);				
		}
	}
	else
	{
		if(Sleep_5second == 1)
		{
//			Lin_GoTo_WakeUp();
			Lin_Init();
		}
						
		Sleep_5second = 0;
		Amo_timer_Stop(timer_82);
	}
		Board_state = DEVICE_STATE_MAIN_IDLE;
}

/*
* @brief Handle_Sleep_Enter function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Handle_Sleep_Enter(void)
{
	nvmem_data_type nv;
	
	//nvmem_read(&nv);
	read_system_nv(&nv);
	uint8_t idx = 0;
	idx = (uint8_t)last_mode_save;
	nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
	nv_max_cnt = nv.nv_item.write_cnt;
	nv.nv_item.drive_type = (uint8_t)ATCU_DriverSideType;
	nv.nv_item.profile = (uint8_t)idx;
	nv.nv_item.profile_active[idx] = (uint8_t)idx;
#ifdef AMO_DTC_SETTING	
	nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 637);
#else
	nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 441);
#endif
	nvmem_write(nv.nv_data,PAGE_SIZE);			
	Last_Setting_Save(last_mode_save);
	PINS_DRV_ClearPins(GPIO_PORTC, CAN_STB_MASK);
	Board_state = DEVICE_STATE_MAIN_IDLE;
}

/*
* @brief Handle_Event_Choice function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/

void Handle_Event_Choice(void)
{
	DeviceEvent = Evt_queue_value();
		if((DeviceEvent & DEVICE_CAN_EVNT_LAST_MODE_EVENT) != 0)
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CAN_EVNT_LAST_MODE_EVENT;

			Board_state = DEVICE_STATE_LAST_MODE_INIT;
		} 							
#ifdef KEY_ENABLE
		else if((DeviceEvent & DEVICE_LH_KEY_PRESS_EVENT) != 0)
		{
			DeviceEvent = DeviceEvent & ~DEVICE_LH_KEY_PRESS_EVENT;

			Board_state = DEVICE_STATE_LH_KEY_INIT;
		}
		else if((DeviceEvent & DEVICE_RH_KEY_PRESS_EVENT) != 0)
		{
			DeviceEvent = DeviceEvent & ~DEVICE_RH_KEY_PRESS_EVENT;

			Board_state = DEVICE_STATE_RH_KEY_INIT;
		}
		else if((DeviceEvent & DEVICE_LH_LONG_PRESS_EVENT) != 0)
		{
			DeviceEvent = DeviceEvent & ~DEVICE_LH_LONG_PRESS_EVENT;

			Board_state = DEVICE_STATE_KEY_FULL_CLOSE_MODE_INIT;
		}
		else if((DeviceEvent & DEVICE_RH_LONG_PRESS_EVENT) != 0)
		{
			DeviceEvent = DeviceEvent & ~DEVICE_RH_LONG_PRESS_EVENT;

			Board_state = DEVICE_STATE_KEY_FULL_CLOSE_MODE_INIT;
		}
		else if((DeviceEvent & DEVICE_KEY_FOCUS_PRESS_EVENT) != 0)
		{
			DeviceEvent = DeviceEvent & ~DEVICE_KEY_FOCUS_PRESS_EVENT;

			Board_state = DEVICE_STATE_KEY_FOCUS_MODE_INIT;
		} 			
		else if((DeviceEvent & DEVICE_H_KEY_FULL_CLOSE_LIN_CAN_EVENT) != 0)  //Full Close mode Hot key LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_H_KEY_FULL_CLOSE_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;		
		}
		else if((DeviceEvent & DEVICE_STICK_FULL_CLOSE_LIN_CAN_EVENT) != 0) //Full Close mode STICK LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_STICK_FULL_CLOSE_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;
		}
		else if((DeviceEvent & DEVICE_H_KEY_FOCUS_LIN_CAN_EVENT) != 0)	//Focus mode Hot key LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_H_KEY_FOCUS_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_FOCUS_MODE_INIT; 	
		}
		else if((DeviceEvent & DEVICE_STICK_FOCUS_LIN_CAN_EVENT) != 0) //Focus mode STICK LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_STICK_FOCUS_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_FOCUS_MODE_INIT;
		}
		else if((DeviceEvent & DEVICE_H_KEY_SPREAD_LIN_CAN_EVENT) != 0)  //Spread mode Hot key LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_H_KEY_SPREAD_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_SPREAD_MODE_INIT;		
		}
		else if((DeviceEvent & DEVICE_H_KEY_CYCLE_LIN_CAN_EVENT) != 0)	//Cycle mode Hot key LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_H_KEY_CYCLE_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_CYCLE_MODE_INIT; 	
		} 
#endif
		else if((DeviceEvent & DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT) != 0)  //Manual mode input CAN->LIN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT;

			Board_state = DEVICE_STATE_MANUAL_MODE_INIT;		
		}
#ifdef AMO_GN7_PE_SETTING_NONE
		else if((DeviceEvent & DEVICE_CTS_MANUAL_CAN_LIN_EVENT) != 0) //Manual mode input CAN->LIN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CTS_MANUAL_CAN_LIN_EVENT;
		
			Board_state = DEVICE_STATE_MANUAL_MODE_INIT;
		}
#endif
		else if((DeviceEvent & DEVICE_P_OIP_MANUAL_LIN_CAN_EVENT) != 0)  //Manual mode input LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_MANUAL_LIN_CAN_EVENT; //Manual mode input LIN->CAN

			Board_state = DEVICE_STATE_MANUAL_MODE_INIT;
		}
#ifdef AMO_GN7_PE_SETTING_NONE
		else if((DeviceEvent & DEVICE_CTS_MANUAL_LIN_CAN_EVENT) != 0)
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CTS_MANUAL_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_MANUAL_MODE_INIT;
		}
#endif
		else if((DeviceEvent & DEVICE_P_OIP_FULL_CLOSE_CAN_LIN_EVENT) != 0)  //Full Close mode input CAN->LIN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_FULL_CLOSE_CAN_LIN_EVENT;

			Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;		
		}
#ifdef AMO_GN7_PE_SETTING_NONE
		else if((DeviceEvent & DEVICE_CTS_FULL_CLOSE_CAN_LIN_EVENT) != 0) //Full Close input CAN->LIN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CTS_FULL_CLOSE_CAN_LIN_EVENT;

			Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;
		}
		else if((DeviceEvent & DEVICE_P_OIP_FULL_CLOSE_LIN_CAN_EVENT) != 0)  //Full Close mode input LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_FULL_CLOSE_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;		
		}
		else if((DeviceEvent & DEVICE_CTS_FULL_CLOSE_LIN_CAN_EVENT) != 0) //Full Close input LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CTS_FULL_CLOSE_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_FULL_CLOSE_MODE_INIT;
		}
#endif
		else if((DeviceEvent & DEVICE_P_OIP_FOCUS_CAN_LIN_EVENT) != 0)	//Focus mode input CAN->LIN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_FOCUS_CAN_LIN_EVENT;

			Board_state = DEVICE_STATE_FOCUS_MODE_INIT; 	
		}
#ifdef AMO_GN7_PE_SETTING_NONE
		else if((DeviceEvent & DEVICE_CTS_FOCUS_CAN_LIN_EVENT) != 0) //Focus input CAN->LIN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CTS_FOCUS_CAN_LIN_EVENT;

			Board_state = DEVICE_STATE_FOCUS_MODE_INIT;
		}
		else if((DeviceEvent & DEVICE_P_OIP_FOCUS_LIN_CAN_EVENT) != 0)	//Focus mode input LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_FOCUS_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_FOCUS_MODE_INIT; 	
		}
		else if((DeviceEvent & DEVICE_CTS_FOCUS_LIN_CAN_EVENT) != 0) //Focus input LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CTS_FOCUS_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_FOCUS_MODE_INIT;
		}
#endif
		else if((DeviceEvent & DEVICE_P_OIP_SPREAD_CAN_LIN_EVENT) != 0)  //SPREAD mode input CAN->LIN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_SPREAD_CAN_LIN_EVENT;

			Board_state = DEVICE_STATE_SPREAD_MODE_INIT;		
		}
#ifdef AMO_GN7_PE_SETTING_NONE
		else if((DeviceEvent & DEVICE_CTS_SPREAD_CAN_LIN_EVENT) != 0) //SPREAD input CAN->LIN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CTS_SPREAD_CAN_LIN_EVENT;

			Board_state = DEVICE_STATE_SPREAD_MODE_INIT;
		}
		else if((DeviceEvent & DEVICE_CTS_FOCUS_LIN_CAN_EVENT) != 0)	//SPREAD mode input LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CTS_FOCUS_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_SPREAD_MODE_INIT;		
		}
		else if((DeviceEvent & DEVICE_STICK_FOCUS_LIN_CAN_EVENT) != 0) //SPREAD input LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_STICK_FOCUS_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_SPREAD_MODE_INIT;
		}
#endif
		else if((DeviceEvent & DEVICE_P_OIP_CYCLE_CAN_LIN_EVENT) != 0)	//Cycle mode input CAN->LIN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_CYCLE_CAN_LIN_EVENT;

			Board_state = DEVICE_STATE_CYCLE_MODE_INIT; 	
		}
#ifdef AMO_GN7_PE_SETTING_NONE
		else if((DeviceEvent & DEVICE_CTS_CYCLE_CAN_LIN_EVENT) != 0)//Cycle input CAN->LIN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CTS_CYCLE_CAN_LIN_EVENT;

			Board_state = DEVICE_STATE_CYCLE_MODE_INIT;
		}
		else if((DeviceEvent & DEVICE_P_OIP_CYCLE_LIN_CAN_EVENT) != 0)	//Cycle mode input LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_P_OIP_CYCLE_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_CYCLE_MODE_INIT; 	
		}
		else if((DeviceEvent & DEVICE_CTS_CYCLE_LIN_CAN_EVENT) != 0) //Cycle input LIN->CAN
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CTS_CYCLE_LIN_CAN_EVENT;

			Board_state = DEVICE_STATE_CYCLE_MODE_INIT;
		}
#endif
		else if((DeviceEvent & DEVICE_ACTUATOR_RECOVERY_EVENT) != 0) //Recovery mode
		{
			DeviceEvent = DeviceEvent & ~DEVICE_ACTUATOR_RECOVERY_EVENT;

			Board_state = DEVICE_STATE_RECOVERY_MODE_INIT;
		}
		else if((DeviceEvent & DEVICE_ACTUATOR_RECOVERY_RELEASE_EVENT) != 0) //Recovery release
		{
			DeviceEvent = DeviceEvent & ~DEVICE_ACTUATOR_RECOVERY_RELEASE_EVENT;

			Board_state = DEVICE_STATE_RECOVERY_MAIN;
		}
		else if((DeviceEvent & DEVICE_ACTUATOR_SECURE_EVENT) != 0) //Recovery mode
		{
			DeviceEvent = DeviceEvent & ~DEVICE_ACTUATOR_SECURE_EVENT;

			Board_state = DEVICE_STATE_SECURE_MODE_INIT;
		}
		else if((DeviceEvent & DEVICE_ACTUATOR_SECURE_RELEASE_EVENT) != 0) //Recovery release
		{
			DeviceEvent = DeviceEvent & ~DEVICE_ACTUATOR_SECURE_RELEASE_EVENT;

			Board_state = DEVICE_STATE_SECURE_MODE_INIT;
		}
		else if((DeviceEvent & DEVICE_CAN_EVNT_UTILITY_MODE_EVENT) != 0) //Recovery release
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CAN_EVNT_UTILITY_MODE_EVENT;

			Board_state = DEVICE_STATE_UTILITY_MODE_INIT;
		} 			
		else if((DeviceEvent & DEVICE_CAN_RXDATA_TIMEOUT_EVENT) != 0)  //RX Timeout
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CAN_RXDATA_TIMEOUT_EVENT;

			Board_state = DEVICE_RX_TIMEOUT_INIT;
		}
		else if((DeviceEvent & DEVICE_CAN_RXDATA_RECOVERY_EVENT) != 0)	//RX Recovery
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CAN_RXDATA_RECOVERY_EVENT;

			Board_state = DEVICE_RX_RECOVERY_INIT;
		} 		
		else if((DeviceEvent & DEVICE_CAN_EVNT_ATCU_SEND_EVENT) != 0)  
		{
			DeviceEvent = DeviceEvent & ~DEVICE_CAN_EVNT_ATCU_SEND_EVENT;

			Board_state = DEVICE_EVNT_ATCU_SEND_INIT;
		}
		else if((DeviceEvent & DEVICE_EVNT_DTC_EVENT) != 0)  
		{
			DeviceEvent = DeviceEvent & ~DEVICE_EVNT_DTC_EVENT;

			Board_state = DEVICE_STATC_DTC_MAIN;
		} 
		else if((DeviceEvent & DEVICE_EVNT_OTA_EVENT) != 0)  
		{
			DeviceEvent = DeviceEvent & ~DEVICE_EVNT_OTA_EVENT;

			Board_state = DEVICE_STATC_FW_OTA_MAIN;
		} 
		else if((DeviceEvent & DEVICE_SLEEP_ENTER_EVENT) != 0)	
		{
			DeviceEvent = DeviceEvent & ~DEVICE_SLEEP_ENTER_EVENT;

			Board_state = DEVICE_STATE_SLEEP_ENTER;
		}
		else if((DeviceEvent & DEVICE_ATCU_TX_EVENT) != 0)	
		{
			DeviceEvent = DeviceEvent & ~DEVICE_ATCU_TX_EVENT;

			Board_state = DEVICE_STATC_ATCU_TX_MAIN;
		} 				
		else
		{
			Board_state = DEVICE_STATE_SLEEP_INIT;
		}

}


#endif

