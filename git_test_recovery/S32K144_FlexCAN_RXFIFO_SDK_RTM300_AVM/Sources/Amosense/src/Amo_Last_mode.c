#ifndef AMO_VENTMODE_C
#define AMO_VENTMODE_C

/*===========================================================================*/
/* Project   :  AMOSENSE VENT MODE function Software                                                                */
/* File name :  Amo_Vent_mode.c                                                                             */
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
#include "Amo_Cycle.h"
#include "Amo_Vent_mode.h"
#include "Amo_System_setting.h"
#include "Amo_Last_mode.h"
/* Ptoto type function define */

static void RHD_Hu_Front_FullClose_mode_setting_Backup(void);
static void Hu_Front_FullClose_mode_setting_Backup(void);

/*
* @brief Hu_Front_FullClose_mode_setting_Backup function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Hu_Front_FullClose_mode_setting_Backup(void)
{
	if((Hu_Frdr_Vent_Side_sig_backup == 0x02) || (Hu_Frdr_Vent_Center_sig_backup == 0x02) || (Hu_FrPs_Vent_Side_sig_backup == 0x02) || (Hu_FrPs_Vent_Center_sig_backup == 0x02)) //Full close mode setting
	{
		if(Hu_Frdr_Vent_Side_sig_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
			lin_FrDrVent_FullClose_Backup(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
		}

		if(Hu_Frdr_Vent_Center_sig_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
			lin_FrDrVent_FullClose_Backup(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
		}

		if(Hu_FrPs_Vent_Side_sig_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
			lin_FrPsVent_FullClose_Backup(&LIN_RH_EVNT_MASTER_CMD);
		}

		if(Hu_FrPs_Vent_Center_sig_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
			lin_FrPsVent_FullClose_Backup(&LIN_RH_EVNT_MASTER_CMD);
		}
	}
}

/*
* @brief RHD_Hu_Front_FullClose_mode_setting_Backup function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void RHD_Hu_Front_FullClose_mode_setting_Backup(void)
{
	if((Hu_Frdr_Vent_Side_sig_backup == 0x02) || (Hu_Frdr_Vent_Center_sig_backup == 0x02) || (Hu_FrPs_Vent_Side_sig_backup == 0x02) || (Hu_FrPs_Vent_Center_sig_backup == 0x02)) //Full close mode setting
	{
		if(Hu_Frdr_Vent_Side_sig_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
			RHD_lin_FrPsVent_FullClose_Backup(&LIN_RH_EVNT_MASTER_CMD);
		}

		if(Hu_Frdr_Vent_Center_sig_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
			RHD_lin_FrPsVent_FullClose_Backup(&LIN_RH_EVNT_MASTER_CMD);
		}

		if(Hu_FrPs_Vent_Side_sig_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
			RHD_lin_FrDrVent_FullClose_Backup(&LIN_LH_EVNT_MASTER_CMD);
		}

		if(Hu_FrPs_Vent_Center_sig_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
			RHD_lin_FrDrVent_FullClose_Backup(&LIN_LH_EVNT_MASTER_CMD);
		}
	}
}

/*
* @brief Last_full_close_mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Last_full_close_mode(void)
{
	if(ATCU_DriverSideType == LHD)
	{
		Hu_Front_FullClose_mode_setting_Backup();
	}
	else if(ATCU_DriverSideType == RHD)
	{
		RHD_Hu_Front_FullClose_mode_setting_Backup();
	}
	else
	{
	}
}

/*
* @brief Last_dr_foucs_mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Last_dr_foucs_mode(void)
{
	if(ATCU_DriverSideType == LHD)
	{
		lin_FrDrVent_Focus_Backup(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
	}
	else if(ATCU_DriverSideType == RHD)
	{
		RHD_lin_FrPsVent_Focus_Backup(&LIN_RH_EVNT_MASTER_CMD);
	}
	else
	{
	}
}

/*
* @brief Last_ps_foucs_mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Last_ps_foucs_mode(void)
{
	if(ATCU_DriverSideType == LHD)
	{
		lin_FrPsVent_Focus_Backup(&LIN_RH_EVNT_MASTER_CMD);
	}
	else if(ATCU_DriverSideType == RHD)
	{
		RHD_lin_FrDrVent_Focus_Backup(&LIN_LH_EVNT_MASTER_CMD);
	}
	else
	{
	}
}

/*
* @brief Last_dr_spread_mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Last_dr_spread_mode(void)
{
	if(ATCU_DriverSideType == LHD)
	{
		lin_FrDrVent_Spread_Backup(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
	}
	else if(ATCU_DriverSideType == RHD)
	{
		RHD_lin_FrPsVent_Spread_Backup(&LIN_RH_EVNT_MASTER_CMD);
	}
	else
	{
	}
}

/*
* @brief Last_ps_spread_mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Last_ps_spread_mode(void)
{
	if(ATCU_DriverSideType == LHD)
	{
		lin_FrPsVent_Spread_Backup(&LIN_RH_EVNT_MASTER_CMD);
	}
	else if(ATCU_DriverSideType == RHD)
	{
		RHD_lin_FrDrVent_Spread_Backup(&LIN_LH_EVNT_MASTER_CMD);
	}
	else
	{
	}
}

/*
* @brief Last_dr_cycle_mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Last_dr_cycle_mode(void)
{
	if(ATCU_DriverSideType == LHD)
	{
		lin_FrDrCycleMode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);
	}
	else if(ATCU_DriverSideType == RHD)
	{
		lin_FrPsCycleMode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
	}
	else
	{
	}
}

/*
* @brief Last_ps_cycle_mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Last_ps_cycle_mode(void)
{
	if(ATCU_DriverSideType == LHD)
	{
		lin_FrPsCycleMode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
	}
	else if(ATCU_DriverSideType == RHD)
	{
		lin_FrDrCycleMode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);
	}
	else
	{
	}
}

/*
* @brief Last_dr_manual_mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Last_dr_manual_mode(void)
{
	if(ATCU_DriverSideType == LHD)
	{
		FrDrLinData_Parsing_Backup(&LIN_LH_EVNT_MASTER_CMD);
		lin_FrDrmanualMode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);
		}
	else if(ATCU_DriverSideType == RHD)
	{
		RHD_FrPsLinData_Parsing_Backup(&LIN_RH_EVNT_MASTER_CMD);
		lin_FrPsmanualMode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
	}
	else
	{
	}
}

/*
* @brief Last_ps_manual_mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Last_ps_manual_mode(void)
{
	if(ATCU_DriverSideType == LHD)
	{
		FrPsLinData_Parsing_Backup(&LIN_RH_EVNT_MASTER_CMD);
		lin_FrPsmanualMode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
	}
	else if(ATCU_DriverSideType == RHD)
	{
		RHD_FrDrLinData_Parsing_Backup(&LIN_LH_EVNT_MASTER_CMD);
		lin_FrDrmanualMode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);
	}
	else
	{
	}
}

#ifdef AMO_NVM_SETTING
/*
* @brief Last_Mode_dr_fr_set_fullclose_func function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Last_Mode_dr_fr_set_fullclose_func(void)
{
	if((Hu_Frdr_Vent_Side_sig_backup == 0x02) || (Hu_Frdr_Vent_Center_sig_backup == 0x02) || (Hu_FrPs_Vent_Side_sig_backup == 0x02) || (Hu_FrPs_Vent_Center_sig_backup == 0x02))
	{
		if(Hu_Frdr_Vent_Side_sig_backup == 0x02)
		{
			/*Full close driver side setup*/
			//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00;											
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x02;
		}

		if(Hu_Frdr_Vent_Center_sig_backup == 0x02)
		{
			/*Full close driver center setup*/
			//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00;								
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x02;
		}

		if(Hu_FrPs_Vent_Side_sig_backup == 0x02)
		{
			/*Full close passenger side setup*/
			//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00;										
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x02;
		}

		if(Hu_FrPs_Vent_Center_sig_backup == 0x02)
		{
			/*Full close passenger center setup*/
			//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00;														
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x02;
		}
		
		Last_full_close_mode();

	#ifdef AMO_GN7_PE_SETTING_NONE
		HU_Rear_mode_setting();
	#endif
	}
}

/*
* @brief Last_Mode_dr_fr_set_func function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Last_Mode_dr_fr_set_func(void)
{
	if(((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Frdr_Vent_Side_sig_backup != 0x02)) ||((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Frdr_Vent_Center_sig_backup != 0x02)))
	{
		Last_dr_foucs_mode();
#ifdef KEY_ENABLE
		if(key_full_left_close == 1)
		{
			key_full_left_close = 0;
		}
	
		if(Hu_Fr_dr_Vent_Center_signal == 0x02)
		{
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}
	
		else if(Hu_Fr_dr_Vent_Side_signal == 0x02)
		{
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}
		else
		{
			PINS_DRV_SetPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}
#endif
	}
	if(((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_FrPs_Vent_Side_sig_backup != 0x02)) ||((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_FrPs_Vent_Center_sig_backup != 0x02)))
	{
		Last_ps_foucs_mode();
#ifdef KEY_ENABLE
		if(key_full_right_close == 1)
		{
			key_full_right_close = 0;
		}
		if(Hu_Fr_Ps_Vent_Center_signal == 0x02)
		{
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}

		else if(Hu_Fr_Ps_Vent_Side_signal == 0x02)
		{
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}
		else
		{
			PINS_DRV_SetPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
		}
#endif
	}
	
	if(((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Frdr_Vent_Side_sig_backup != 0x02)) ||((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Frdr_Vent_Center_sig_backup != 0x02)))
	{
		Last_dr_spread_mode();
#ifdef KEY_ENABLE
		if(key_full_left_close == 1)
		{
			key_full_left_close = 0;
		}

		if(Hu_Fr_dr_Vent_Center_signal == 0x02)
		{
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}

		else if(Hu_Fr_dr_Vent_Side_signal == 0x02)
		{
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}
		else
		{
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_SetPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}
#endif
	}

	if(((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_FrPs_Vent_Side_sig_backup != 0x02)) ||((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_FrPs_Vent_Center_sig_backup != 0x02)))
	{
		Last_ps_spread_mode();
#ifdef KEY_ENABLE
		if(key_full_right_close == 1)
		{
			key_full_right_close = 0;
		}
		if(Hu_Fr_Ps_Vent_Center_signal == 0x02)
		{
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}
		else if(Hu_Fr_Ps_Vent_Side_signal == 0x02)
		{
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}
		else
		{
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_SetPins(PTC,RH_AVOID_INDI_PWM_MASK);
		}
#endif
	}

	if((Hu_Fr_Dr_Vent_Mode == SWING_MODE) && (Hu_Fr_Ps_Vent_Mode == SWING_MODE) && (Hu_Frdr_Vent_Side_sig_backup != 0x02) && (Hu_Frdr_Vent_Center_sig_backup != 0x02) && (Hu_FrPs_Vent_Side_sig_backup != 0x02) && (Hu_FrPs_Vent_Center_sig_backup != 0x02))
	{
		lin_FrDrPsCycleMode_task_Backup();
#ifdef KEY_ENABLE
		if(key_full_left_close == 1)
		{
			key_full_left_close = 0;
		}
		if(key_full_right_close == 1)
		{
			key_full_right_close = 0;
		}
		button_LH_cycle_check = 0x03;
		button_RH_cycle_check = 0x03;
		PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
	}
	else if((Hu_Fr_Dr_Vent_Mode == SWING_MODE)&&(Hu_Frdr_Vent_Side_sig_backup != 0x02) &&(Hu_Frdr_Vent_Side_sig_backup != 0x02))
	{
		Last_dr_cycle_mode();
#ifdef KEY_ENABLE
		if(key_full_left_close == 1)
		{
			key_full_left_close = 0;
		}
		button_LH_cycle_check = 0x03;
		PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
	}
	else if((Hu_Fr_Ps_Vent_Mode == SWING_MODE) && (Hu_FrPs_Vent_Side_sig_backup != 0x02) && (Hu_FrPs_Vent_Side_sig_backup != 0x02))
	{
		Last_ps_cycle_mode();
#ifdef KEY_ENABLE
		button_RH_cycle_check = 0x03;
		if(key_full_right_close == 1)
		{
			key_full_right_close = 0;
		}
		PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
	}

	else
	{
	}

	if((Hu_Fr_Dr_Vent_Mode == FREE_MODE) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE))
	{
		if(((Hu_Fr_Dr_Vent_Mode == FREE_MODE)&&(Hu_Frdr_Vent_Side_sig_backup != 0x02)) || ((Hu_Fr_Dr_Vent_Mode == FREE_MODE)&&(Hu_Frdr_Vent_Side_sig_backup != 0x02)))
		{
			/*Vent info setup*/
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04;														
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
			Last_dr_manual_mode();
		}

		if(((Hu_Fr_Ps_Vent_Mode == FREE_MODE)&&(Hu_FrPs_Vent_Side_sig_backup != 0x02)) || ((Hu_Fr_Ps_Vent_Mode == FREE_MODE)&&(Hu_FrPs_Vent_Side_sig_backup != 0x02)))
		{               
			/*Vent info setup*/
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x04;													
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
			Last_ps_manual_mode();
		}
	}

}
#endif

#endif

