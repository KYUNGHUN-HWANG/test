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
void Manual_side_type(void);
void Focus_side_type(void);
void Spread_side_type(void);
void Cycle_side_type(void);
void Full_close_side_type(void);
void Manual_dr_lin_type(void);
void Manual_pr_lin_type(void);

/*
* @brief Manual_dr_lin_type function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Manual_dr_lin_type(void)
{
	if(ATCU_DriverSideType == LHD)
	{
		FrDrLinData_Parsing(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
//		lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);
		Hu_FrDrEVntSidePt_X_invalid = 2047, Hu_FrDrEVntSidePt_Y_invalid = 2047, Hu_FrDrEVntCtrPt_X_invalid = 2047, Hu_FrDrEVntCtrPt_Y_invalid = 2047; 							
	}
	else if(ATCU_DriverSideType == RHD)
	{
		RHD_FrPsLinData_Parsing(&LIN_RH_EVNT_MASTER_CMD);
		lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);
		Hu_FrDrEVntSidePt_X_invalid = 2047, Hu_FrDrEVntSidePt_Y_invalid = 2047, Hu_FrDrEVntCtrPt_X_invalid = 2047, Hu_FrDrEVntCtrPt_Y_invalid = 2047;
	}
	else
	{
	}
}

/*
* @brief Manual_pr_lin_type function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Manual_pr_lin_type(void)
{
	if(ATCU_DriverSideType == LHD)
	{
//						CanData_Update_Check_FrPsPt(&Can_data_21_Info);
		FrPsLinData_Parsing(&LIN_RH_EVNT_MASTER_CMD);
		lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);
		Hu_FrPsEVntSidePt_X_invalid = 2047, Hu_FrPsEVntSidePt_Y_invalid = 2047, Hu_FrPsEVntCtrPt_X_invalid = 2047, Hu_FrPsEVntCtrPt_Y_invalid = 2047;
	}
	else if(ATCU_DriverSideType == RHD)
	{
		RHD_FrDrLinData_Parsing(&LIN_LH_EVNT_MASTER_CMD);
		lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);
		Hu_FrPsEVntSidePt_X_invalid = 2047, Hu_FrPsEVntSidePt_Y_invalid = 2047, Hu_FrPsEVntCtrPt_X_invalid = 2047, Hu_FrPsEVntCtrPt_Y_invalid = 2047;
	}
	else
	{
	}
}

/*
* @brief Manual_side_type function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Manual_side_type(void)
{
	if(Touch_mode_Dr_set == HU_DRIVER_TOUCH)
	{
		/*driver manual mode drive init */
		Touch_mode_Dr_set = NONE_DRIVER;
#ifdef KEY_ENABLE
		PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif									
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Manual_dr_evt_signal();
		Manual_dr_lin_type();
	}
	if(Touch_mode_Ps_set == HU_PASSENGER_TOUCH)
	{
		/*passenger manual mode drive init */
		Touch_mode_Ps_set = NONE_PASSENGER;
#ifdef KEY_ENABLE
		PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif				
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x04; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Manual_ps_evt_signal();
		Manual_pr_lin_type();
	}
}

/*
* @brief Focus_side_type function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Focus_side_type(void)
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
}

/*
* @brief Spread_side_type function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Spread_side_type(void)
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
}

/*
* @brief Spread_side_type function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Cycle_side_type(void)
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
}

/*
* @brief Full_close_side_type function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Full_close_side_type(void)
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
}
#endif

