/*
 * Amo_Mode_Setting.c
 *
 *  Created on: 2025. 1. 6.
 *      Author: S2402004
 */
#ifndef __AMO_MODE_C
#define __AMO_MODE_C

#include "Cpu.h"
#include "clockMan1.h"

#include "dmaController1.h"
#include "pin_mux.h"
#include "watchdog1.h"
#include "pwrMan1.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stdlib.h"

#include "Amo_main.h"
#include "Amo_CAN_Parsing.h"
#include "Amo_Mode_Setting.h"
#include "ewl_misra_types.h"
#include "Amo_STATE_Event.h"
#include "Amo_timer.h"
#include "Amo_nvm.h"
#include "Amo_Lin.h"
#include "Amo_Cycle.h"

extern uint8_t button_LH_cycle_check;
extern uint8_t button_RH_cycle_check;
extern uint8_t key_full_left_close;
extern uint8_t key_full_right_close;

///////////////////////////////////////////////////////////////////////////////
//CAN TX
extern Lvnt_1_data Can_Tx_Evnt_1;
extern Lvnt_2_data Can_Tx_Evnt_2;
extern Lvnt_3_data Can_Tx_Evnt_3;
extern Lvnt_4_data Can_Tx_Evnt_4;
extern Lvnt_5_data Can_Tx_Evnt_5;
extern Lvnt_6_data Can_Tx_Evnt_6;
extern Lvnt_7_data Can_Tx_Evnt_7;
extern Lvnt_8_data Can_Tx_Evnt_8;
extern Lvnt_9_data Can_Tx_Evnt_9;

///////////////////////////////////////////////////////////////////////////////

extern Atcu_33_data Can_data_33_Info;

extern Evnt_mode Cts_Fr_Dr_Vent_Mode;
extern Evnt_mode Cts_Fr_Ps_Vent_Mode;
extern Evnt_mode Cts_Fr_Dr_Vent_Mode_remember;
extern Evnt_mode Cts_Fr_Ps_Vent_Mode_remember;

extern uint8_t Cts_Fr_dr_Vent_Side_siganl;
extern uint8_t Cts_Fr_dr_Vent_Center_signal;
extern uint8_t Cts_Fr_Ps_Vent_Side_signal;
extern uint8_t Cts_Fr_Ps_Vent_Center_signal;

///////////////////////////////////////////////////////////////////////////////

extern Evnt_mode Hu_Fr_Dr_Vent_Mode_Remember;
extern Evnt_mode Hu_Fr_Ps_Vent_Mode_Remember;

///////////////////////////////////////////////////////////////////////////////

extern Atcu_28_data Can_data_28_Info;  //CTS RECEIVED 

extern Evnt_mode Cts_Rr_Dr_Vent_Mode;
extern Evnt_mode Cts_Rr_Ps_Vent_Mode;
extern Evnt_mode Cts_Rr_Dr_Vent_Mode_remember;
extern Evnt_mode Cts_Rr_Ps_Vent_Mode_remember;

extern uint8_t Cts_Rr_Dr_Vent_siganl;
extern uint8_t Cts_Rr_Ps_Vent_siganl;

///////////////////////////////////////////////////////////////////////////////

extern Atcu_22_data Can_data_22_Info; //P-OIP RECEIVED Rear driver/passenger

extern Evnt_mode Hu_Rr_Dr_Vent_Mode;
extern Evnt_mode Hu_Rr_Ps_Vent_Mode;
extern Evnt_mode Hu_Rr_Dr_Vent_Mode_Remember;
extern Evnt_mode Hu_Rr_Ps_Vent_Mode_Remember;

///////////////////////////////////////////////////////////////////////////////

extern uint8_t P_Oip_Cts_priority;
extern Drv_type ATCU_DriverSideType;

extern uint8_t Hu_Fr_dr_Vent_Side_signal_backup;
extern uint8_t Hu_Fr_dr_Vent_Center_signal_backup;
extern uint8_t Hu_Fr_Ps_Vent_Side_signal_backup;
extern uint8_t Hu_Fr_Ps_Vent_Center_signal_backup;

void Hu_Front_mode_FullClose_setting()
{
	if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02 || Hu_Fr_dr_Vent_Center_signal_backup == 0x02 || Hu_Fr_Ps_Vent_Side_signal_backup == 0x02 || Hu_Fr_Ps_Vent_Center_signal_backup == 0x02) //Full close mode setting
	{
		if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
			lin_FrDrVent_FullClose_Mode_task(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
		}
		if(Hu_Fr_dr_Vent_Center_signal_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
			lin_FrDrVent_FullClose_Mode_task(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
		}

		if(Hu_Fr_Ps_Vent_Side_signal == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
			lin_FrPsVent_FullClose_Mode_task(&LIN_RH_EVNT_MASTER_CMD);

		}

		if(Hu_Fr_Ps_Vent_Center_signal == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
			lin_FrPsVent_FullClose_Mode_task(&LIN_RH_EVNT_MASTER_CMD);
		}
		
		Hu_Fr_dr_Vent_Side_signal = 0x00;
		Hu_Fr_dr_Vent_Center_signal = 0x00;
		Hu_Fr_Ps_Vent_Side_signal = 0x00;
		Hu_Fr_Ps_Vent_Center_signal = 0x00;
	}
}



void RHD_Hu_Front_mode_FullClose_setting()
{
	if(Hu_Fr_dr_Vent_Side_signal == 0x02 || Hu_Fr_dr_Vent_Center_signal == 0x02 || Hu_Fr_Ps_Vent_Side_signal == 0x02 || Hu_Fr_Ps_Vent_Center_signal == 0x02) //Full close mode setting
	{
		if(Hu_Fr_dr_Vent_Side_signal == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
			RHD_lin_FrPsVent_FullClose_Mode_task(&LIN_RH_EVNT_MASTER_CMD);
		}
		if(Hu_Fr_dr_Vent_Center_signal == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
			RHD_lin_FrPsVent_FullClose_Mode_task(&LIN_RH_EVNT_MASTER_CMD);
		}

		if(Hu_Fr_Ps_Vent_Side_signal == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
			RHD_lin_FrDrVent_FullClose_Mode_task(&LIN_LH_EVNT_MASTER_CMD);

		}

		if(Hu_Fr_Ps_Vent_Center_signal == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
			RHD_lin_FrDrVent_FullClose_Mode_task(&LIN_LH_EVNT_MASTER_CMD);
		}
	}
}


void Hu_Front_Spread_mode_setting()
{
if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE)
	{
		lin_FrDrVent_Spread_Mode_task(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
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
if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE)
	{
		lin_FrPsVent_Spread_Mode_task(&LIN_RH_EVNT_MASTER_CMD);
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
}

void RHD_Hu_Front_Spread_mode_setting()
{
if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE)
	{
		RHD_lin_FrPsVent_Spread_Mode_task(&LIN_RH_EVNT_MASTER_CMD);
		
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
	if((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01))
	{
		RHD_lin_FrDrVent_Spread_Mode_task(&LIN_LH_EVNT_MASTER_CMD);
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
}


void Hu_Front_Cycle_mode_setting()
{
	//if(((Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01)) && ((Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01)))
	if((Hu_Fr_Dr_Vent_Mode == SWING_MODE) && (Hu_Fr_Ps_Vent_Mode == SWING_MODE))
		{
			lin_FrDrPsCycleMode_task();
			
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
		//else if((Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01))
		else if(Hu_Fr_Dr_Vent_Mode == SWING_MODE)
		{
			lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
			
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
		else if(Hu_Fr_Ps_Vent_Mode == SWING_MODE)
		{
			lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
			
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
}

void RHD_Hu_Front_Cycle_mode_setting()
{
	if(((Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01)) && ((Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01)))
		{
			lin_FrDrPsCycleMode_task();
			
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
		else if((Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01))
		{
			lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
			
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
		else if((Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01))
		{
			lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
			
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
}



void Hu_Front_Focus_mode_setting(void)
{
		if(Hu_Fr_Dr_Vent_Mode == FACING_MODE)
		{
			lin_FrDrVent_Focus_Mode_task(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
		
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
		if(Hu_Fr_Ps_Vent_Mode == FACING_MODE)
		{
			lin_FrPsVent_Focus_Mode_task(&LIN_RH_EVNT_MASTER_CMD);
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
		else
		{
		}
}

void RHD_Hu_Front_Focus_mode_setting(void)
{
		if((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01))
		{
			RHD_lin_FrPsVent_Focus_Mode_task(&LIN_RH_EVNT_MASTER_CMD);
		
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
		
		if(Hu_Fr_Ps_Vent_Mode == FACING_MODE)
		{
			RHD_lin_FrDrVent_Focus_Mode_task(&LIN_LH_EVNT_MASTER_CMD);
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
		else
		{
		}
}


void Hu_Front_FullClose_mode_setting_Backup()
{
	if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02 || Hu_Fr_dr_Vent_Center_signal_backup == 0x02 || Hu_Fr_Ps_Vent_Side_signal_backup == 0x02 || Hu_Fr_Ps_Vent_Center_signal_backup == 0x02) //Full close mode setting
	{
		if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
			lin_FrDrVent_FullClose_Mode_task_Backup(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
		}

		if(Hu_Fr_dr_Vent_Center_signal_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
			lin_FrDrVent_FullClose_Mode_task_Backup(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
		}

		if(Hu_Fr_Ps_Vent_Side_signal_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
			lin_FrPsVent_FullClose_Mode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
		}

		if(Hu_Fr_Ps_Vent_Center_signal_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
			lin_FrPsVent_FullClose_Mode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
		}
	}
}

void RHD_Hu_Front_FullClose_mode_setting_Backup()
{
	if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02 || Hu_Fr_dr_Vent_Center_signal_backup == 0x02 || Hu_Fr_Ps_Vent_Side_signal_backup == 0x02 || Hu_Fr_Ps_Vent_Center_signal_backup == 0x02) //Full close mode setting
	{
		if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
			RHD_lin_FrPsVent_FullClose_Mode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
		}

		if(Hu_Fr_dr_Vent_Center_signal_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
#endif
			RHD_lin_FrPsVent_FullClose_Mode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
		}

		if(Hu_Fr_Ps_Vent_Side_signal_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
			RHD_lin_FrDrVent_FullClose_Mode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);
		}

		if(Hu_Fr_Ps_Vent_Center_signal_backup == 0x02)
		{
#ifdef KEY_ENABLE
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
#endif
			RHD_lin_FrDrVent_FullClose_Mode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);
		}
	}
}



void Hu_Front_Focus_mode_setting_Backup()  //nv restore setting
{

	if(Hu_Fr_Dr_Vent_Mode == FACING_MODE)
	{
		lin_FrDrVent_Focus_Mode_task_Backup(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);

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

	if(Hu_Fr_Ps_Vent_Mode == FACING_MODE)
	{
		lin_FrPsVent_Focus_Mode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
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
	
	else
	{
	}
}

void Hu_Front_Spread_mode_setting_Backup()  //nv restore setting
{
	if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE)
	{
		lin_FrDrVent_Spread_Mode_task_Backup(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);

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

	if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE)
	{
		lin_FrPsVent_Spread_Mode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);

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
	
	else
	{
	}
}

void Hu_Front_Cycle_mode_setting_Backup()  //nv restore setting
{
	if(((Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Side_signal_backup == 0x01) || (Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01)) && ((Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01)))
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
	else if((Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01))
	{
		lin_FrDrCycleMode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);

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
	else if((Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01))
	{
		lin_FrPsCycleMode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);

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
}


/*
void Hu_Front_mode_setting_Backup()  //nv restore setting
{
	if((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01))
	{
		lin_FrDrVentMode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);

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
		else
		{
			PINS_DRV_SetPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}
#endif
	}

	if((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01))
	{
		lin_FrDrVentMode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);

#ifdef KEY_ENABLE
		if(key_full_left_close == 1)
		{
			key_full_left_close = 0;
		}

		if(Hu_Fr_dr_Vent_Side_signal == 0x02)
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

	if((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01))
	{
		lin_FrPsVentMode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
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
		else
		{
			PINS_DRV_SetPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
		}
#endif
	}

	if((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01))
	{
		lin_FrPsVentMode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);

#ifdef KEY_ENABLE
		if(key_full_right_close == 1)
		{
			key_full_right_close = 0;
		}

		if(Hu_Fr_Ps_Vent_Side_signal == 0x02)
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

	if((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01))
	{
		lin_FrDrVentMode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);

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
		else
		{
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_SetPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}
#endif
	}

	if((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01))
	{
		lin_FrDrVentMode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);

#ifdef KEY_ENABLE
		if(key_full_left_close == 1)
		{
			key_full_left_close = 0;
		}

		if(Hu_Fr_dr_Vent_Side_signal == 0x02)
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

	if((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01))
	{
		lin_FrPsVentMode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);

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
		else
		{
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_SetPins(PTC,RH_AVOID_INDI_PWM_MASK);
		}
#endif
	}

	if((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01))
	{
		lin_FrPsVentMode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);

#ifdef KEY_ENABLE
		if(key_full_right_close == 1)
		{
			key_full_right_close = 0;
		}
		if(Hu_Fr_Ps_Vent_Side_signal == 0x02)
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

	if(((Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01)) && ((Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01)))
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
	else if((Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01))
	{
		lin_FrDrCycleMode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);

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
	else if((Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01))
	{
		lin_FrPsCycleMode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);

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
}
*/

#ifdef KEY_ENABLE
void Hu_Front_mode_manual_key_setting()
{
if(button_LH_cycle_check == 0x03 && button_RH_cycle_check == 0x03)
{
	if((Hu_Fr_Ps_Vent_Mode == FREE_MODE) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE))
	{

		LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = 0;
		LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
		LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = 0;
		LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;

		lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);

		PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
	}
		if((Hu_Fr_Dr_Vent_Mode == FREE_MODE ) || (Hu_Fr_Dr_Vent_Mode == FREE_MODE))
		{

		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 0;
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 0;
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;

		lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);

		PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}
}

else if(button_LH_cycle_check == 0x03) //|| button_RH_cycle_check == 0x03) //|| button_RH_cycle_check == 0x03)
{
	if((Hu_Fr_Dr_Vent_Mode == FREE_MODE) || (Hu_Fr_Dr_Vent_Mode == FREE_MODE))
	{

		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 0;
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 0;
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;

		lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);

		PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
	}
}
else if(button_RH_cycle_check == 0x03)
{
	if((Hu_Fr_Ps_Vent_Mode == FREE_MODE) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE))
	{
		LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = 0;
		LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
		LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = 0;
		LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;

		lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);
			
		PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
	}
}

if(button_LH_cycle_check == 0x04 && button_RH_cycle_check == 0x04)
{
	if((Hu_Fr_Dr_Vent_Mode == FREE_MODE) || (Hu_Fr_Dr_Vent_Mode == FREE_MODE))
		{
			FrDrLinData_Parsing(&LIN_LH_EVNT_MASTER_CMD);
			lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);

			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);

		}

	if((Hu_Fr_Ps_Vent_Mode == FREE_MODE) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE))
		{
			FrPsLinData_Parsing(&LIN_RH_EVNT_MASTER_CMD);
			lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);

			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
		}
}

else if(button_LH_cycle_check == 0x04)
{
	if((Hu_Fr_Dr_Vent_Mode == FREE_MODE) || (Hu_Fr_Dr_Vent_Mode == FREE_MODE))
	{
		FrDrLinData_Parsing(&LIN_LH_EVNT_MASTER_CMD);
		lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);
		
		PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
	}
}

else if(button_RH_cycle_check == 0x04)
{

	if((Hu_Fr_Ps_Vent_Mode == FREE_MODE) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE))
	{
		FrPsLinData_Parsing(&LIN_RH_EVNT_MASTER_CMD);
		lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);

		PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
	}
}
}
#endif


#ifdef AMO_GN7_PE_SETTING_NONE
void HU_Rear_mode_setting()
{
	if((Hu_Rr_Dr_Vent_signal == 0x02) || (Hu_Rr_Dr_Vent_signal == 0x02))   //Full close mode setting
	{
		if(Hu_Rr_Dr_Vent_signal == 0x02)
		{
					lin_RrCycleMode_task(&LIN_RC_EVNT_MASTER_CMD);
		}

		if(Hu_Rr_Dr_Vent_signal == 0x02)
		{
				lin_RrVentMode_task(&LIN_RC_EVNT_MASTER_CMD);
		}
	}

	if((Hu_Rr_Dr_Vent_Mode == FACING_MODE) || (Hu_Rr_Ps_Vent_Mode == FACING_MODE))
		{
			if(Hu_Rr_Dr_Vent_Mode == FACING_MODE)
			{
				lin_RrVentMode_task(&LIN_RC_EVNT_MASTER_CMD);
			}

			if(Hu_Rr_Ps_Vent_Mode == FACING_MODE)
			{
				lin_RrVentMode_task(&LIN_RC_EVNT_MASTER_CMD);
			}
		}

		if((Hu_Rr_Dr_Vent_Mode == AVOID_MODE) || (Hu_Rr_Ps_Vent_Mode == AVOID_MODE))
		{
			if(Hu_Rr_Dr_Vent_Mode == AVOID_MODE)
			{
				lin_RrVentMode_task(&LIN_RC_EVNT_MASTER_CMD);
			}

			if(Hu_Rr_Ps_Vent_Mode == AVOID_MODE)
			{
				lin_RrVentMode_task(&LIN_RC_EVNT_MASTER_CMD);
			}

		}

		if((Hu_Rr_Dr_Vent_Mode == SWING_MODE) || (Hu_Rr_Ps_Vent_Mode == SWING_MODE))
		{
			if(Hu_Rr_Dr_Vent_Mode == SWING_MODE)
			{
				lin_RrCycleMode_task(&LIN_RC_EVNT_MASTER_CMD);
			}

			if(Hu_Rr_Ps_Vent_Mode == SWING_MODE)
			{
				lin_RrCycleMode_task(&LIN_RC_EVNT_MASTER_CMD);
			}
		}
}
#endif

//CTS
#ifdef AMO_GN7_PE_SETTING_NONE
void Cts_Front_mode_setting()
{
	if((Cts_Fr_dr_Vent_Side_siganl == 0x02) || (Cts_Fr_dr_Vent_Center_signal == 0x02) || (Cts_Fr_Ps_Vent_Side_signal == 0x02) || (Cts_Fr_Ps_Vent_Center_signal == 0x02))  //Full close mode
	{
		if(Cts_Fr_dr_Vent_Side_siganl == 0x02)
		{
			if(Cts_Fr_Dr_Vent_Mode == 0x00)
			{

			}
		}

		if(Cts_Fr_dr_Vent_Center_signal == 0x02)
		{
			if(Cts_Fr_Dr_Vent_Mode == 0x00)
			{

			}

		}

		if(Cts_Fr_Ps_Vent_Side_signal == 0x02)
		{
			if(Cts_Fr_Ps_Vent_Mode == 0x00)
			{

			}
		}

		if(Cts_Fr_Ps_Vent_Center_signal == 0x02)
		{
			if(Cts_Fr_Ps_Vent_Mode == 0x00)
			{

			}
		}

	}


	if((Cts_Fr_Dr_Vent_Mode == FACING_MODE) || (Cts_Fr_Ps_Vent_Mode == FACING_MODE))
		{
			if(Cts_Fr_Dr_Vent_Mode == FACING_MODE)
			{

			}

			if(Cts_Fr_Ps_Vent_Mode == FACING_MODE)
			{

			}

		}

		if((Cts_Fr_Dr_Vent_Mode == AVOID_MODE) || (Cts_Fr_Ps_Vent_Mode == AVOID_MODE))
		{
			if(Cts_Fr_Dr_Vent_Mode == AVOID_MODE)
			{

			}

			if(Cts_Fr_Ps_Vent_Mode == AVOID_MODE)
			{

			}

		}

		if((Cts_Fr_Dr_Vent_Mode == SWING_MODE) || (Cts_Fr_Ps_Vent_Mode == SWING_MODE))
		{
			if(Cts_Fr_Dr_Vent_Mode == SWING_MODE)
			{

			}

			if(Cts_Fr_Ps_Vent_Mode == SWING_MODE)
			{

			}

		}
}
#endif

#ifdef AMO_GN7_PE_SETTING_NONE
void Cts_Rear_mode_setting()
{
	if((Cts_Rr_Dr_Vent_siganl == 0x02) || (Cts_Rr_Ps_Vent_siganl == 0x02)) //Full close mode
	{
		if(Cts_Rr_Dr_Vent_siganl == 0x02)
		{
			if(Cts_Rr_Dr_Vent_Mode == 0x00)
			{

			}
		}

		if(Cts_Rr_Ps_Vent_siganl == 0x02)
		{
			if(Cts_Rr_Ps_Vent_Mode == 0x00)
			{

			}
		}
	}

	if((Cts_Rr_Dr_Vent_Mode == FACING_MODE) || (Cts_Rr_Ps_Vent_Mode == FACING_MODE))
	{
		if(Cts_Rr_Dr_Vent_Mode == FACING_MODE)
		{

		}

		if(Cts_Rr_Ps_Vent_Mode == FACING_MODE)
		{

		}

	}

	if((Cts_Rr_Dr_Vent_Mode == AVOID_MODE) || (Cts_Rr_Ps_Vent_Mode == AVOID_MODE))
	{
		if(Cts_Rr_Dr_Vent_Mode == AVOID_MODE)
		{

		}

		if(Cts_Rr_Ps_Vent_Mode == AVOID_MODE)
		{

		}

	}

	if((Cts_Rr_Dr_Vent_Mode == SWING_MODE) || (Cts_Rr_Ps_Vent_Mode == SWING_MODE))
	{
		if(Cts_Rr_Dr_Vent_Mode == SWING_MODE)
		{

		}

		if(Cts_Rr_Ps_Vent_Mode == SWING_MODE)
		{

		}
	}
}
#endif

#ifdef AMO_NVM_SETTING
void Last_Mode_dr_fr_set_fullclose_func()
{
	if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02 || Hu_Fr_dr_Vent_Center_signal_backup == 0x02 || Hu_Fr_Ps_Vent_Side_signal_backup == 0x02 || Hu_Fr_Ps_Vent_Center_signal_backup == 0x02)
	{
		if(Hu_Fr_dr_Vent_Side_signal_backup == 0x02)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x02;
		}

		if(Hu_Fr_dr_Vent_Center_signal_backup == 0x02)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x02;
		}

		if(Hu_Fr_Ps_Vent_Side_signal_backup == 0x02)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x02;
		}

		if(Hu_Fr_Ps_Vent_Center_signal_backup == 0x02)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x02;
		}
		
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
	#ifdef AMO_GN7_PE_SETTING_NONE
		HU_Rear_mode_setting();
	#endif
	}
}

void Last_Mode_dr_fr_set_func()
{
	if(((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_dr_Vent_Side_signal_backup != 0x02)) ||((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_dr_Vent_Center_signal_backup != 0x02)))
	{
		if(ATCU_DriverSideType == LHD)
		{
			lin_FrDrVent_Focus_Mode_task_Backup(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
		}
		else if(ATCU_DriverSideType == RHD)
		{
			RHD_lin_FrPsVent_Focus_Mode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
		}
		else
		{
		}
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
	if(((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_Fr_Ps_Vent_Side_signal_backup != 0x02)) ||((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_Fr_Ps_Vent_Center_signal_backup != 0x02)))
	{
		if(ATCU_DriverSideType == LHD)
		{
			lin_FrPsVent_Focus_Mode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
		}
		else if(ATCU_DriverSideType == RHD)
		{
			RHD_lin_FrDrVent_Focus_Mode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);
		}
		else
		{
		}
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
	
	if(((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_dr_Vent_Side_signal_backup != 0x02)) ||((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_dr_Vent_Center_signal_backup != 0x02)))
	{
		if(ATCU_DriverSideType == LHD)
		{
			lin_FrDrVent_Spread_Mode_task_Backup(&LIN_LH_EVNT_MASTER_CMD, &LIN_EVNT_PT);
		}
		else if(ATCU_DriverSideType == RHD)
		{
			RHD_lin_FrPsVent_Spread_Mode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
		}
		else
		{
		}
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

	if(((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Side_signal_backup != 0x02)) ||((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Center_signal_backup != 0x02)))
	{
		if(ATCU_DriverSideType == LHD)
		{
			lin_FrPsVent_Spread_Mode_task_Backup(&LIN_RH_EVNT_MASTER_CMD);
		}
		else if(ATCU_DriverSideType == RHD)
		{
			RHD_lin_FrDrVent_Spread_Mode_task_Backup(&LIN_LH_EVNT_MASTER_CMD);
		}
		else
		{
		}
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

	if((Hu_Fr_Dr_Vent_Mode == SWING_MODE) && (Hu_Fr_Ps_Vent_Mode == SWING_MODE) && (Hu_Fr_dr_Vent_Side_signal_backup != 0x02) && (Hu_Fr_dr_Vent_Center_signal_backup != 0x02) && (Hu_Fr_Ps_Vent_Side_signal_backup != 0x02) && (Hu_Fr_Ps_Vent_Center_signal_backup != 0x02))
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
	else if((Hu_Fr_Dr_Vent_Mode == SWING_MODE)&&(Hu_Fr_dr_Vent_Side_signal_backup != 0x02) &&(Hu_Fr_dr_Vent_Side_signal_backup != 0x02))
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
	else if((Hu_Fr_Ps_Vent_Mode == SWING_MODE) && (Hu_Fr_Ps_Vent_Side_signal_backup != 0x02) && (Hu_Fr_Ps_Vent_Side_signal_backup != 0x02))
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
		if(((Hu_Fr_Dr_Vent_Mode == FREE_MODE)&&(Hu_Fr_dr_Vent_Side_signal_backup != 0x02)) || ((Hu_Fr_Dr_Vent_Mode == FREE_MODE)&&(Hu_Fr_dr_Vent_Side_signal_backup != 0x02)))
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;

//			lin_FrDrCycleMode_TimerStop();
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

		if(((Hu_Fr_Ps_Vent_Mode == FREE_MODE)&&(Hu_Fr_Ps_Vent_Side_signal_backup != 0x02)) || ((Hu_Fr_Ps_Vent_Mode == FREE_MODE)&&(Hu_Fr_Ps_Vent_Side_signal_backup != 0x02)))
		{                     
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x04;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;

//			lin_FrPsCycleMode_TimerStop();
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
	}
#endif
}

#ifdef AMO_GN7_PE_SETTING_NONE
void Last_Mode_rear_dr_ps_set_func(void)
{
	if((Hu_Rr_Dr_Vent_signal == 0x02) || (Hu_Rr_Ps_Vent_signal == 0x02))
	{
		if(Hu_Rr_Dr_Vent_signal == 0x02)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0x00; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x02;
		}

		if(Hu_Rr_Ps_Vent_signal == 0x02)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x00; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x02;
		}
		Evt_queue_add(DEVICE_P_OIP_FULL_CLOSE_CAN_LIN_EVENT);
	}

	if(Hu_Rr_Dr_Vent_signal == 0x01)
	{
		if(Hu_Rr_Dr_Vent_Mode == 0x00)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = Cts_Rr_Dr_Vent_Mode_remember; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x01;
			Hu_Rr_Dr_Vent_Mode = Hu_Rr_Dr_Vent_Mode_Remember;
		}
	}

	if(Hu_Rr_Ps_Vent_signal == 0x01)
	{
		if(Hu_Rr_Ps_Vent_Mode == 0x00)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = Cts_Rr_Ps_Vent_Mode_remember; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x01;
			Hu_Rr_Ps_Vent_Mode = Hu_Rr_Ps_Vent_Mode_Remember;
		}
	}

	if((Hu_Rr_Dr_Vent_Mode == FACING_MODE) || (Hu_Rr_Ps_Vent_Mode == FACING_MODE))
	{
		if(Hu_Rr_Dr_Vent_Mode == FACING_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0x01;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
			Hu_Rr_Dr_Vent_Mode_Remember = 0x01;
		}

		if(Hu_Rr_Ps_Vent_Mode == FACING_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x01;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
			Hu_Rr_Ps_Vent_Mode_Remember = 0x01;
		}
		Evt_queue_add(DEVICE_P_OIP_FOCUS_CAN_LIN_EVENT);
	}

	if((Hu_Rr_Dr_Vent_Mode == AVOID_MODE) || (Hu_Rr_Ps_Vent_Mode == AVOID_MODE))
	{
		if(Hu_Rr_Dr_Vent_Mode == AVOID_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0x02;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
			Hu_Rr_Dr_Vent_Mode_Remember = 0x02;
		}

		if(Hu_Rr_Ps_Vent_Mode == AVOID_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x02;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
			Hu_Rr_Ps_Vent_Mode_Remember = 0x02;
		}
		Evt_queue_add(DEVICE_P_OIP_SPREAD_CAN_LIN_EVENT);
	}

	if((Hu_Rr_Dr_Vent_Mode == SWING_MODE) || (Hu_Rr_Ps_Vent_Mode == SWING_MODE))
	{
		if(Hu_Rr_Dr_Vent_Mode == SWING_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0x03;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
			Hu_Rr_Dr_Vent_Mode_Remember = 0x03;
		}

		if(Hu_Rr_Ps_Vent_Mode == SWING_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x03;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
			Hu_Rr_Ps_Vent_Mode_Remember = 0x03;
		}
		Evt_queue_add(DEVICE_P_OIP_CYCLE_CAN_LIN_EVENT);
	}

	if((Hu_Rr_Dr_Vent_Mode == FREE_MODE) || (Hu_Rr_Ps_Vent_Mode == FREE_MODE))
	{
		if(Hu_Rr_Dr_Vent_Mode == FREE_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0x04;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
			Hu_Rr_Dr_Vent_Mode_Remember = 0x04;
		}

		if(Hu_Rr_Ps_Vent_Mode == FREE_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x04;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
			Hu_Rr_Ps_Vent_Mode_Remember = 0x04;
		}
		Evt_queue_add(DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT);
		//P_Oip_Cts_priority = 0; //20241202_imsi test
	}

	if(((Hu_Rr_Dr_Vent_Mode_Remember == 0x00) && (Hu_Rr_Dr_Vent_signal != 0x02)) || ((Hu_Rr_Ps_Vent_Mode_Remember == 0x00) && (Hu_Rr_Ps_Vent_signal != 0x02)))
	{
		P_Oip_Cts_priority = 0;
	}

	if(Hu_Fr_Dr_Vent_Mode == MODE_OFF)
	{
		Hu_Rr_Dr_Vent_Mode_Remember = MODE_OFF;
		P_Oip_Cts_priority = 0;
		//VENT MODE OFF
	}
	if(Hu_Fr_Ps_Vent_Mode == MODE_OFF)
	{
		Hu_Rr_Ps_Vent_Mode_Remember = MODE_OFF;
		P_Oip_Cts_priority = 0;
		//VENT MODE OFF
	}
}
#endif

#endif

