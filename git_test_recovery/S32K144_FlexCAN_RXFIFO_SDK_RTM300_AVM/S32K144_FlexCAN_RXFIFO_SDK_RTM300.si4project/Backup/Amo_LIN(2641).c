#define Amo_LIN_C_

#include "Cpu.h"
#include "Amo_main.h"
#include "flexcan_driver.h"
#include "Amo_STATE_Event.h"
#include "Amo_Timer.h"
//#include "Amo_CAN_Parsing.h"

extern uint8_t sleepFlag;

extern flexcan_msgbuff_t recvBuff1, recvBuff2;

extern uint16_t Hu_Fr_Dr_Vent_Side_Bdry_Up_X;
extern uint16_t Hu_Fr_Dr_Vent_Side_Bdry_Lo_X;
extern uint16_t Hu_FrDrEVntSidePt_X;
extern uint16_t Hu_FrDrEVntCtrPt_X;

extern Lvnt_1_data Can_Tx_Evnt_1;
extern Lvnt_2_data Can_Tx_Evnt_2;
extern Lvnt_3_data Can_Tx_Evnt_3;
extern Lvnt_4_data Can_Tx_Evnt_4;
extern Lvnt_5_data Can_Tx_Evnt_5;
extern Lvnt_6_data Can_Tx_Evnt_6;
extern Lvnt_7_data Can_Tx_Evnt_7;
extern Lvnt_8_data Can_Tx_Evnt_8;
extern Lvnt_9_data Can_Tx_Evnt_9;
//extern uint16_t selectEVnt;

extern lin_LH_EVNT_MASTER_CMD_11 LIN_LH_EVNT_MASTER_CMD;


CanLin_oldNewValue_check_t CanValue =
{
	{	
		.Hu_FrDrEVntSidePt_X_oldVal = 150, //initial value
		.Hu_FrDrEVntSidePt_Y_oldVal = 180,
		.Hu_FrDrEVntCtrPt_X_oldVal = 150,
		.Hu_FrDrEVntCtrPt_Y_oldVal = 180
	},
	{
		.Hu_FrPsEVntSidePt_X_oldVal = 150,
		.Hu_FrPsEVntSidePt_Y_oldVal = 180,
		.Hu_FrPsEVntCtrPt_X_oldVal = 150,
		.Hu_FrPsEVntCtrPt_Y_oldVal = 180
	},
	{
		.Hu_RrDrEVntConsPt_X_oldVal = 150,
		.Hu_RrDrEVntConsPt_Y_oldVal = 180,
		.Hu_RrPsEVntConsPt_X_oldVal = 150,
		.Hu_RrPsEVntConsPt_Y_oldVal = 180
	}
};

SelectEVnt_t CurrentEVnt = 
{
	.selectEVnt = DefaultPt,
	.selectEVnt1 = DefaultPt
};


void Lin_Init(void)
{
	/* Initialize LIN network interface */
	l_sys_init();

	l_ifc_init(LI0);
	l_ifc_init(LI1);
	l_ifc_init(LI2);

	/* Set Schedule table to Normal */
	//    l_sch_set(LI0, LI0_Normal, 0u);
	l_sch_set(LI0, LI0_SCHEDULER_EVNT_NORMAL_FR_DRV, 0u);
	//		l_sch_set(LI0, LI0_SCHEDULER_EVNT_SPECIAL_CMD_FR_DRV, 0u);

	//    l_sch_set(LI1, LI1_Normal, 0u);
	l_sch_set(LI1, LI1_SCHEDULER_EVNT_NORMAL_FR_PASS, 0u);
	//		l_sch_set(LI1, LI1_SCHEDULER_EVNT_SPECIAL_CMD_FR_PASS, 0u);

	//    l_sch_set(LI2, LI2_Normal, 0u);
	l_sch_set(LI2, LI2_SCHEDULER_EVNT_NORMAL_RR_CTR, 0u);
	//		l_sch_set(LI2, LI2_SCHEDULER_EVNT_SPECIAL_CMD_RR_CTR, 0u);

	//    l_sch_set(LI0, LI0_SCHEDULER_EVNT_NORMAL_FR_DRV, 0u);
	//    l_sch_set(LI0, LI0_SCHEDULER_EVNT_SPECIAL_CMD_FR_DRV, 0u);

	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD);
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD);
	SetLIN_EVNT_RrSPEED(&LIN_RC_EVNT_MASTER_CMD);

}

void lin_sleep_task(void)
{

//	lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 20;
	if(sleepFlag == 1)
	{
		l_sch_set(LI0, LI0_GOTO_SLEEP_SCHEDULE, 0u);
		sleepFlag = 0;
	}
	if(sleepFlag == 2)
	{
		l_ifc_wake_up(LI0);
		sleepFlag = 0;
	}
	 /* Check node state */
	if (LIN_NODE_STATE_SLEEP_MODE == lin_lld_get_state(LI0))
	{
		PINS_DRV_SetPins(GPIO_PORTA, TEST_OUT1_MASK);
		/* Turn off all LEDs */
	}
	if (LIN_NODE_STATE_IDLE == lin_lld_get_state(LI0))
	{
		PINS_DRV_ClearPins(GPIO_PORTA, TEST_OUT1_MASK);
		/* Turn off all LEDs */
	}
}

void Set_LinToCan_L_EVNT_HU_FrDrEvntPtDisp_XY(Lvnt_1_data *L1_DataBuf, lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_X = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
}

void Set_LinToCan_L_EVNT_HU_FrPsEvntPtDisp_XY(Lvnt_2_data *L2_DataBuf, lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_X = lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
}


void lin_master_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{

//	lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 20;

	if (l_flg_tst_LI0_FR_SLLR_ActuatorState_flag())
	{
	    //Clear this flag...
		l_flg_clr_LI0_FR_SLLR_ActuatorState_flag();
	}

	 /* Check node state */
	if (LIN_NODE_STATE_SLEEP_MODE == lin_lld_get_state(LI0))
	{
		/* Turn off all LEDs */
	}
}

void lin_FrDrmanualMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	lin_Ctrl->LDATA.EVNT_SPEED = 0xA;
	
	l_u8_wr_LI0_EVNT_SPEED(lin_Ctrl->LDATA.EVNT_SPEED);
	if(l_flg_tst_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition_flag() || l_flg_tst_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition_flag())
	{
	    //Clear this flag...
	  l_flg_clr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition_flag();
		l_flg_clr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition_flag();
	
		l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);
		l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);
		l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);
		l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);
	}

	Set_LinToCan_L_EVNT_HU_FrDrEvntPtDisp_XY(&Can_Tx_Evnt_1, &LIN_LH_EVNT_MASTER_CMD);

}

void lin_FrPsmanualMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
  lin_Ctrl->LDATA.EVNT_SPEED = 0xA;
	
	l_u8_wr_LI1_EVNT_SPEED(lin_Ctrl->LDATA.EVNT_SPEED);
	if(l_flg_tst_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition_flag() || l_flg_tst_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition_flag())
	{
	    //Clear this flag...
	  l_flg_clr_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition_flag();
		l_flg_clr_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition_flag();
			
		l_u16_wr_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition);
		l_u16_wr_LI1_EVNT_Front_Center_RH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition);
		l_u16_wr_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition);
		l_u16_wr_LI1_EVNT_Front_Side_RH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition);
	}

	Set_LinToCan_L_EVNT_HU_FrPsEvntPtDisp_XY(&Can_Tx_Evnt_2, &LIN_RH_EVNT_MASTER_CMD);

}

void lin_RrmanualMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
  lin_Ctrl->LDATA.EVNT_REAR_LH_SPEED = 0xA;
	lin_Ctrl->LDATA.EVNT_REAR_RH_SPEED = 0xA;
	
	l_u8_wr_LI2_EVNT_REAR_LH_SPEED(lin_Ctrl->LDATA.EVNT_REAR_LH_SPEED);
	l_u8_wr_LI2_EVNT_REAR_RH_SPEED(lin_Ctrl->LDATA.EVNT_REAR_RH_SPEED);
	if(l_flg_tst_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition_flag() || l_flg_tst_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition_flag())
	{
	    //Clear this flag...
	  l_flg_clr_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition_flag();
		l_flg_clr_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition_flag();
			
		l_u16_wr_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition);
		l_u16_wr_LI2_EVNT_Rear_Center_RH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition);
		l_u16_wr_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition);
		l_u16_wr_LI2_EVNT_Rear_Center_LH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition);
	}

}


void lin_FrDrVentMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
  lin_Ctrl->LDATA.EVNT_SPEED = 0xA;
	l_u8_wr_LI0_EVNT_SPEED(lin_Ctrl->LDATA.EVNT_SPEED);

	if(l_flg_tst_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition_flag() || l_flg_tst_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition_flag())
	{
		//Clear this flag...
	  l_flg_clr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition_flag();
		l_flg_clr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition_flag();
	
		if((Hu_Fr_Dr_Vent_Mode == DEFAULT_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x02)) //fullclose
		{
			Amo_timer_Stop(timer_16);
			Amo_timer_Stop(timer_17);
			Amo_timer_Stop(timer_18);
			Amo_timer_Stop(timer_19);
			Amo_timer_Stop(timer_20);
			Amo_timer_Stop(timer_21);
			lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 1256;
			lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 1023;
		}
		
		if((Hu_Fr_Dr_Vent_Mode == DEFAULT_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x02)) //fullclose
		{
			Amo_timer_Stop(timer_16);
			Amo_timer_Stop(timer_17);
			Amo_timer_Stop(timer_18);
			Amo_timer_Stop(timer_19);
			Amo_timer_Stop(timer_20);
			Amo_timer_Stop(timer_21);
			lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 792;
			lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 1023;
		}

		if((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01)) //spread mode
		{
			Amo_timer_Stop(timer_16);
			Amo_timer_Stop(timer_17);
			Amo_timer_Stop(timer_18);
			Amo_timer_Stop(timer_19);
			Amo_timer_Stop(timer_20);
			Amo_timer_Stop(timer_21);
			lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 1898;
			lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 1788;
		}
		
		if((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01)) //spread mode
		{
			Amo_timer_Stop(timer_16);
			Amo_timer_Stop(timer_17);
			Amo_timer_Stop(timer_18);
			Amo_timer_Stop(timer_19);
			Amo_timer_Stop(timer_20);
			Amo_timer_Stop(timer_21);
			lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 150;
			lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 1788;
		}

		if((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01)) //focus mode
		{
			Amo_timer_Stop(timer_16);
			Amo_timer_Stop(timer_17);
			Amo_timer_Stop(timer_18);
			Amo_timer_Stop(timer_19);
			Amo_timer_Stop(timer_20);
			Amo_timer_Stop(timer_21);
			lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 150;
			lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
		}
		
		if(Hu_Fr_Dr_Vent_Mode == FACING_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01) //focus mode
		{
			Amo_timer_Stop(timer_16);
			Amo_timer_Stop(timer_17);
			Amo_timer_Stop(timer_18);
			Amo_timer_Stop(timer_19);
			Amo_timer_Stop(timer_20);
			Amo_timer_Stop(timer_21);
			lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 1898;
			lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;
		}

		l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);
		l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);
		l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);
		l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);

	}
}


void lin_FrPsVentMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
  lin_Ctrl->LDATA.EVNT_SPEED = 0xA;
	l_u8_wr_LI1_EVNT_SPEED(lin_Ctrl->LDATA.EVNT_SPEED);

	if(l_flg_tst_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition_flag() || l_flg_tst_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition_flag())
	{
		//Clear this flag...
	  l_flg_clr_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition_flag();
		l_flg_clr_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition_flag();
	
		if((Hu_Fr_Ps_Vent_Mode == DEFAULT_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x02)) //fullclose
		{
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = 791;
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 1023;
		}
		
		if((Hu_Fr_Ps_Vent_Mode == DEFAULT_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x02)) //fullclose
		{
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = 1256;
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 1023;
		}

		if((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01)) //spread mode
		{
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = 150;
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 1788;
		}
		
		if((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01)) //spread mode
		{
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = 1898;
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 1788;
		}

		if((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01)) //focus mode
		{
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = 1898;
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;
		}
		
		if(Hu_Fr_Ps_Vent_Mode == FACING_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01) //focus mode
		{
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = 150;
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;
		}
		
		l_u16_wr_LI1_EVNT_Front_Side_RH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition);
		l_u16_wr_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition);
		l_u16_wr_LI1_EVNT_Front_Center_RH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition);
		l_u16_wr_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition);
	}
}


void lin_RrVentMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
  lin_Ctrl->LDATA.EVNT_REAR_LH_SPEED = 0xA;
	lin_Ctrl->LDATA.EVNT_REAR_RH_SPEED = 0xA;
	
	l_u8_wr_LI2_EVNT_REAR_LH_SPEED(lin_Ctrl->LDATA.EVNT_REAR_LH_SPEED);
	l_u8_wr_LI2_EVNT_REAR_RH_SPEED(lin_Ctrl->LDATA.EVNT_REAR_RH_SPEED);

	if(l_flg_tst_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition_flag() || l_flg_tst_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition_flag())
	{
		//Clear this flag...
	  l_flg_clr_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition_flag();
		l_flg_clr_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition_flag();
	
		if((Hu_Rr_Dr_Vent_Mode == DEFAULT_MODE) && (Hu_Rr_Dr_Vent_signal == 0x02)) //fullclose
		{
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = 1256;
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = 1023;
		}
		
		if((Hu_Rr_Ps_Vent_Mode == DEFAULT_MODE) && (Hu_Rr_Ps_Vent_signal == 0x02)) //fullclose
		{
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = 792;
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = 1023;
		}

		if((Hu_Rr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Rr_Dr_Vent_signal == 0x01)) //spread mode
		{
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = 1898;
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = 1788;
		}
		
		if((Hu_Rr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Rr_Ps_Vent_signal == 0x01)) //spread mode
		{
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = 150;
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = 1788;
		}

		if((Hu_Rr_Dr_Vent_Mode == FACING_MODE) && (Hu_Rr_Dr_Vent_signal == 0x01)) //focus mode
		{
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = 150;
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = 0;
		}
		
		if(Hu_Rr_Ps_Vent_Mode == FACING_MODE && Hu_Rr_Ps_Vent_signal == 0x01) //focus mode
		{
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = 1898;
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = 0;
		}
		
		l_u16_wr_LI2_EVNT_Rear_Center_LH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition);
		l_u16_wr_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition);
		l_u16_wr_LI2_EVNT_Rear_Center_RH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition);
		l_u16_wr_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition);		
	}
}



void lin_CycleMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	lin_SwingData_Sinario0();
}

void lin_SwingData_Sinario0()
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;

	l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);
	

	Amo_timer_Stop(timer_16);
	Amo_timer_Start(timer_16, 1199, false, lin_SwingData_Sinario1);
}

void lin_SwingData_Sinario1()
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_SPEED = 0x3;

	l_u8_wr_LI0_EVNT_SPEED(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_SPEED);

	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 1748;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 300;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;
	
	l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);	

	Amo_timer_Stop(timer_17);
	Amo_timer_Start(timer_17, 1701, false, lin_SwingData_Sinario2);

}

void lin_SwingData_Sinario2()
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 1748;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 260;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 300;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 260;

	l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);

	Amo_timer_Stop(timer_18);
	Amo_timer_Start(timer_18, 2502, false, lin_SwingData_Sinario3);
}

void lin_SwingData_Sinario3()
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 300;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 260;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 1748;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 260;

	l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);

	Amo_timer_Stop(timer_19);
	Amo_timer_Start(timer_19, 1701, false, lin_SwingData_Sinario4);
}

void lin_SwingData_Sinario4()
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 300;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 1788;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 1748;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 1788;

	l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);

	Amo_timer_Stop(timer_20);
	Amo_timer_Start(timer_20, 2502, false, lin_SwingData_Sinario5);
}

void lin_SwingData_Sinario5()
{
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 1748;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 1788;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 300;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 1788;

	l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);
	l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);

	Amo_timer_Stop(timer_21);
	Amo_timer_Start(timer_21, 1701, false, lin_SwingData_Sinario1);
}





void SetLIN_EVNT_FrDrSPEED(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{

	lin_Ctrl->LDATA.EVNT_SPEED = 0xA;
	
	l_u8_wr_LI0_EVNT_SPEED(lin_Ctrl->LDATA.EVNT_SPEED);
}

void SetLIN_EVNT_FrPsSPEED(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{

	lin_Ctrl->LDATA.EVNT_SPEED = 0xA;
	
	l_u8_wr_LI1_EVNT_SPEED(lin_Ctrl->LDATA.EVNT_SPEED);
}

void SetLIN_EVNT_RrSPEED(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{

	lin_Ctrl->LDATA.EVNT_REAR_LH_SPEED = 0xA;
	lin_Ctrl->LDATA.EVNT_REAR_RH_SPEED = 0xA;
	
	l_u8_wr_LI2_EVNT_REAR_LH_SPEED(lin_Ctrl->LDATA.EVNT_REAR_LH_SPEED);
	l_u8_wr_LI2_EVNT_REAR_RH_SPEED(lin_Ctrl->LDATA.EVNT_REAR_RH_SPEED);
}


void CanData_Update_Check_FrDrPt(Atcu_20_data *canPtDataBuf)
{
	CanValue.FrDrPt.Hu_FrDrEVntSidePt_X_newVal = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_X;
	CanValue.FrDrPt.Hu_FrDrEVntSidePt_Y_newVal = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_Y;
	CanValue.FrDrPt.Hu_FrDrEVntCtrPt_X_newVal = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_X;
	CanValue.FrDrPt.Hu_FrDrEVntCtrPt_Y_newVal = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_Y;


	if((CanValue.FrDrPt.Hu_FrDrEVntSidePt_X_oldVal != CanValue.FrDrPt.Hu_FrDrEVntSidePt_X_newVal) || \
		(CanValue.FrDrPt.Hu_FrDrEVntSidePt_Y_oldVal != CanValue.FrDrPt.Hu_FrDrEVntSidePt_Y_newVal))
	{
		CurrentEVnt.selectEVnt = FrDrSidePt_XY;
		
		CanValue.FrDrPt.Hu_FrDrEVntSidePt_X_oldVal = CanValue.FrDrPt.Hu_FrDrEVntSidePt_X_newVal;
		CanValue.FrDrPt.Hu_FrDrEVntSidePt_Y_oldVal = CanValue.FrDrPt.Hu_FrDrEVntSidePt_Y_newVal;

	}

	if((CanValue.FrDrPt.Hu_FrDrEVntCtrPt_X_oldVal != CanValue.FrDrPt.Hu_FrDrEVntCtrPt_X_newVal) || \
		(CanValue.FrDrPt.Hu_FrDrEVntCtrPt_Y_oldVal != CanValue.FrDrPt.Hu_FrDrEVntCtrPt_Y_newVal))
	{
		CurrentEVnt.selectEVnt1 = FrDrCtrPt_XY;
		
		CanValue.FrDrPt.Hu_FrDrEVntCtrPt_X_oldVal = CanValue.FrDrPt.Hu_FrDrEVntCtrPt_X_newVal;
		CanValue.FrDrPt.Hu_FrDrEVntCtrPt_Y_oldVal = CanValue.FrDrPt.Hu_FrDrEVntCtrPt_Y_newVal;
	}
}


void CanData_Update_Check_FrPsPt(Atcu_21_data *canPtDataBuf)
{
	CanValue.FrPsPt.Hu_FrPsEVntSidePt_X_newVal = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_X;
	CanValue.FrPsPt.Hu_FrPsEVntSidePt_Y_newVal = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_Y;
	CanValue.FrPsPt.Hu_FrPsEVntCtrPt_X_newVal = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_X;
	CanValue.FrPsPt.Hu_FrPsEVntCtrPt_Y_newVal = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_Y;


	if((CanValue.FrPsPt.Hu_FrPsEVntSidePt_X_oldVal != CanValue.FrPsPt.Hu_FrPsEVntSidePt_X_newVal) || \
		(CanValue.FrPsPt.Hu_FrPsEVntSidePt_Y_oldVal != CanValue.FrPsPt.Hu_FrPsEVntSidePt_Y_newVal))
	{
		CurrentEVnt.selectEVnt = FrPsSidePt_XY;
		
		CanValue.FrPsPt.Hu_FrPsEVntSidePt_X_oldVal = CanValue.FrPsPt.Hu_FrPsEVntSidePt_X_newVal;
		CanValue.FrPsPt.Hu_FrPsEVntSidePt_Y_oldVal = CanValue.FrPsPt.Hu_FrPsEVntSidePt_Y_newVal;

	}

	if((CanValue.FrPsPt.Hu_FrPsEVntCtrPt_X_oldVal != CanValue.FrPsPt.Hu_FrPsEVntCtrPt_X_newVal) || \
		(CanValue.FrPsPt.Hu_FrPsEVntCtrPt_Y_oldVal != CanValue.FrPsPt.Hu_FrPsEVntCtrPt_Y_newVal))
	{
		CurrentEVnt.selectEVnt1 = FrPsCtrPt_XY;
		
		CanValue.FrPsPt.Hu_FrPsEVntCtrPt_X_oldVal = CanValue.FrPsPt.Hu_FrPsEVntCtrPt_X_newVal;
		CanValue.FrPsPt.Hu_FrPsEVntCtrPt_Y_oldVal = CanValue.FrPsPt.Hu_FrPsEVntCtrPt_Y_newVal;
	}
}


void CanData_Update_Check_RrPt(Atcu_22_data *canPtDataBuf)
{
	CanValue.RrPt.Hu_RrDrEVntConsPt_X_newVal = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_X;
	CanValue.RrPt.Hu_RrDrEVntConsPt_Y_newVal = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_Y;
	CanValue.RrPt.Hu_RrPsEVntConsPt_X_newVal = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_X;
	CanValue.RrPt.Hu_RrPsEVntConsPt_Y_newVal = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_Y;


	if((CanValue.RrPt.Hu_RrDrEVntConsPt_X_oldVal != CanValue.RrPt.Hu_RrDrEVntConsPt_X_newVal) || \
		(CanValue.RrPt.Hu_RrDrEVntConsPt_Y_oldVal != CanValue.RrPt.Hu_RrDrEVntConsPt_Y_newVal))
	{
		CurrentEVnt.selectEVnt = RrDrCons_XY;
		
		CanValue.RrPt.Hu_RrDrEVntConsPt_X_oldVal = CanValue.RrPt.Hu_RrDrEVntConsPt_X_newVal;
		CanValue.RrPt.Hu_RrDrEVntConsPt_Y_oldVal = CanValue.RrPt.Hu_RrDrEVntConsPt_Y_newVal;

	}

	if((CanValue.RrPt.Hu_RrPsEVntConsPt_X_oldVal != CanValue.RrPt.Hu_RrPsEVntConsPt_X_newVal) || \
		(CanValue.RrPt.Hu_RrPsEVntConsPt_Y_oldVal != CanValue.RrPt.Hu_RrPsEVntConsPt_Y_newVal))
	{
		CurrentEVnt.selectEVnt1 = RrPsCons_XY;
		
		CanValue.RrPt.Hu_RrPsEVntConsPt_X_oldVal = CanValue.RrPt.Hu_RrPsEVntConsPt_X_newVal;
		CanValue.RrPt.Hu_RrPsEVntConsPt_Y_oldVal = CanValue.RrPt.Hu_RrPsEVntConsPt_Y_newVal;
	}
}



void FrDrLinData_Parsing(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
//	if(CurrentEVnt.selectEVnt == FrDrSidePt_XY)
//	{
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_X(&Can_data_20_Info, &Can_data_23_Info);
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_Y(&Can_data_20_Info, &Can_data_23_Info);

//		CurrentEVnt.selectEVnt = DefaultPt;
//	}
//	if(CurrentEVnt.selectEVnt1 == FrDrCtrPt_XY)
//	{
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_X(&Can_data_20_Info, &Can_data_23_Info);
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_Y(&Can_data_20_Info, &Can_data_23_Info);
	
//		CurrentEVnt.selectEVnt1 = DefaultPt;
//	}
}

void FrPsLinData_Parsing(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	if(CurrentEVnt.selectEVnt == FrPsSidePt_XY)
	{
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_X(&Can_data_21_Info, &Can_data_24_Info);
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_Y(&Can_data_21_Info, &Can_data_24_Info);

		CurrentEVnt.selectEVnt = DefaultPt;
	}
	if(CurrentEVnt.selectEVnt1 == FrPsCtrPt_XY)
	{
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_X(&Can_data_21_Info, &Can_data_24_Info);
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_Y(&Can_data_21_Info, &Can_data_24_Info);
	
		CurrentEVnt.selectEVnt1 = DefaultPt;
	}
}

void RrLinData_Parsing(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
	if(CurrentEVnt.selectEVnt == RrDrCons_XY)
	{
		lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_RrDrEVntConsPt_X(&Can_data_22_Info, &Can_data_25_Info);
		lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = SET_L_ATCU_HU_RrDrEVntConsPt_Y(&Can_data_22_Info, &Can_data_25_Info);

		CurrentEVnt.selectEVnt = DefaultPt;
	}
	if(CurrentEVnt.selectEVnt1 == RrPsCons_XY)
	{
		lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_RrPsEVntConsPt_X(&Can_data_22_Info, &Can_data_25_Info);
		lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = SET_L_ATCU_HU_RrPsEVntConsPt_Y(&Can_data_22_Info, &Can_data_25_Info);
	
		CurrentEVnt.selectEVnt1 = DefaultPt;
	}
}

uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_X(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrDrSidePt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_X * canFactor));
	CanLinValue[FrDrSidePt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_X;

	CanLinValue[FrDrSidePt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrDr_SidePt_X(Conv_Can_Pt_X);

	ret_value = CanLinValue[FrDrSidePt_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}

uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_X(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_X * canFactor));
	CanLinValue[FrDrCtrPt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_X;

	CanLinValue[FrDrCtrPt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrDr_CtrPt_X(Conv_Can_Pt_X);
	
	ret_value = CanLinValue[FrDrCtrPt_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_Y(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrDrSidePt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_Y * canFactor));
	CanLinValue[FrDrSidePt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_Y;

	CanLinValue[FrDrSidePt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrDr_SidePt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[FrDrSidePt_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_Y(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	
	CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y * canFactor));
	CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_Y;

	CanLinValue[FrDrCtrPt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrDr_CtrPt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[FrDrCtrPt_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_X(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrPsSidePt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_X * canFactor));
	CanLinValue[FrPsSidePt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_X;

	CanLinValue[FrPsSidePt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrPs_SidePt_X(Conv_Can_Pt_X);

	ret_value = CanLinValue[FrPsSidePt_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}

uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_X(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrPsCtrPt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_X * canFactor));
	CanLinValue[FrPsCtrPt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_X;

	CanLinValue[FrPsCtrPt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrPs_CtrPt_X(Conv_Can_Pt_X);
	
	ret_value = CanLinValue[FrPsCtrPt_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_Y(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrPsSidePt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_Y * canFactor));
	CanLinValue[FrPsSidePt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_Y;

	CanLinValue[FrPsSidePt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrPs_SidePt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[FrPsSidePt_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_Y(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	
	CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y * canFactor));
	CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_Y;

	CanLinValue[FrPsCtrPt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrPs_CtrPt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[FrPsCtrPt_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_X(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[RrDrCons_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_X * canFactor));
	CanLinValue[RrDrCons_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_X;

	CanLinValue[RrDrCons_X].Pt_X.LinData_Pt_X = Cal_CanToLin_RrDr_ConsPt_X(Conv_Can_Pt_X);

	ret_value = CanLinValue[RrDrCons_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}

uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_X(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[RrPsCons_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_X * canFactor));
	CanLinValue[RrPsCons_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_X;

	CanLinValue[RrPsCons_X].Pt_X.LinData_Pt_X = Cal_CanToLin_RrPs_ConsPt_X(Conv_Can_Pt_X);
	
	ret_value = CanLinValue[RrPsCons_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_Y(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[RrDrCons_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_Y * canFactor));
	CanLinValue[RrDrCons_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_Y;

	CanLinValue[RrDrCons_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_RrDr_ConsPt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[RrDrCons_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_Y(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	
	CanLinValue[RrPsCons_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_Y * canFactor));
	CanLinValue[RrPsCons_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_Y;

	CanLinValue[RrPsCons_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_RrPs_ConsPt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[RrPsCons_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}


uint16_t CONV_CanToLin_TargetPos_void(void)
{
	uint16_t ret_value = FALSE;

	int16_t LinPosRange = 600;
	int16_t CanPosRange = 0, CanPosition = 0, CanPosCenter, CanData = 0, linFactor = 0, LinData = 0;

	CanPosRange = (Hu_Fr_Dr_Vent_Side_Bdry_Up_X) - (Hu_Fr_Dr_Vent_Side_Bdry_Lo_X);

//	CanPosRange = 300;

	CanPosition = Hu_FrDrEVntSidePt_X;

	CanPosCenter = CanPosRange>>1;

	CanData = CanPosition - CanPosCenter;
	linFactor = (LinPosRange / CanPosRange);
	LinData = (CanData * linFactor) & 0x7FF;

	return ret_value = LinData;
}



