#define Amo_LIN_C_

#include "Cpu.h"
#include "Amo_main.h"
#include "flexcan_driver.h"
#include "Amo_STATE_Event.h"
#include "Amo_Timer.h"
#include "Amo_Calculate.h"
#include "Amo_CAN_Parsing.h"
#include "Amo_Mode_Setting.h"
#include "Amo_Cycle.h"

uint8_t FR_LH_Target_Pt_act_flag = 0;
uint8_t FR_RH_Target_Pt_act_flag = 0;

uint8_t lin_WakeupFlag = false;

volatile bool backup_Flag =FALSE;

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

lin_LH_EVNT_MASTER_CMD_11 LIN_LH_EVNT_MASTER_CMD;
lin_RH_EVNT_MASTER_CMD_21	LIN_RH_EVNT_MASTER_CMD;
#ifdef AMO_GN7_PE_SETTING_NONE		
lin_RC_EVNT_MASTER_CMD_31	LIN_RC_EVNT_MASTER_CMD;
#endif

extern int16_t Fr_Dr_side_leftright_target_backup;
extern int16_t Fr_Dr_side_updown_target_backup;
extern int16_t Fr_Dr_ctr_leftright_target_backup;
extern int16_t Fr_Dr_ctr_updown_target_backup;
extern int16_t Fr_Ps_side_leftright_target_backup;
extern int16_t Fr_Ps_side_updown_target_backup;
extern int16_t Fr_Ps_ctr_leftright_target_backup;
extern int16_t Fr_Ps_ctr_updown_target_backup;
#ifdef AMO_GN7_PE_SETTING_NONE
extern int16_t Rr_Dr_side_leftright_target_backup;
extern int16_t Rr_Dr_side_updown_targe_backup;
extern int16_t Rr_Ps_ctr_leftright_target_backup;
extern int16_t Rr_Ps_ctr_updown_target_backup;
#endif

extern uint16_t Hu_FrDrEVntSidePt_X_backup;
extern uint16_t Hu_FrDrEVntSidePt_Y_backup;
extern uint16_t Hu_FrDrEVntCtrPt_X_backup;
extern uint16_t Hu_FrDrEVntCtrPt_Y_backup;
extern uint16_t Hu_FrPsEVntSidePt_X_backup;
extern uint16_t Hu_FrPsEVntSidePt_Y_backup;
extern uint16_t Hu_FrPsEVntCtrPt_X_backup;
extern uint16_t Hu_FrPsEVntCtrPt_Y_backup;
#ifdef AMO_GN7_PE_SETTING_NONE		
extern uint16_t Hu_RrDrEVntConsPt_X_backup;
extern uint16_t Hu_RrDrEVntConsPt_Y_backup;
extern uint16_t Hu_RrPsEVntConsPt_X_backup;
extern uint16_t Hu_RrPsEVntConsPt_Y_backup;
#endif

extern uint8_t Hu_Frdr_Vent_Side_sig_backup;
extern uint8_t Hu_Frdr_Vent_Center_sig_backup;
extern uint8_t Hu_FrPs_Vent_Side_sig_backup;
extern uint8_t Hu_FrPs_Vent_Center_sig_backup;

extern uint16_t Hu_FrDrEVntSidePt_X_invalid;
extern uint16_t Hu_FrDrEVntSidePt_Y_invalid;
extern uint16_t Hu_FrDrEVntCtrPt_X_invalid;
extern uint16_t Hu_FrDrEVntCtrPt_Y_invalid;

extern uint16_t Hu_FrPsEVntSidePt_X_invalid;
extern uint16_t Hu_FrPsEVntSidePt_Y_invalid;
extern uint16_t Hu_FrPsEVntCtrPt_X_invalid;
extern uint16_t Hu_FrPsEVntCtrPt_Y_invalid;


extern uint8_t Evnt_COM_Onoff;

extern uint8_t lin_rxResponseFlag_sllr;
extern uint8_t lin_rxResponseFlag_slud;
extern uint8_t lin_rxResponseFlag_cllr;
extern uint8_t lin_rxResponseFlag_clud;
extern uint8_t lin_rxResponseFlag_crlr;
extern uint8_t lin_rxResponseFlag_crud;
extern uint8_t lin_rxResponseFlag_srlr;
extern uint8_t lin_rxResponseFlag_srud;

extern uint8_t Evnt_IGN2_Onoff;
extern uint8_t linbusOff_FltCheck_Start_Flag;

CanLinValue_t CanLinValue[20];

//l_ifc_handle L_IFC_HANDLE;

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
	#ifdef AMO_GN7_PE_SETTING_NONE		
	{
		.Hu_RrDrEVntConsPt_X_oldVal = 150,
		.Hu_RrDrEVntConsPt_Y_oldVal = 180,
		.Hu_RrPsEVntConsPt_X_oldVal = 150,
		.Hu_RrPsEVntConsPt_Y_oldVal = 180
	}
	#endif
};


EvntMode_flag_t evntModeflags = {FALSE};

Lin_EVntPt_t LIN_EVNT_PT;
FR_LinBusOff_flag_t FR_LINBUSOff_FLAG;
FR_DTC_Flt_flag_t	FR_DTC_Flt_flag;
lin_LH_EVNT_SPECIAL_CMD_0 LIN_LH_EVNT_SPECIAL_CMD;
lin_RH_EVNT_SPECIAL_CMD_0 LIN_RH_EVNT_SPECIAL_CMD;
#ifdef AMO_GN7_PE_SETTING_NONE		
lin_RC_EVNT_SPECIAL_CMD_0 LIN_RC_EVNT_SPECIAL_CMD;
#endif
t_lin_LH_FR_STATUS	LIN_LH_FR_STATUS;
t_lin_RH_FR_STATUS	LIN_RH_FR_STATUS;
#ifdef AMO_GN7_PE_SETTING_NONE		
t_lin_RC_RR_STATUS	LIN_RC_RR_STATUS;
#endif

#if (defined ACT_ANGLE_UPDATE || defined ACT_ANGLE_UPDATE_2 || defined ACT_ANGLE_UPDATE_3)
const uint16_t canFactor = 10U;
const uint16_t linRangePt_X = 700U; 
const uint16_t linRangePt_Y = 900U;

#elif defined(OLD_ANGLE)
const uint16_t canFactor = 10U;
const uint16_t linRangePt_X = 580U; //29dgree //600U 30dgree 
const uint16_t linRangePt_Y = 480U; //24dgree //500U 25dgree //520U 26dgree
#endif



void Lin_GoTo_Sleep(void)
{
	//lin_WakeupFlag = true;
	
	l_sch_set(LI0, LI0_GOTO_SLEEP_SCHEDULE, 0u);
	l_sch_set(LI1, LI1_GOTO_SLEEP_SCHEDULE, 0u);
	//l_sch_set(LI2, LI2_GOTO_SLEEP_SCHEDULE, 0u);
}

void Lin_GoTo_WakeUp(void)
{
//	Lin_Init();
	
//	if(lin_WakeupFlag == true)
//	{	
		l_ifc_wake_up(LI0);
		l_sch_set(LI0, LI0_SCHEDULER_EVNT_NORMAL_FR_DRV, 0u);
		l_ifc_wake_up(LI1);
		l_sch_set(LI1, LI1_SCHEDULER_EVNT_NORMAL_FR_PASS, 0u);

//		lin_WakeupFlag = false;
//	}
}

#ifdef LIN_BUSOFF_CHECK_DELAY_50MS
void Lin_FR_DR_BusOff_Check(FR_LinBusOff_flag_t *fault_flag)
{
	if(lin_rxResponseFlag_sllr == true)
	{
		lin_rxResponseFlag_sllr = false;
		
		if(l_flg_tst_LI0_FR_SLLR_STATE_flag())
		{
//			PINS_DRV_SetPins(GPIO_PORTC, TEST_OUT1_MASK);

			l_flg_clr_LI0_FR_SLLR_STATE_flag();
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag = false;

//			PINS_DRV_ClearPins(GPIO_PORTC, TEST_OUT1_MASK);
		}
		else 
		{ 
			if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
			{
				FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag = true;
			}
		}
	}

	if(lin_rxResponseFlag_slud == true)
	{
		lin_rxResponseFlag_slud = false;
		
		if(l_flg_tst_LI0_FR_SLUD_STATE_flag())
		{
			l_flg_clr_LI0_FR_SLUD_STATE_flag();
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag = false;
		}
		else 
		{ 
			if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
			{
				FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag = true;
			}
		}
	}

	if(lin_rxResponseFlag_cllr == true)
	{
		lin_rxResponseFlag_cllr = false;
		
		if(l_flg_tst_LI0_FR_CLLR_STATE_flag())
		{
			l_flg_clr_LI0_FR_CLLR_STATE_flag();
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag = false;
		}
		else 
		{ 
			if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
			{
				FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag = true;
			}
		}
	}

	if(lin_rxResponseFlag_clud == true)
	{
		lin_rxResponseFlag_clud = false;
		
		if(l_flg_tst_LI0_FR_CLUD_STATE_flag())
		{
//			PINS_DRV_SetPins(GPIO_PORTC, TEST_OUT1_MASK);

			l_flg_clr_LI0_FR_CLUD_STATE_flag();
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag = false;

//			PINS_DRV_ClearPins(GPIO_PORTC, TEST_OUT1_MASK);
		}
		else 
		{ 
			if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
			{
				FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag = true;
			}
		}
	}
}

void Lin_FR_PS_BusOff_Check(FR_LinBusOff_flag_t *fault_flag)
{
	if(lin_rxResponseFlag_crlr == true)
	{
		lin_rxResponseFlag_crlr = false;
		
		if(l_flg_tst_LI1_FR_CRLR_STATE_flag())
		{
			l_flg_clr_LI1_FR_CRLR_STATE_flag();
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag = false;
		}
		else 
		{ 
			if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
			{
				FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag = true;
			}
		}
	}

	if(lin_rxResponseFlag_crud == true)
	{
		lin_rxResponseFlag_crud = false;
		
		if(l_flg_tst_LI1_FR_CRUD_STATE_flag())
		{
			l_flg_clr_LI1_FR_CRUD_STATE_flag();
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag = false;
		}
		else 
		{ 
			if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
			{
				FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag = true;
			}
		}
	}

	if(lin_rxResponseFlag_srlr == true)
	{
		lin_rxResponseFlag_srlr = false;
		
		if(l_flg_tst_LI1_FR_SRLR_STATE_flag())
		{
			l_flg_clr_LI1_FR_SRLR_STATE_flag();
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag = false;
		}
		else 
		{ 
			if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
			{
				FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag = true;
			}
		}
	}

	if(lin_rxResponseFlag_srud == true)
	{
		lin_rxResponseFlag_srud = false;
		
		if(l_flg_tst_LI1_FR_SRUD_STATE_flag())
		{
			PINS_DRV_SetPins(GPIO_PORTC, TEST_OUT1_MASK);

			l_flg_clr_LI1_FR_SRUD_STATE_flag();
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag = false;

			PINS_DRV_ClearPins(GPIO_PORTC, TEST_OUT1_MASK);
		}
		else 
		{ 
			if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
			{
				FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag = true;
			}
		}
	}
}


#else
void Lin_FR_DR_BusOff_Check(FR_LinBusOff_flag_t *fault_flag)
{
	// SLLR LIN-Bus Off flag check
	if(l_flg_tst_LI0_FR_SLLR_STATE_flag())
	{
		l_flg_clr_LI0_FR_SLLR_STATE_flag();

#ifdef LIN_DTC_LINBUSOFF_TEST
		if((linBusoffCount_sllr < LIN_BUSOFF_DELAY) && (fault_flag->FR_SLLR_LIN_BusOff == false))
		{
			linBusoffCount_sllr++;
			
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag = true;
		}
		else //testCount >= 2 || fault_flag->FR_SLLR_LIN_BusOff != false
		{
			linBusoffCount_sllr = 0;
			fault_flag->FR_SLLR_LIN_BusOff = true;
			
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag = false;
		}
#else
//		if(fault_flag->FR_SLLR_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag = false;
		}

#endif

	}
	else 
	{ 
//		fault_flag->FR_SLLR_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag = true;
		}
	}

	// SLUD LIN-Bus Off flag check
	if(l_flg_tst_LI0_FR_SLUD_STATE_flag())
	{
		l_flg_clr_LI0_FR_SLUD_STATE_flag();

#ifdef LIN_DTC_LINBUSOFF_TEST
		if((linBusoffCount_slud < LIN_BUSOFF_DELAY) && (fault_flag->FR_SLUD_LIN_BusOff == false))
		{
			linBusoffCount_slud++;
			
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag = true;
		}

		else
		{
			linBusoffCount_slud = 0;
			fault_flag->FR_SLUD_LIN_BusOff = true;
			
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag = false;
		}
#else
//		if(fault_flag->FR_SLUD_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag = false;
		}
#endif

	}
	else 
	{ 
//		fault_flag->FR_SLUD_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag = true;
		}
	}

	// CLLR LIN-Bus Off flag check
	if(l_flg_tst_LI0_FR_CLLR_STATE_flag())
	{
		l_flg_clr_LI0_FR_CLLR_STATE_flag();
	
#ifdef LIN_DTC_LINBUSOFF_TEST
		if((linBusoffCount_cllr < LIN_BUSOFF_DELAY) && (fault_flag->FR_CLLR_LIN_BusOff == false))
		{
			linBusoffCount_cllr++;
			
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag = true;
		}
		else
		{
			linBusoffCount_cllr = 0;
			fault_flag->FR_CLLR_LIN_BusOff = true;
			
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag = false;
		}
#else
//		if(fault_flag->FR_CLLR_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag = false;
		}
#endif

	}
	else 
	{ 
//		fault_flag->FR_CLLR_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag = true;
		}
	}

	// CLUD LIN-Bus Off flag check
	if(l_flg_tst_LI0_FR_CLUD_STATE_flag())
	{
		l_flg_clr_LI0_FR_CLUD_STATE_flag();

#ifdef LIN_DTC_LINBUSOFF_TEST
		if((linBusoffCount_clud < LIN_BUSOFF_DELAY) && (fault_flag->FR_CLUD_LIN_BusOff == false))
		{
			linBusoffCount_clud++;
			
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag = true;
		}
		else
		{
			linBusoffCount_clud = 0;
			fault_flag->FR_CLUD_LIN_BusOff = true;
			
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag = false;
		}
#else
//		if(fault_flag->FR_CLUD_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag = false;
		}
#endif

	}
	else 
	{ 
//		fault_flag->FR_CLUD_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag = true;
		}
	}
}

void Lin_FR_PS_BusOff_Check(FR_LinBusOff_flag_t *fault_flag)
{
		// CRLR LIN-Bus Off flag check
	if(l_flg_tst_LI1_FR_CRLR_STATE_flag())
	{
		l_flg_clr_LI1_FR_CRLR_STATE_flag();

#ifdef LIN_DTC_LINBUSOFF_TEST
		if((linBusoffCount_crlr < LIN_BUSOFF_DELAY) && (fault_flag->FR_CRLR_LIN_BusOff == false))
		{
			linBusoffCount_crlr++;
			
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag = true;
		}
		else
		{
			linBusoffCount_crlr = 0;
			fault_flag->FR_CRLR_LIN_BusOff = true;
			
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag = false;
		}
#else
//		if(fault_flag->FR_CRLR_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag = false;
		}
#endif

	}
	else 
	{ 
//		fault_flag->FR_CRLR_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag = true;
		}
	}

	// CRUD LIN-Bus Off flag check
	if(l_flg_tst_LI1_FR_CRUD_STATE_flag())
	{
		l_flg_clr_LI1_FR_CRUD_STATE_flag();

#ifdef LIN_DTC_LINBUSOFF_TEST		
		if((linBusoffCount_crud < LIN_BUSOFF_DELAY) && (fault_flag->FR_CRUD_LIN_BusOff == false))
		{
			linBusoffCount_crud++;
			
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag = true;
		}
		else
		{
			linBusoffCount_crud = 0;
			fault_flag->FR_CRUD_LIN_BusOff = true;
			
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag = false;
		}
#else
//		if(fault_flag->FR_CRUD_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag = false;
		}
#endif

	}
	else 
	{ 
//		fault_flag->FR_CRUD_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag = true;
		}
	}

	// SRLR LIN-Bus Off flag check
	if(l_flg_tst_LI1_FR_SRLR_STATE_flag())
	{
		l_flg_clr_LI1_FR_SRLR_STATE_flag();

#ifdef LIN_DTC_LINBUSOFF_TEST		
		if((linBusoffCount_srlr < LIN_BUSOFF_DELAY) && (fault_flag->FR_SRLR_LIN_BusOff == false))
		{
			linBusoffCount_srlr++;
			
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag = true;
		}
		else
		{
			linBusoffCount_srlr = 0;
			fault_flag->FR_SRLR_LIN_BusOff = true;
			
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag = false;
		}
#else
//		if(fault_flag->FR_SRLR_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag = false;
		}
#endif

	}
	else 
	{ 
//		fault_flag->FR_SRLR_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag = true;
		}
	}

	// SRUD LIN-Bus Off flag check
	if(l_flg_tst_LI1_FR_SRUD_STATE_flag())
	{
		l_flg_clr_LI1_FR_SRUD_STATE_flag();

#ifdef LIN_DTC_LINBUSOFF_TEST		
		if((linBusoffCount_srud < LIN_BUSOFF_DELAY) && (fault_flag->FR_SRUD_LIN_BusOff == false))
		{
			linBusoffCount_srud++;
			
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag = true;
		}
		else
		{
			linBusoffCount_srud = 0;
			fault_flag->FR_SRUD_LIN_BusOff = true;
			
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag = false;
		}
#else
//		if(fault_flag->FR_SRUD_LIN_BusOff == false)
		{
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag = false;
		}
#endif

	}
	else 
	{ 
//		fault_flag->FR_SRUD_LIN_BusOff = true;

		if(AllBits_Zero_Fr_ExtFlt(FR_DTC_Flt_flag.FR_EXT_FLT.fltByte))
		{
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag = true;
		}
	}

}
#endif //LIN_BUSOFF_CHECK_DELAY_50MS



#ifdef LIN_BUSOFF_CHECK_DELAY_50MS
void lin_BusOff(void)
{
	if((linbusOff_FltCheck_Start_Flag == TRUE) && (Evnt_IGN2_Onoff == 1))
	{
		Lin_FR_DR_BusOff_Check(&FR_LINBUSOff_FLAG);
		Lin_FR_PS_BusOff_Check(&FR_LINBUSOff_FLAG);
	}
}

#else

void lin_BusOff(void)
{
	Lin_FR_DR_BusOff_Check(&FR_LINBUSOff_FLAG);
	Lin_FR_PS_BusOff_Check(&FR_LINBUSOff_FLAG);
}
#endif


void Lin_Init(void)
{
	/* Initialize LIN network interface */
	l_sys_init();

	l_ifc_init(LI0);
	l_ifc_init(LI1);
	
#ifdef AMO_GN7_PE_SETTING_NONE
	l_ifc_init(LI2);
#endif

	Lin_GoTo_WakeUp();

	/* Set Schedule table to Normal */

	l_sch_set(LI0, LI0_SCHEDULER_EVNT_NORMAL_FR_DRV, 0u);
	l_sch_set(LI1, LI1_SCHEDULER_EVNT_NORMAL_FR_PASS, 0u);

//	Amo_timer_Stop(timer_90);
//	Amo_timer_Start(timer_90, 500, true, lin_BusOff);  //500
	
#ifdef AMO_GN7_PE_SETTING_NONE	
	l_sch_set(LI2, LI2_SCHEDULER_EVNT_NORMAL_RR_CTR, 0u);
#endif
	
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);
	
#ifdef AMO_GN7_PE_SETTING_NONE
	SetLIN_EVNT_RrSPEED(&LIN_RC_EVNT_MASTER_CMD, ACT_RPM_10);
#endif

}

void lin_sleep_task(void)
{
	if(sleepFlag == 1)
	{
		l_sch_set(LI0, LI0_GOTO_SLEEP_SCHEDULE, 0u);
		sleepFlag = 0;
	}
	if(sleepFlag == 2)
	{
		l_ifc_wake_up(LI0);
		l_sch_set(LI0, LI0_SCHEDULER_EVNT_NORMAL_FR_DRV, 0u);
		sleepFlag = 0;
	}
	 /* Check node state */
	if (LIN_NODE_STATE_SLEEP_MODE == lin_lld_get_state(LI0))
	{
//		PINS_DRV_SetPins(GPIO_PORTC, TEST_OUT1_MASK);
		/* Turn off all LEDs */
	}
	if (LIN_NODE_STATE_IDLE == lin_lld_get_state(LI0))
	{
//		PINS_DRV_ClearPins(GPIO_PORTC, TEST_OUT1_MASK);
		/* Turn off all LEDs */
	}
}

void Set_LinToCan_L_EVNT_HU_FrDrEvntPtDisp_XY(Lvnt_1_data *L1_DataBuf, const Atcu_20_data *canPtDataBuf)
{
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_X;
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_Y;
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_X;
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_Y;

	Hu_FrDrEVntSidePt_X_backup = L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_X;
	Hu_FrDrEVntSidePt_Y_backup = L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_Y;
	Hu_FrDrEVntCtrPt_X_backup = L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_X;
	Hu_FrDrEVntCtrPt_Y_backup = L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y;	
}

void Set_LinToCan_L_EVNT_HU_FrPsEvntPtDisp_XY(Lvnt_2_data *L2_DataBuf, const Atcu_21_data *canPtDataBuf)
{
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_X;
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_Y;
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_X;
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_Y;

	Hu_FrPsEVntSidePt_X_backup = L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_X;
	Hu_FrPsEVntSidePt_Y_backup = L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_Y;
	Hu_FrPsEVntCtrPt_X_backup = L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_X;
	Hu_FrPsEVntCtrPt_Y_backup = L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y;
}

#ifdef AMO_GN7_PE_SETTING_NONE		
void Set_LinToCan_L_EVNT_HU_RrEvntPtDisp_XY(Lvnt_3_data *L3_DataBuf, const Atcu_22_data *canPtDataBuf)
{
	L3_DataBuf->data.L_EVNT_HU_RrDrEvntConsPtDisp_X = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_X;
	L3_DataBuf->data.L_EVNT_HU_RrDrEvntConsPtDisp_Y = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_Y;
	L3_DataBuf->data.L_EVNT_HU_RrPsEvntConsPtDisp_X = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_X;
	L3_DataBuf->data.L_EVNT_HU_RrPsEvntConsPtDisp_Y = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_Y;

	Hu_RrDrEVntConsPt_X_backup = L3_DataBuf->data.L_EVNT_HU_RrDrEvntConsPtDisp_X;
	Hu_RrDrEVntConsPt_Y_backup = L3_DataBuf->data.L_EVNT_HU_RrDrEvntConsPtDisp_Y;
	Hu_RrPsEVntConsPt_X_backup = L3_DataBuf->data.L_EVNT_HU_RrPsEvntConsPtDisp_X;
	Hu_RrPsEVntConsPt_Y_backup = L3_DataBuf->data.L_EVNT_HU_RrPsEvntConsPtDisp_Y;
}
#endif

void lin_master_task(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
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

void lin_Write_FrDrEVntPt_XY(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Lvnt_3_data *L3_DataBuf)
{
	//if(L3_DataBuf->data.L_EVNT_FrDrEvntCtrFlt != E_ERROR)
	//{
	if(Evnt_COM_Onoff)
	{
		l_u16_wr_LI0_EVNT_Front_Center_LH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition);
	//}
	//if(L3_DataBuf->data.L_EVNT_FrDrEvntSideFlt != E_ERROR)
	//{
		l_u16_wr_LI0_EVNT_Front_Side_LH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition);
		l_u16_wr_LI0_EVNT_Front_Side_LH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition);
	//}
	//if(L3_DataBuf->data.L_EVNT_FrDrEvntCtrFlt != E_ERROR)
	//{
		l_u16_wr_LI0_EVNT_Front_Center_LH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition);
	//}
	}
}

void lin_Write_FrPsEVntPt_XY(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Lvnt_3_data *L3_DataBuf)
{
//	if(L3_DataBuf->data.L_EVNT_FrPsEvntCtrFlt != E_ERROR)
	//{
	if(Evnt_COM_Onoff)
	{
		l_u16_wr_LI1_EVNT_Front_Center_RH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition);
		l_u16_wr_LI1_EVNT_Front_Center_RH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition);
	//}
//	if(L3_DataBuf->data.L_EVNT_FrPsEvntSideFlt != E_ERROR)
	//{
		l_u16_wr_LI1_EVNT_Front_Side_RH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition);
		l_u16_wr_LI1_EVNT_Front_Side_RH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition);
	//}
	}
}

#ifdef AMO_GN7_PE_SETTING_NONE		
void lin_Write_RrEVntConsPt_XY(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
	l_u16_wr_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition);
	l_u16_wr_LI2_EVNT_Rear_Center_RH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition);
	l_u16_wr_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition);
	l_u16_wr_LI2_EVNT_Rear_Center_LH_UpDown_TargetPosition(lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition);
}
#endif

void lin_FrDrmanualMode_task(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
//	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
//	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

//	Amo_timer_Stop(timer_60);
//	Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
	
	Set_LinToCan_L_EVNT_HU_FrDrEvntPtDisp_XY(&Can_Tx_Evnt_1, &Can_data_20_Info);
	//Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
}

void lin_FrDrmanualMode_task_Backup(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl) //MODE BACKUP
{
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

//	Amo_timer_Stop(timer_60);
//	Amo_timer_Start(timer_60, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_LH_Check);
	
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = Hu_FrDrEVntSidePt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = Hu_FrDrEVntSidePt_Y_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = Hu_FrDrEVntCtrPt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = Hu_FrDrEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = (uint64_t)Hu_Fr_Dr_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = Hu_Frdr_Vent_Side_sig_backup;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = Hu_Frdr_Vent_Center_sig_backup;
}

void lin_FrPsmanualMode_task(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
	
//		Amo_timer_Stop(timer_63);
//		Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);

	Set_LinToCan_L_EVNT_HU_FrPsEvntPtDisp_XY(&Can_Tx_Evnt_2, &Can_data_21_Info);
	//Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);
}

void lin_FrPsmanualMode_task_Backup(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl) //MODE BACKUP
{
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
	
//	Amo_timer_Stop(timer_63);
//	Amo_timer_Start(timer_63, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RH_Check);
	
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = Hu_FrPsEVntSidePt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = Hu_FrPsEVntSidePt_Y_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = Hu_FrPsEVntCtrPt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = Hu_FrPsEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = (uint64_t)Hu_Fr_Ps_Vent_Mode;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = Hu_FrPs_Vent_Side_sig_backup;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = Hu_FrPs_Vent_Center_sig_backup;	
}

#ifdef AMO_GN7_PE_SETTING_NONE		
void lin_RrmanualMode_task(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
	SetLIN_EVNT_RrSPEED(&LIN_RC_EVNT_MASTER_CMD, ACT_RPM_10);
	
	if(l_flg_tst_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition_flag() || l_flg_tst_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition_flag())
	{
	    //Clear this flag...
	  l_flg_clr_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition_flag();
		l_flg_clr_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition_flag();

		lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);

//		Amo_timer_Stop(timer_66);
//		Amo_timer_Start(timer_66, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RC_Check);
	}
	
	Set_LinToCan_L_EVNT_HU_RrEvntPtDisp_XY(&Can_Tx_Evnt_3, &Can_data_22_Info);
	Set_LinToCan_Disp_RrDr_Mode(&Can_Tx_Evnt_4);
	Set_LinToCan_Disp_RrPs_Mode(&Can_Tx_Evnt_4);
}
#endif






////////////////////////////////////////////////////////////////////////////////////////////////////////// Full Close 

typedef enum {
    SIDE_VENT,
    CENTER_VENT,
} VentPositionType;


void lin_FrDrVent_FullClose_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
   FrDr_FullCloseMode_task(Hu_Frdr_Vent_Side_sig_backup, Hu_Frdr_Vent_Center_sig_backup);
}

void RHD_lin_FrDrVent_FullClose_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	FrPs_FullCloseMode_task(Hu_Frdr_Vent_Side_sig_backup, Hu_Frdr_Vent_Center_sig_backup);
}

void lin_FrDrVent_FullClose_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	FrDr_FullCloseMode_task(Hu_Frdr_Vent_Side_sig_backup, Hu_Frdr_Vent_Center_sig_backup);
}

void RHD_lin_FrDrVent_FullClose_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	FrPs_FullCloseMode_task(Hu_Frdr_Vent_Side_sig_backup, Hu_Frdr_Vent_Center_sig_backup);
}

void lin_FrPsVent_FullClose_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
   FrPs_FullCloseMode_task(Hu_FrPs_Vent_Side_sig_backup, Hu_FrPs_Vent_Center_sig_backup);
}

void RHD_lin_FrPsVent_FullClose_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
   FrDr_FullCloseMode_task(Hu_FrPs_Vent_Side_sig_backup, Hu_FrPs_Vent_Center_sig_backup);
}

void lin_FrPsVent_FullClose_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	FrPs_FullCloseMode_task(Hu_FrPs_Vent_Side_sig_backup, Hu_FrPs_Vent_Center_sig_backup);
}

void RHD_lin_FrPsVent_FullClose_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	FrDr_FullCloseMode_task(Hu_FrPs_Vent_Side_sig_backup, Hu_FrPs_Vent_Center_sig_backup);
}



void FrDr_FullCloseCycleControl(void)
{
	if(Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Mode == SWING_MODE) 
	{
		lin_FrDrPsCycleMode_task();
	} 
	else if(Hu_Fr_Dr_Vent_Mode == SWING_MODE) 
	{
		lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
	}	
	else if(Hu_Fr_Ps_Vent_Mode == SWING_MODE && (Hu_Frdr_Vent_Center_sig_backup == 0x02 && Hu_Frdr_Vent_Side_sig_backup == 0x02))
	{
		lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
	}	
}


void FrDr_FullCloseMode_task(uint8_t side_sig, uint8_t center_sig)
{
		SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

	if(side_sig == 0x02) {
	  FrDr_SetLinTargetPosition_side((uint16_t)FullClose_min, (uint16_t)FullClose_UD_Rev);
	}

	if(center_sig == 0x02) {
	  FrDr_SetLinTargetPosition_ctr((uint16_t)FullClose_max, (uint16_t)FullClose_UD_Rev);
	}

	if((side_sig == 0x02 && center_sig != 0x02) || (center_sig == 0x02 && side_sig != 0x02) || (center_sig == 0x02 && side_sig == 0x02)) 
	{
	  FrDr_FullCloseCycleControl();
	}

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
	if(backup_Flag == FALSE)
	{
		Set_LinToCan_Disp_FrDr_XY(&Can_Tx_Evnt_1);
	}
	else if(backup_Flag == TRUE)
	{
		Set_LinToCan_Disp_FrDr_XY_Backup();
	}
}




void FrPs_FullCloseCycleControl(void)
{
	if(Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Mode == SWING_MODE) 
	{
			lin_FrDrPsCycleMode_task();
	} 
	else if(Hu_Fr_Ps_Vent_Mode == SWING_MODE) 
	{
			lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
	}
	else if(Hu_Fr_Dr_Vent_Mode == SWING_MODE && (Hu_FrPs_Vent_Center_sig_backup == 0x02 && Hu_FrPs_Vent_Side_sig_backup == 0x02))
	{
		lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
	}	
}


void FrPs_FullCloseMode_task(uint8_t side_sig, uint8_t center_sig)
{
		SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();

	if(side_sig == 0x02) {
	  FrPs_SetLinTargetPosition_side((uint16_t)FullClose_max, (uint16_t)FullClose_UD);
	}

	if(center_sig == 0x02) {
	  FrPs_SetLinTargetPosition_ctr((uint16_t)FullClose_min, (uint16_t)FullClose_UD);
	}

	if((side_sig == 0x02 && center_sig != 0x02) || (center_sig == 0x02 && side_sig != 0x02) || (center_sig == 0x02 && side_sig == 0x02)) 
	{
	  FrPs_FullCloseCycleControl();
	}

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
	if(backup_Flag == FALSE)
	{
		Set_LinToCan_Disp_FrPs_XY(&Can_Tx_Evnt_2);
	}
	else if(backup_Flag == TRUE)
	{
		Set_LinToCan_Disp_FrPs_XY_Backup();
	}
}




void FrDr_SetLinTargetPosition_side(uint16_t LH_Side_LR, uint16_t LH_Side_UD) 
{
	LIN_EVNT_PT.FrDrPt.prevPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
	LIN_EVNT_PT.FrDrPt.prevPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;

  LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = LH_Side_LR;
  LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = LH_Side_UD;

	LIN_EVNT_PT.FrDrPt.currentPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
	LIN_EVNT_PT.FrDrPt.currentPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
}

void FrDr_SetLinTargetPosition_ctr(uint16_t LH_Ctr_LR, uint16_t LH_Ctr_UD) 
{
	LIN_EVNT_PT.FrDrPt.prevPt_FR_CLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
	LIN_EVNT_PT.FrDrPt.prevPt_FR_CLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition; 

	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = LH_Ctr_LR;
	LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = LH_Ctr_UD;

	LIN_EVNT_PT.FrDrPt.currentPt_FR_CLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
	LIN_EVNT_PT.FrDrPt.currentPt_FR_CLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
}


void FrPs_SetLinTargetPosition_side(uint16_t RH_Side_LR, uint16_t RH_Side_UD) 
{
	LIN_EVNT_PT.FrPsPt.prevPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
	LIN_EVNT_PT.FrPsPt.prevPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = RH_Side_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = RH_Side_UD;

	LIN_EVNT_PT.FrPsPt.currentPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
	LIN_EVNT_PT.FrPsPt.currentPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
}

void FrPs_SetLinTargetPosition_ctr(uint16_t RH_Ctr_LR, uint16_t RH_Ctr_UD) 
{
	LIN_EVNT_PT.FrPsPt.prevPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
	LIN_EVNT_PT.FrPsPt.prevPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;

	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = RH_Ctr_LR;
	LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = RH_Ctr_UD;

	LIN_EVNT_PT.FrPsPt.currentPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
	LIN_EVNT_PT.FrPsPt.currentPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
}








///////////////////////////////////////////////////////////////////////////////////////////////////////////////// Focus & Spread


void lin_FrDrVent_Focus_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	FrDr_FocusSpreadMode_task();
}

void RHD_lin_FrDrVent_Focus_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	FrPs_FocusSpreadMode_task();
}

void lin_FrDrVent_Focus_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	FrDr_FocusSpreadMode_task();
}

void RHD_lin_FrDrVent_Focus_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	FrPs_FocusSpreadMode_task();
}

void lin_FrPsVent_Focus_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	FrPs_FocusSpreadMode_task();
}

void RHD_lin_FrPsVent_Focus_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	FrDr_FocusSpreadMode_task();
}

void lin_FrPsVent_Focus_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	FrPs_FocusSpreadMode_task();
}

void RHD_lin_FrPsVent_Focus_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	FrDr_FocusSpreadMode_task();
}

void lin_FrDrVent_Spread_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	FrDr_FocusSpreadMode_task();
}

void RHD_lin_FrDrVent_Spread_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	FrPs_FocusSpreadMode_task();
}

void lin_FrDrVent_Spread_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	FrDr_FocusSpreadMode_task();
}

void RHD_lin_FrDrVent_Spread_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	FrPs_FocusSpreadMode_task();
}

void lin_FrPsVent_Spread_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	FrPs_FocusSpreadMode_task();
}

void RHD_lin_FrPsVent_Spread_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	FrDr_FocusSpreadMode_task();
}

void lin_FrPsVent_Spread_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	FrPs_FocusSpreadMode_task();
}

void RHD_lin_FrPsVent_Spread_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	FrDr_FocusSpreadMode_task();
}




void FrDr_FocusSpreadMode_task(void)
{
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
	lin_FrDrCycleMode_TimerStop();
	lin_CycleMode_CanDisp_TimerStop();

	if(Hu_Fr_Dr_Vent_Mode == FACING_MODE)
	{
		FrDr_SetLinTargetPosition_side((uint16_t)Half_max_LR, (uint16_t)FocusSpread_UD_Rev);
		FrDr_SetLinTargetPosition_ctr((uint16_t)Half_min_LR, (uint16_t)FocusSpread_UD_Rev);
	}

	if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE)
	{
		FrDr_SetLinTargetPosition_side((uint16_t)Half_min_LR, (uint16_t)FocusSpread_UD_Rev);
		FrDr_SetLinTargetPosition_ctr((uint16_t)Half_max_LR, (uint16_t)FocusSpread_UD_Rev);
	}
	
	if((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_Ps_Vent_Mode == SWING_MODE ))
	{
		lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
	} 

	if((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Mode == SWING_MODE ))
	{
		lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
	} 

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	if(backup_Flag == FALSE)
	{
		Set_LinToCan_Disp_FrDr_XY(&Can_Tx_Evnt_1);
	}
	if(backup_Flag == TRUE)
	{
		Set_LinToCan_Disp_FrDr_XY_Backup();
	}
}



void FrPs_FocusSpreadMode_task(void)
{
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);
	lin_FrPsCycleMode_TimerStop();
	lin_CycleMode_CanDisp_TimerStop();

	if(Hu_Fr_Ps_Vent_Mode == FACING_MODE)
	{
		FrPs_SetLinTargetPosition_side((uint16_t)Half_min_LR, (uint16_t)FocusSpread_UD);
		FrPs_SetLinTargetPosition_ctr((uint16_t)Half_max_LR, (uint16_t)FocusSpread_UD);
	}

	if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE)
	{
		FrPs_SetLinTargetPosition_side((uint16_t)Half_max_LR, (uint16_t)FocusSpread_UD);
		FrPs_SetLinTargetPosition_ctr((uint16_t)Half_min_LR, (uint16_t)FocusSpread_UD);
	}
	
	if((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_Fr_Dr_Vent_Mode == SWING_MODE))
	{
		lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
	} 

	if((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Dr_Vent_Mode == SWING_MODE))
	{
		lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
	} 
	
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
	
	if(backup_Flag == FALSE)
	{
		Set_LinToCan_Disp_FrPs_XY(&Can_Tx_Evnt_2);
	}
	if(backup_Flag == TRUE)
	{
		Set_LinToCan_Disp_FrPs_XY_Backup();
	}

}








/////////////////////////////////////////////////////////////////////////////////////////////////



#ifdef AMO_GN7_PE_SETTING_NONE		
void lin_RrVentMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
	SetLIN_EVNT_RrSPEED(&LIN_RC_EVNT_MASTER_CMD, ACT_RPM_10);

	if(l_flg_tst_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition_flag() || l_flg_tst_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition_flag())
	{
		//Clear this flag...
	  l_flg_clr_LI2_EVNT_Rear_Center_LH_LeftRight_TargetPosition_flag();
		l_flg_clr_LI2_EVNT_Rear_Center_RH_LeftRight_TargetPosition_flag();
	
		//if((Hu_Rr_Dr_Vent_Mode == DEFAULT_MODE) && (Hu_Rr_Dr_Vent_signal == 0x02)) //fullclose
		if(Hu_Rr_Dr_Vent_signal == 0x02) //fullclose
		{
			lin_RrCycleMode_TimerStop();
		
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = (uint16_t)FullClose_min;
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = (uint16_t)FullClose_UD;
		}
		
		//if((Hu_Rr_Ps_Vent_Mode == DEFAULT_MODE) && (Hu_Rr_Ps_Vent_signal == 0x02)) //fullclose
		if(Hu_Rr_Ps_Vent_signal == 0x02) //fullclose
		{
			lin_RrCycleMode_TimerStop();
			
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = (uint16_t)FullClose_max;
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = (uint16_t)FullClose_UD;
		}

		if((Hu_Rr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Rr_Dr_Vent_signal == 0x01)) //spread mode
		{
			lin_RrCycleMode_TimerStop();
			
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = (uint16_t)Half_min_LR;
			#if defined (ACT_ANGLE_UPDATE_2)
		lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = (uint16_t)FocusSpread_UD;
		#else
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = (uint16_t)Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif
		}
		
		if((Hu_Rr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Rr_Ps_Vent_signal == 0x01)) //spread mode
		{
			lin_RrCycleMode_TimerStop();
			
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = (uint16_t)Half_max_LR;
			#if defined (ACT_ANGLE_UPDATE_2)
		lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = (uint16_t)FocusSpread_UD;
		#else
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = (uint16_t)Min_UD; //24degree // 1798 25degree; // 1788 26degree (2048-1788);
		#endif

		}

		if((Hu_Rr_Dr_Vent_Mode == FACING_MODE) && (Hu_Rr_Dr_Vent_signal == 0x01)) //focus mode
		{
			lin_RrCycleMode_TimerStop();
			
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = (uint16_t)Half_max_LR;
			#if defined (ACT_ANGLE_UPDATE_2)
		lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = (uint16_t)FocusSpread_UD;
		#else
			lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = 0;
		#endif
		}
		
		if(Hu_Rr_Ps_Vent_Mode == FACING_MODE && Hu_Rr_Ps_Vent_signal == 0x01) //focus mode
		{
			lin_RrCycleMode_TimerStop();
			
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = (uint16_t)Half_min_LR;
			#if defined (ACT_ANGLE_UPDATE_2)
		lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = (uint16_t)FocusSpread_UD;
		#else
			lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = 0;
		#endif

		}
		
		lin_Write_RrEVntConsPt_XY(&LIN_RC_EVNT_MASTER_CMD);

//		Amo_timer_Stop(timer_66);
//		Amo_timer_Start(timer_66, OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER, true, Opdone_TimeOut_RC_Check);

		Set_LinToCan_Disp_Rr_XY(&Can_Tx_Evnt_3);
		Set_LinToCan_Disp_RrDr_Mode(&Can_Tx_Evnt_4);
		Set_LinToCan_Disp_RrPs_Mode(&Can_Tx_Evnt_4);
	}
}
#endif

void SetLIN_EVNT_FrDrSPEED(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, uint8_t rpm)
{
	if(rpm == ACT_RPM_10) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_10; }
	else if(rpm == ACT_RPM_3) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_3; }
	else if(rpm == ACT_RPM_2) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_2; }
	else if(rpm == ACT_RPM_1) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_1; }
	
	l_u8_wr_LI0_EVNT_SPEED(lin_Ctrl->LDATA.EVNT_SPEED);
}


void SetLIN_EVNT_FrPsSPEED(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, uint8_t rpm)
{
	if(rpm == ACT_RPM_10) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_10; }
	else if(rpm == ACT_RPM_3) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_3; }
	else if(rpm == ACT_RPM_2) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_2; }
	else if(rpm == ACT_RPM_1) { lin_Ctrl->LDATA.EVNT_SPEED = ACT_RPM_1; }
	
	l_u8_wr_LI1_EVNT_SPEED(lin_Ctrl->LDATA.EVNT_SPEED);
}

#ifdef AMO_GN7_PE_SETTING_NONE		
void SetLIN_EVNT_RrSPEED(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, uint8_t rpm)
{

	if(rpm == ACT_RPM_10) 
	{ 
		lin_Ctrl->LDATA.EVNT_REAR_LH_SPEED = ACT_RPM_10;
		lin_Ctrl->LDATA.EVNT_REAR_RH_SPEED = ACT_RPM_10;
	}
	if(rpm == ACT_RPM_3) 
	{ 
		lin_Ctrl->LDATA.EVNT_REAR_LH_SPEED = ACT_RPM_3;
		lin_Ctrl->LDATA.EVNT_REAR_RH_SPEED = ACT_RPM_3; 
	}

	l_u8_wr_LI2_EVNT_REAR_LH_SPEED(lin_Ctrl->LDATA.EVNT_REAR_LH_SPEED);
	l_u8_wr_LI2_EVNT_REAR_RH_SPEED(lin_Ctrl->LDATA.EVNT_REAR_RH_SPEED);
}
#endif



void FrDrLinData_Parsing(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt)
{
	if(Hu_Frdr_Vent_Side_sig_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrDrEVntSidePt_X_invalid != 2047) || (Hu_FrDrEVntSidePt_Y_invalid != 2047))
		{

//			if(evntModeflags.FR_SL_Mode_Flag == TRUE) 
//			{ 
				lin_Pt->FrDrPt.prevPt_FR_SLLR = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
				lin_Pt->FrDrPt.prevPt_FR_SLUD = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;

//				evntModeflags.FR_SL_Mode_Flag = FALSE; 
//			}
			
			lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_X(&Can_data_20_Info, &Can_data_23_Info);
	  	lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_Y(&Can_data_20_Info, &Can_data_23_Info);

			lin_Pt->FrDrPt.currentPt_FR_SLLR = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
			lin_Pt->FrDrPt.currentPt_FR_SLUD = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;


			SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
			lin_Pt->FrDrPt.targetPt = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
			process_slave_command(SLAVE_SLLR, &LIN_EVNT_PT, 10);
			lin_Pt->FrDrPt.targetPt = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
			process_slave_command(SLAVE_SLUD, &LIN_EVNT_PT, 10);
			
			
			Fr_Dr_side_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
			Fr_Dr_side_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;		
		}
		
		if(Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Mode == SWING_MODE)
		{
			lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
		}	
	
		else
		{
	
		}
	}
	
	if(Hu_Frdr_Vent_Center_sig_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrDrEVntCtrPt_X_invalid != 2047) || (Hu_FrDrEVntCtrPt_Y_invalid != 2047))
		{

//			if(evntModeflags.FR_CL_Mode_Flag == TRUE) 
//			{ 
				lin_Pt->FrDrPt.prevPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
				lin_Pt->FrDrPt.prevPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
				
//				evntModeflags.FR_CL_Mode_Flag = FALSE; 
//			}

			lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_X(&Can_data_20_Info, &Can_data_23_Info);
			lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_Y(&Can_data_20_Info, &Can_data_23_Info);	

			lin_Pt->FrDrPt.currentPt_FR_CLLR = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
			lin_Pt->FrDrPt.currentPt_FR_CLUD = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
			
			Fr_Dr_ctr_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
			Fr_Dr_ctr_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;		
		}
		
		if(Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Mode == SWING_MODE)
		{
			lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
		}	
	
		else
		{
	
		}
	}
}

void RHD_FrDrLinData_Parsing(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	if(Hu_FrPs_Vent_Side_sig_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrPsEVntSidePt_X_invalid != 2047) || (Hu_FrPsEVntSidePt_Y_invalid != 2047))
		{
			LIN_EVNT_PT.FrDrPt.prevPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrDrPt.prevPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
			
			lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_X(&Can_data_20_Info, &Can_data_23_Info);
	  	lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_Y(&Can_data_20_Info, &Can_data_23_Info);

			LIN_EVNT_PT.FrDrPt.currentPt_FR_SLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrDrPt.currentPt_FR_SLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
		}
		
		Fr_Dr_side_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
		Fr_Dr_side_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;		
	}
	
	if(Hu_FrPs_Vent_Center_sig_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrPsEVntCtrPt_X_invalid != 2047) || (Hu_FrPsEVntCtrPt_Y_invalid != 2047))
		{
			LIN_EVNT_PT.FrDrPt.prevPt_FR_CLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrDrPt.prevPt_FR_CLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
			
			lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_X(&Can_data_20_Info, &Can_data_23_Info);
			lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_Y(&Can_data_20_Info, &Can_data_23_Info);	

			LIN_EVNT_PT.FrDrPt.currentPt_FR_CLLR = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrDrPt.currentPt_FR_CLUD = LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
		}
		Fr_Dr_ctr_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
		Fr_Dr_ctr_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;		
	}
}


#ifdef KEY_ENABLE
void FrDrLinData_Parsing_Key(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_X_Key(&Can_data_20_Info_Key, &Can_data_23_Info);
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntSidePt_Y_Key(&Can_data_20_Info_Key, &Can_data_23_Info);

		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_X_Key(&Can_data_20_Info_Key, &Can_data_23_Info);
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = SET_L_ATCU_HU_FrDrEVntCtrPt_Y_Key(&Can_data_20_Info_Key, &Can_data_23_Info);

}
#endif

void FrDrLinData_Parsing_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{

	if(Hu_Frdr_Vent_Side_sig_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Fr_Dr_side_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Fr_Dr_side_updown_target_backup;
	}

	if(Hu_Frdr_Vent_Center_sig_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Fr_Dr_ctr_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Fr_Dr_ctr_updown_target_backup;
	}
	
	if(Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Mode == SWING_MODE)
	{
		lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
	} 
	
	else
	{
	
	}

}

void RHD_FrDrLinData_Parsing_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{

	if(Hu_FrPs_Vent_Side_sig_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = Fr_Dr_side_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = Fr_Dr_side_updown_target_backup;
	}

	if(Hu_FrPs_Vent_Center_sig_backup != 0x02)
	{
		lin_FrDrCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = Fr_Dr_ctr_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = Fr_Dr_ctr_updown_target_backup;
	}

}


void FrPsLinData_Parsing(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	if(Hu_FrPs_Vent_Side_sig_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrPsEVntSidePt_X_invalid != 2047) || (Hu_FrPsEVntSidePt_Y_invalid != 2047))
		{
			LIN_EVNT_PT.FrPsPt.prevPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.prevPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
			
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_X(&Can_data_21_Info, &Can_data_24_Info);
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_Y(&Can_data_21_Info, &Can_data_24_Info);	

			LIN_EVNT_PT.FrPsPt.currentPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.currentPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
		}
		if(Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Dr_Vent_Mode == SWING_MODE)
		{
			lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
		}	
	
		else
		{
	
		}
		Fr_Ps_side_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		Fr_Ps_side_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;		
	}

	if(Hu_FrPs_Vent_Center_sig_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrPsEVntCtrPt_X_invalid != 2047) || (Hu_FrPsEVntCtrPt_Y_invalid != 2047))
		{
			LIN_EVNT_PT.FrPsPt.prevPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.prevPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
			
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_X(&Can_data_21_Info, &Can_data_24_Info);
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_Y(&Can_data_21_Info, &Can_data_24_Info);

			LIN_EVNT_PT.FrPsPt.currentPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.currentPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
		}
		if(Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Dr_Vent_Mode == SWING_MODE)
		{
			lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
		}	
	
		else
		{
	
		}
		Fr_Ps_ctr_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		Fr_Ps_ctr_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;		
	}
}


void RHD_FrPsLinData_Parsing(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	if(Hu_Frdr_Vent_Side_sig_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrDrEVntSidePt_X_invalid != 2047) || (Hu_FrDrEVntSidePt_Y_invalid != 2047))
		{
			LIN_EVNT_PT.FrPsPt.prevPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.prevPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
			
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_X(&Can_data_21_Info, &Can_data_24_Info);
			lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_Y(&Can_data_21_Info, &Can_data_24_Info);	
			
			LIN_EVNT_PT.FrPsPt.currentPt_FR_SRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.currentPt_FR_SRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
		}
		Fr_Ps_side_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
		Fr_Ps_side_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;		
	}

	if(Hu_Frdr_Vent_Center_sig_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		if((Hu_FrDrEVntCtrPt_X_invalid != 2047) || (Hu_FrDrEVntCtrPt_Y_invalid != 2047))
		{
			LIN_EVNT_PT.FrPsPt.prevPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.prevPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
		
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_X(&Can_data_21_Info, &Can_data_24_Info);
			lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_Y(&Can_data_21_Info, &Can_data_24_Info);
			
			LIN_EVNT_PT.FrPsPt.currentPt_FR_CRLR = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
			LIN_EVNT_PT.FrPsPt.currentPt_FR_CRUD = LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
		}
		Fr_Ps_ctr_leftright_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
		Fr_Ps_ctr_updown_target_backup = lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;		
	}
}


#ifdef KEY_ENABLE
void FrPsLinData_Parsing_Key(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{

		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_X_Key(&Can_data_21_Info_Key, &Can_data_24_Info);
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntSidePt_Y_Key(&Can_data_21_Info_Key, &Can_data_24_Info);

		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_X_Key(&Can_data_21_Info_Key, &Can_data_24_Info);
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = SET_L_ATCU_HU_FrPsEVntCtrPt_Y_Key(&Can_data_21_Info_Key, &Can_data_24_Info);
}
#endif

void FrPsLinData_Parsing_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{

	if(Hu_FrPs_Vent_Side_sig_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Fr_Ps_side_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Fr_Ps_side_updown_target_backup;
	}
	
	if(Hu_FrPs_Vent_Center_sig_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Fr_Ps_ctr_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Fr_Ps_ctr_updown_target_backup;
	}

		if(Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Dr_Vent_Mode == SWING_MODE)
		{
			lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
		}	
	
		else
		{
	
		}
}

void RHD_FrPsLinData_Parsing_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{

	if(Hu_Frdr_Vent_Side_sig_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = Fr_Ps_side_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = Fr_Ps_side_updown_target_backup;
	}
	
	if(Hu_Frdr_Vent_Center_sig_backup != 0x02)
	{
		lin_FrPsCycleMode_TimerStop();
		lin_CycleMode_CanDisp_TimerStop();
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = Fr_Ps_ctr_leftright_target_backup;
		lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = Fr_Ps_ctr_updown_target_backup;
	}
}

#ifdef AMO_GN7_PE_SETTING_NONE		
void RrLinData_Parsing(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl)
{
		lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition = SET_L_ATCU_HU_RrDrEVntConsPt_X(&Can_data_22_Info, &Can_data_25_Info);
		lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition = SET_L_ATCU_HU_RrDrEVntConsPt_Y(&Can_data_22_Info, &Can_data_25_Info);

		lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition = SET_L_ATCU_HU_RrPsEVntConsPt_X(&Can_data_22_Info, &Can_data_25_Info);
		lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition = SET_L_ATCU_HU_RrPsEVntConsPt_Y(&Can_data_22_Info, &Can_data_25_Info);
	
}
#endif

uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_X(const Atcu_20_data *canPtDataBuf, const Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrDrSidePt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_X * canFactor));
	CanLinValue[FrDrSidePt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_X;

	CanLinValue[FrDrSidePt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrDr_SidePt_X(Conv_Can_Pt_X);
	#if defined (ACT_ANGLE_UPDATE_3)

	ret_value = 4096 - CanLinValue[FrDrSidePt_X].Pt_X.LinData_Pt_X;
	#else
	ret_value = CanLinValue[FrDrSidePt_X].Pt_X.LinData_Pt_X;
	#endif
		
	return ret_value;
}

uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_X(const Atcu_20_data *canPtDataBuf, const Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_X * canFactor));
	CanLinValue[FrDrCtrPt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_X;

	CanLinValue[FrDrCtrPt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrDr_CtrPt_X(Conv_Can_Pt_X);
	#if defined (ACT_ANGLE_UPDATE_3)
	
	ret_value = 4096 - CanLinValue[FrDrCtrPt_X].Pt_X.LinData_Pt_X;
	#else
	ret_value = CanLinValue[FrDrCtrPt_X].Pt_X.LinData_Pt_X;
	#endif
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_Y(const Atcu_20_data *canPtDataBuf, const Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrDrSidePt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_Y * canFactor));
	CanLinValue[FrDrSidePt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_Y;

	CanLinValue[FrDrSidePt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrDr_SidePt_Y(Conv_Can_Pt_Y);
	#if defined (ACT_ANGLE_UPDATE_3)

	ret_value = 4096 - CanLinValue[FrDrSidePt_Y].Pt_Y.LinData_Pt_Y;
	#else
	ret_value = CanLinValue[FrDrSidePt_Y].Pt_Y.LinData_Pt_Y;
	#endif
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_Y(const Atcu_20_data *canPtDataBuf, const Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	
	CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y * canFactor));
	CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_Y;

	CanLinValue[FrDrCtrPt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrDr_CtrPt_Y(Conv_Can_Pt_Y);

	ret_value = 4096 - CanLinValue[FrDrCtrPt_Y].Pt_Y.LinData_Pt_Y;

	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_X(const Atcu_21_data *canPtDataBuf, const Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrPsSidePt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_X * canFactor));
	CanLinValue[FrPsSidePt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_X;

	CanLinValue[FrPsSidePt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrPs_SidePt_X(Conv_Can_Pt_X);
	#if defined (ACT_ANGLE_UPDATE_3)

	ret_value = 4096 -CanLinValue[FrPsSidePt_X].Pt_X.LinData_Pt_X;
	#else
	ret_value = CanLinValue[FrPsSidePt_X].Pt_X.LinData_Pt_X;
	#endif
		
	return ret_value;
}

uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_X(const Atcu_21_data *canPtDataBuf, const Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrPsCtrPt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_X * canFactor));
	CanLinValue[FrPsCtrPt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_X;

	CanLinValue[FrPsCtrPt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrPs_CtrPt_X(Conv_Can_Pt_X);
	#if defined (ACT_ANGLE_UPDATE_3)
	
	ret_value = 4096 -  CanLinValue[FrPsCtrPt_X].Pt_X.LinData_Pt_X;
	#else
	ret_value = CanLinValue[FrPsCtrPt_X].Pt_X.LinData_Pt_X;
	#endif
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_Y(const Atcu_21_data *canPtDataBuf, const Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[FrPsSidePt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_Y * canFactor));
	CanLinValue[FrPsSidePt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_Y;

	CanLinValue[FrPsSidePt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrPs_SidePt_Y(Conv_Can_Pt_Y);
	#if defined (ACT_ANGLE_UPDATE_3)

	ret_value = CanLinValue[FrPsSidePt_Y].Pt_Y.LinData_Pt_Y;
	#else
	ret_value = 4096 - CanLinValue[FrPsSidePt_Y].Pt_Y.LinData_Pt_Y;
	#endif
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_Y(const Atcu_21_data *canPtDataBuf, const Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	
	CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y * canFactor));
	CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_Y;

	CanLinValue[FrPsCtrPt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrPs_CtrPt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[FrPsCtrPt_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}

////////////////////////////////////////////////////////
#ifdef KEY_ENABLE
uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_X_Key(const Atcu_20_data_Key *canPtDataBuf, const Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrDrSidePt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_X * canFactor));
	CanLinValue[FrDrSidePt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_X_Key;
	CanLinValue[FrDrSidePt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrDr_SidePt_X(Conv_Can_Pt_X);
	ret_value = CanLinValue[FrDrSidePt_X].Pt_X.LinData_Pt_X;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_X_Key(const Atcu_20_data_Key *canPtDataBuf, const Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_X * canFactor));
	CanLinValue[FrDrCtrPt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_X_Key;
	CanLinValue[FrDrCtrPt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrDr_CtrPt_X(Conv_Can_Pt_X);
	ret_value = CanLinValue[FrDrCtrPt_X].Pt_X.LinData_Pt_X;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_Y_Key(const Atcu_20_data_Key *canPtDataBuf, const Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrDrSidePt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_Y * canFactor));
	CanLinValue[FrDrSidePt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntSidePt_Y_Key;
	CanLinValue[FrDrSidePt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrDr_SidePt_Y(Conv_Can_Pt_Y);
	ret_value = CanLinValue[FrDrSidePt_Y].Pt_Y.LinData_Pt_Y;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_Y_Key(const Atcu_20_data_Key *canPtDataBuf, const Atcu_23_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y * canFactor));
	CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrDrEVntCtrPt_Y_Key;
	CanLinValue[FrDrCtrPt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrDr_CtrPt_Y(Conv_Can_Pt_Y);
	ret_value = CanLinValue[FrDrCtrPt_Y].Pt_Y.LinData_Pt_Y;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_X_Key(const Atcu_21_data_Key *canPtDataBuf, const Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrPsSidePt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_X * canFactor));
	CanLinValue[FrPsSidePt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_X_Key;
	CanLinValue[FrPsSidePt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrPs_SidePt_X(Conv_Can_Pt_X);
	ret_value = CanLinValue[FrPsSidePt_X].Pt_X.LinData_Pt_X;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_X_Key(const Atcu_21_data_Key *canPtDataBuf, const Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrPsCtrPt_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_X * canFactor));
	CanLinValue[FrPsCtrPt_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_X_Key;
	CanLinValue[FrPsCtrPt_X].Pt_X.LinData_Pt_X = Cal_CanToLin_FrPs_CtrPt_X(Conv_Can_Pt_X);
	ret_value = CanLinValue[FrPsCtrPt_X].Pt_X.LinData_Pt_X;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_Y_Key(const Atcu_21_data_Key *canPtDataBuf, const Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrPsSidePt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_Y * canFactor));
	CanLinValue[FrPsSidePt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntSidePt_Y_Key;
	CanLinValue[FrPsSidePt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrPs_SidePt_Y(Conv_Can_Pt_Y);
	ret_value = CanLinValue[FrPsSidePt_Y].Pt_Y.LinData_Pt_Y;
	return ret_value;
}
uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_Y_Key(const Atcu_21_data_Key *canPtDataBuf, const Atcu_24_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y * canFactor));
	CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_FrPsEVntCtrPt_Y_Key;
	CanLinValue[FrPsCtrPt_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_FrPs_CtrPt_Y(Conv_Can_Pt_Y);
	ret_value = CanLinValue[FrPsCtrPt_Y].Pt_Y.LinData_Pt_Y;
	return ret_value;
}
///////////////////////////////////////////////
#endif


#ifdef AMO_GN7_PE_SETTING_NONE		
uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_X(const Atcu_22_data *canPtDataBuf, const Atcu_25_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[RrDrCons_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_X * canFactor));
	CanLinValue[RrDrCons_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_X;

	CanLinValue[RrDrCons_X].Pt_X.LinData_Pt_X = Cal_CanToLin_RrDr_ConsPt_X(Conv_Can_Pt_X);

	ret_value = CanLinValue[RrDrCons_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}

uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_X(const Atcu_22_data *canPtDataBuf, const Atcu_25_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[RrPsCons_X].Pt_X.Can_RangePt_X = ((canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_X * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_X * canFactor));
	CanLinValue[RrPsCons_X].Pt_X.Can_EVntPt_X = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_X;

	CanLinValue[RrPsCons_X].Pt_X.LinData_Pt_X = Cal_CanToLin_RrPs_ConsPt_X(Conv_Can_Pt_X);
	
	ret_value = CanLinValue[RrPsCons_X].Pt_X.LinData_Pt_X;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_Y(const Atcu_22_data *canPtDataBuf, const Atcu_25_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;

	CanLinValue[RrDrCons_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_Y * canFactor));
	CanLinValue[RrDrCons_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_RrDrEVntConsPt_Y;

	CanLinValue[RrDrCons_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_RrDr_ConsPt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[RrDrCons_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}


uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_Y(const Atcu_22_data *canPtDataBuf, const Atcu_25_data *canBdryDataBuf)
{
	int16_t ret_value = FALSE;
	
	CanLinValue[RrPsCons_Y].Pt_Y.Can_RangePt_Y = ((canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_Y * canFactor) - (canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_Y * canFactor));
	CanLinValue[RrPsCons_Y].Pt_Y.Can_EVntPt_Y = canPtDataBuf->data.L_ATCU_HU_RrPsEVntConsPt_Y;

	CanLinValue[RrPsCons_Y].Pt_Y.LinData_Pt_Y = Cal_CanToLin_RrPs_ConsPt_Y(Conv_Can_Pt_Y);

	ret_value = CanLinValue[RrPsCons_Y].Pt_Y.LinData_Pt_Y;
		
	return ret_value;
}
#endif



void Set_LinToCan_Disp_FrDr_XY(Lvnt_1_data *L1_DataBuf)
{
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_X = SET_L_EVNT_HU_FrDrEVntSidePt_X(&LIN_LH_EVNT_MASTER_CMD, &Can_data_23_Info);
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = SET_L_EVNT_HU_FrDrEVntSidePt_Y(&LIN_LH_EVNT_MASTER_CMD, &Can_data_23_Info);
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = SET_L_EVNT_HU_FrDrEVntCtrPt_X(&LIN_LH_EVNT_MASTER_CMD, &Can_data_23_Info);
	L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = SET_L_EVNT_HU_FrDrEVntCtrPt_Y(&LIN_LH_EVNT_MASTER_CMD, &Can_data_23_Info);

	Hu_FrDrEVntSidePt_X_backup = (uint16_t)L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_X;
	Hu_FrDrEVntSidePt_Y_backup = (uint16_t)L1_DataBuf->data.L_EVNT_HU_FrDrEvntSidePtDisp_Y;
	Hu_FrDrEVntCtrPt_X_backup = (uint16_t)L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_X;
	Hu_FrDrEVntCtrPt_Y_backup = (uint16_t)L1_DataBuf->data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y;

	if((Hu_Fr_dr_Vent_Side_signal != 0x02) && (Hu_Fr_dr_Vent_Center_signal != 0x02))
	{
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_X = Hu_FrDrEVntSidePt_X_backup;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_Y = Hu_FrDrEVntSidePt_Y_backup;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_X = Hu_FrDrEVntCtrPt_X_backup;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_Y = Hu_FrDrEVntCtrPt_Y_backup;
		Hu_FrDrEVntSidePt_X_reserve = Hu_FrDrEVntSidePt_X_backup;
		Hu_FrDrEVntSidePt_Y_reserve = Hu_FrDrEVntSidePt_Y_backup;
		Hu_FrDrEVntCtrPt_X_reserve = Hu_FrDrEVntCtrPt_X_backup;
		Hu_FrDrEVntCtrPt_Y_reserve = Hu_FrDrEVntCtrPt_Y_backup;		
	}

	if((Hu_Fr_dr_Vent_Side_signal == 0x02) || (Hu_Fr_dr_Vent_Center_signal == 0x02))
	{
		if(Hu_Fr_dr_Vent_Side_signal == 0x02)
		{
			Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_X = Hu_FrDrEVntSidePt_X_backup;
			Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_Y = Hu_FrDrEVntSidePt_Y_backup;	
		}
		else if(Hu_Fr_dr_Vent_Center_signal == 0x02)
		{
			Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_X = Hu_FrDrEVntCtrPt_X_backup;
			Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_Y = Hu_FrDrEVntCtrPt_Y_backup;			
		}
		else
		{
		
		}
	}	
}

void Set_LinToCan_Disp_FrDr_XY_Backup(void)
{
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = Hu_FrDrEVntSidePt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = Hu_FrDrEVntSidePt_Y_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = Hu_FrDrEVntCtrPt_X_backup;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = Hu_FrDrEVntCtrPt_Y_backup;

//	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = Hu_Frdr_Vent_Side_sig_backup;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = Hu_Frdr_Vent_Center_sig_backup; 		
}


uint16_t SET_L_EVNT_HU_FrDrEVntSidePt_X(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Atcu_23_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_X;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_FrDrEVntSidePt_Y(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Atcu_23_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_Y;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Reverse_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_FrDrEVntCtrPt_X(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Atcu_23_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_X;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_FrDrEVntCtrPt_Y(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Atcu_23_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Reverse_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

void Set_LinToCan_Disp_FrPs_XY(Lvnt_2_data *L2_DataBuf)
{
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_X = SET_L_EVNT_HU_FrPsEVntSidePt_X(&LIN_RH_EVNT_MASTER_CMD, &Can_data_24_Info);
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = SET_L_EVNT_HU_FrPsEVntSidePt_Y(&LIN_RH_EVNT_MASTER_CMD, &Can_data_24_Info);
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = SET_L_EVNT_HU_FrPsEVntCtrPt_X(&LIN_RH_EVNT_MASTER_CMD, &Can_data_24_Info);
	L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = SET_L_EVNT_HU_FrPsEVntCtrPt_Y(&LIN_RH_EVNT_MASTER_CMD, &Can_data_24_Info);

	Hu_FrPsEVntSidePt_X_backup = (uint16_t)L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_X;
	Hu_FrPsEVntSidePt_Y_backup = (uint16_t)L2_DataBuf->data.L_EVNT_HU_FrPsEvntSidePtDisp_Y;
	Hu_FrPsEVntCtrPt_X_backup = (uint16_t)L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_X;
	Hu_FrPsEVntCtrPt_Y_backup = (uint16_t)L2_DataBuf->data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y;

	if((Hu_Fr_Ps_Vent_Side_signal != 0x02) && (Hu_Fr_Ps_Vent_Center_signal != 0x02))
	{
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_X = Hu_FrPsEVntSidePt_X_backup;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_Y = Hu_FrPsEVntSidePt_Y_backup;	
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_X = Hu_FrPsEVntCtrPt_X_backup;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_Y = Hu_FrPsEVntCtrPt_Y_backup;	
		Hu_FrPsEVntSidePt_X_reserve = Hu_FrPsEVntSidePt_X_backup;
		Hu_FrPsEVntSidePt_Y_reserve = Hu_FrPsEVntSidePt_Y_backup;
		Hu_FrPsEVntCtrPt_X_reserve = Hu_FrPsEVntCtrPt_X_backup;
		Hu_FrPsEVntCtrPt_Y_reserve = Hu_FrPsEVntCtrPt_Y_backup;
	}
	
	if((Hu_Fr_Ps_Vent_Side_signal == 0x02) || (Hu_Fr_Ps_Vent_Center_signal == 0x02))
	{
		if(Hu_Fr_Ps_Vent_Side_signal == 0x02)
		{
			Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_X = Hu_FrPsEVntSidePt_X_backup;
			Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_Y = Hu_FrPsEVntSidePt_Y_backup;	
		}
		else if(Hu_Fr_Ps_Vent_Center_signal == 0x02)
		{
			Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_X = Hu_FrPsEVntCtrPt_X_backup;
			Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_Y = Hu_FrPsEVntCtrPt_Y_backup;			
		}
		else
		{
		
		}
	}
}

void Set_LinToCan_Disp_FrPs_XY_Backup(void)
{
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = Hu_FrPsEVntSidePt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = Hu_FrPsEVntSidePt_Y_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = Hu_FrPsEVntCtrPt_X_backup;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = Hu_FrPsEVntCtrPt_Y_backup;

//	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = Hu_FrPs_Vent_Side_sig_backup;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = Hu_FrPs_Vent_Center_sig_backup;
}
	
uint16_t SET_L_EVNT_HU_FrPsEVntSidePt_X(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Atcu_24_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_X;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_FrPsEVntSidePt_Y(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Atcu_24_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_Y;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_FrPsEVntCtrPt_X(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Atcu_24_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_X;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_FrPsEVntCtrPt_Y(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Atcu_24_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

#ifdef AMO_GN7_PE_SETTING_NONE		
void Set_LinToCan_Disp_Rr_XY(Lvnt_3_data *L3_DataBuf)
{
	L3_DataBuf->data.L_EVNT_HU_RrDrEvntConsPtDisp_X = SET_L_EVNT_HU_RrDrEVntSidePt_X(&LIN_RC_EVNT_MASTER_CMD, &Can_data_25_Info);
	L3_DataBuf->data.L_EVNT_HU_RrDrEvntConsPtDisp_Y = SET_L_EVNT_HU_RrDrEVntSidePt_Y(&LIN_RC_EVNT_MASTER_CMD, &Can_data_25_Info);
	L3_DataBuf->data.L_EVNT_HU_RrPsEvntConsPtDisp_X = SET_L_EVNT_HU_RrPsEVntCtrPt_X(&LIN_RC_EVNT_MASTER_CMD, &Can_data_25_Info);
	L3_DataBuf->data.L_EVNT_HU_RrPsEvntConsPtDisp_Y = SET_L_EVNT_HU_RrPsEVntCtrPt_Y(&LIN_RC_EVNT_MASTER_CMD, &Can_data_25_Info);
}

uint16_t SET_L_EVNT_HU_RrDrEVntSidePt_X(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, const Atcu_25_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Rear_Center_LH_LeftRight_TargetPosition;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_X;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_RrDrEVntSidePt_Y(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, const Atcu_25_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Rear_Center_LH_UpDown_TargetPosition;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryUp_Y;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrDrEVntConsBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_RrPsEVntCtrPt_X(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, const Atcu_25_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Rear_Center_RH_LeftRight_TargetPosition;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_X;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_L_EVNT_HU_RrPsEVntCtrPt_Y(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, const Atcu_25_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl->LDATA.EVNT_Rear_Center_RH_UpDown_TargetPosition;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryUP_Y;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_RrPsEVntConsBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}
#endif

void Set_LinToCan_Disp_FrDr_Mode(Lvnt_4_data *L4_DataBuf)
{
	L4_DataBuf->data.L_EVNT_FrDrEvntModeSet = (uint64_t)Hu_Fr_Dr_Vent_Mode;
	L4_DataBuf->data.L_EVNT_FrDrEvntSideOpnCls = Hu_Frdr_Vent_Side_sig_backup;
	L4_DataBuf->data.L_EVNT_FrDrEvntCtrOpnCls = Hu_Frdr_Vent_Center_sig_backup;
}

void Set_LinToCan_Disp_FrPs_Mode(Lvnt_4_data *L4_DataBuf)
{
	L4_DataBuf->data.L_EVNT_FrPsEvntModeSet = (uint64_t)Hu_Fr_Ps_Vent_Mode;
	L4_DataBuf->data.L_EVNT_FrPsEvntSideOpnCls = Hu_FrPs_Vent_Side_sig_backup;
	L4_DataBuf->data.L_EVNT_FrPsEvntCtrOpnCls = Hu_FrPs_Vent_Center_sig_backup;
}

#ifdef AMO_GN7_PE_SETTING_NONE		
void Set_LinToCan_Disp_RrDr_Mode(Lvnt_4_data *L4_DataBuf)
{
	L4_DataBuf->data.L_EVNT_RrDrEvntModeSet = (uint64_t)Hu_Rr_Dr_Vent_Mode;
	L4_DataBuf->data.L_EVNT_RrDrEvntConsOpnCls = Hu_Rr_Dr_Vent_signal;
}

void Set_LinToCan_Disp_RrPs_Mode(Lvnt_4_data *L4_DataBuf)
{
	L4_DataBuf->data.L_EVNT_RrPsEvntModeSet = (uint64_t)Hu_Rr_Ps_Vent_Mode;
	L4_DataBuf->data.L_EVNT_RrPsEvntConsOpnCls = Hu_Rr_Ps_Vent_signal;
}
#endif

