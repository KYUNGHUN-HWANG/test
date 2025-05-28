#define AMO_CYCLE_C_

#include <string.h>
#include "Cpu.h"
#include "Amo_main.h"
#include "flexcan_driver.h"
#include "Amo_STATE_Event.h"
#include "Amo_Timer.h"
#include "Amo_CAN_Parsing.h"
#include "Amo_Cycle.h"


extern uint8_t Tx_candata[8];

volatile bool SLLR_OP_check = FALSE;
volatile bool SLUD_OP_check = FALSE;
volatile bool CLLR_OP_check = FALSE;
volatile bool CLUD_OP_check = FALSE;
volatile bool SRLR_OP_check = FALSE;
volatile bool SRUD_OP_check = FALSE;
volatile bool CRLR_OP_check = FALSE;
volatile bool CRUD_OP_check = FALSE;

extern Evnt_mode Hu_Fr_Dr_Vent_Mode;
extern Evnt_mode Hu_Fr_Ps_Vent_Mode;


volatile static uint8_t op_num = TRUE;

int isFirstCall = 1;



void lin_FrDrPsCycleMode_task(void)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	isFirstCall = 1;
	lin_FrDrPsSwingData_Scenario();

	//Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
	//Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_38);
	Amo_timer_Start(timer_38, 50, true, Set_LinToCan_Disp_FrDrCycle_XY);
	Amo_timer_Stop(timer_39);
	Amo_timer_Start(timer_39, 50, true, Set_LinToCan_Disp_FrPsCycle_XY);
}


void lin_FrDrPsCycleMode_task_Backup(void)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	isFirstCall = 1;
	lin_FrDrPsSwingData_Scenario();

	Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
	Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_38);
	Amo_timer_Start(timer_38, 50, true, Set_LinToCan_Disp_FrDrCycle_XY);
	Amo_timer_Stop(timer_39);
	Amo_timer_Start(timer_39, 50, true, Set_LinToCan_Disp_FrPsCycle_XY);
}


void lin_FrDrPsSwingData_Scenario(void)
{
   static int scenario = 1;

   if (isFirstCall == 1) {
     scenario = 1;
		 isFirstCall = 0;
   }
  
  switch(scenario) {
      case 1:
          lin_FrDrPsSwingData(0, 0, 0, 0, 0, 0, 0, 0);
          scenario = 2;
          break;
          
      case 2:
    	  SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_3);
    	  SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_3);
          lin_FrDrPsSwingData((uint16_t)Min_LR, 0, (uint16_t)Max_LR, 0, (uint16_t)Max_LR, 0, (uint16_t)Min_LR, 0);
          scenario = 3;
          break;
          
      case 3:
          lin_FrDrPsSwingData((uint16_t)Min_LR, (uint16_t)Max_UD, (uint16_t)Max_LR, (uint16_t)Max_UD, (uint16_t)Max_LR, (uint16_t)Min_UD, (uint16_t)Min_LR, (uint16_t)Min_UD);
          scenario = 4;
          break;
          
      case 4:
          lin_FrDrPsSwingData((uint16_t)Max_LR, (uint16_t)Max_UD, (uint16_t)Min_LR, (uint16_t)Max_UD, (uint16_t)Min_LR, (uint16_t)Min_UD, (uint16_t)Max_LR, (uint16_t)Min_UD);
          scenario = 5;
          break;
          
      case 5:
          lin_FrDrPsSwingData((uint16_t)Max_LR, (uint16_t)Min_UD, (uint16_t)Min_LR, (uint16_t)Min_UD, (uint16_t)Min_LR, (uint16_t)Max_UD, (uint16_t)Max_LR, (uint16_t)Max_UD);
          scenario = 6;
          break;
          
      case 6:
          lin_FrDrPsSwingData((uint16_t)Min_LR, (uint16_t)Min_UD, (uint16_t)Max_LR, (uint16_t)Min_UD, (uint16_t)Max_LR, (uint16_t)Max_UD, (uint16_t)Min_LR, (uint16_t)Max_UD);
          scenario = 3;  
          break;
					
			default:
				/* default state */
				break;
  }
}



void lin_FrDrPsSwingData(uint16_t LH_Side_LR, uint16_t LH_Side_UD, uint16_t LH_Ctr_LR, uint16_t LH_Ctr_UD, uint16_t RH_Side_LR, uint16_t RH_Side_UD, uint16_t RH_Ctr_LR, uint16_t RH_Ctr_UD)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	if(ATCU_DriverSideType == LHD)
	{
		if(Hu_Frdr_Vent_Side_sig_backup != 0x02)
		{
				FrDr_SetLinTargetPosition_side(LH_Side_LR, LH_Side_UD);
		}
		if(Hu_Frdr_Vent_Center_sig_backup != 0x02)
		{
				FrDr_SetLinTargetPosition_ctr(LH_Ctr_LR, LH_Ctr_UD);
		}
		if(Hu_FrPs_Vent_Side_sig_backup != 0x02)
		{
				FrPs_SetLinTargetPosition_side(RH_Side_LR, RH_Side_UD);
		}
		if(Hu_FrPs_Vent_Center_sig_backup != 0x02)
		{
				FrPs_SetLinTargetPosition_ctr(RH_Ctr_LR, RH_Ctr_UD);
		}
	}
	if(ATCU_DriverSideType == RHD)
	{
		if(Hu_FrPs_Vent_Side_sig_backup != 0x02)
		{
				FrDr_SetLinTargetPosition_side(LH_Side_LR, LH_Side_UD);
		}
		if(Hu_FrPs_Vent_Center_sig_backup != 0x02)
		{
				FrDr_SetLinTargetPosition_ctr(LH_Ctr_LR, LH_Ctr_UD);
		}
		if(Hu_Frdr_Vent_Side_sig_backup != 0x02)
		{
				FrPs_SetLinTargetPosition_side(RH_Side_LR, RH_Side_UD);
		}
		if(Hu_Frdr_Vent_Center_sig_backup != 0x02)
		{
				FrPs_SetLinTargetPosition_ctr(RH_Ctr_LR, RH_Ctr_UD);
		}
	}

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_70);
	Amo_timer_Start(timer_70, 1000, false, OpDone_Check);
}


void OpDone_Check(void)
{
	if(FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag == true) l_bool_wr_LI1_FR_SRLR_OpDone(op_num);
	if(FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag == true) l_bool_wr_LI1_FR_SRUD_OpDone(op_num);
	if(FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag == true) l_bool_wr_LI1_FR_CRLR_OpDone(op_num);
	if(FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag == true) l_bool_wr_LI1_FR_CRUD_OpDone(op_num);

	if(FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag == true) l_bool_wr_LI0_FR_SLLR_OpDone(op_num);
	if(FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag == true) l_bool_wr_LI0_FR_SLUD_OpDone(op_num);
	if(FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag == true) l_bool_wr_LI0_FR_CLLR_OpDone(op_num);
	if(FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag == true) l_bool_wr_LI0_FR_CLUD_OpDone(op_num);

	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();
	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();
	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_71);
	Amo_timer_Start(timer_71, 50, false, OpDone_Check);

	if((SLLR_OP_check == 1) && (SLUD_OP_check == 1) && (CLLR_OP_check == 1) && (CLUD_OP_check == 1) && \
		(SRLR_OP_check == 1) && (SRUD_OP_check == 1) && (CRLR_OP_check == 1) && (CRUD_OP_check == 1))
	{
		lin_FrDrPsSwingData_Scenario();
	}

}



void lin_FrDrCycleMode_task(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();
	
	isFirstCall = 1;
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
//	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_3); //test hkh

	lin_FrDrSwingData_Scenario();
	//Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_38);
	Amo_timer_Start(timer_38, 50, true, Set_LinToCan_Disp_FrDrCycle_XY);
}

void lin_FrDrCycleMode_task_Backup(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	isFirstCall = 1;
	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_10);
//	SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_3); //test hkh

	lin_FrDrSwingData_Scenario();
	Set_LinToCan_Disp_FrDr_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_38);
	Amo_timer_Start(timer_38, 50, true, Set_LinToCan_Disp_FrDrCycle_XY);
}


void lin_FrDrSwingData_Scenario(void)
{
  static int scenario = 1;

	 if (isFirstCall == 1) {
  	 scenario = 1;
		 isFirstCall = 0;
   }
  
  switch(scenario) {
      case 1:
          lin_FrDrSwingData(0, 0, 0, 0);
          scenario = 2;
          break;
          
      case 2:
					SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_3);
          lin_FrDrSwingData((uint16_t)Min_LR, 0, (uint16_t)Max_LR, 0);
          scenario = 3;
          break;
          
      case 3:
          lin_FrDrSwingData((uint16_t)Min_LR, (uint16_t)Max_UD, (uint16_t)Max_LR, (uint16_t)Max_UD);
          scenario = 4;
          break;
          
      case 4:
          lin_FrDrSwingData((uint16_t)Max_LR, (uint16_t)Max_UD, (uint16_t)Min_LR, (uint16_t)Max_UD);
          scenario = 5;
          break;
          
      case 5:
          lin_FrDrSwingData((uint16_t)Max_LR, (uint16_t)Min_UD, (uint16_t)Min_LR, (uint16_t)Min_UD);
          scenario = 6;
          break;
          
      case 6:
          lin_FrDrSwingData((uint16_t)Min_LR, (uint16_t)Min_UD, (uint16_t)Max_LR, (uint16_t)Min_UD);
          scenario = 3;  
          break;
					
			default:
				/* default state */
				break;
  }
}


void lin_FrDrSwingData(uint16_t LH_Side_LR, uint16_t LH_Side_UD, uint16_t LH_Ctr_LR, uint16_t LH_Ctr_UD)
{
	lin_FrDrCycleMode_TimerStop();

	if(ATCU_DriverSideType == LHD)
	{
		if(Hu_Frdr_Vent_Side_sig_backup != 0x02)
		{
				FrDr_SetLinTargetPosition_side(LH_Side_LR, LH_Side_UD);
		}
		if(Hu_Frdr_Vent_Center_sig_backup != 0x02)
		{
				FrDr_SetLinTargetPosition_ctr(LH_Ctr_LR, LH_Ctr_UD);
		}
	}
		if(ATCU_DriverSideType == RHD)
	{
		if(Hu_FrPs_Vent_Side_sig_backup != 0x02)
		{
				FrDr_SetLinTargetPosition_side(LH_Side_LR, LH_Side_UD);
		}
		if(Hu_FrPs_Vent_Center_sig_backup != 0x02)
		{
				FrDr_SetLinTargetPosition_ctr(LH_Ctr_LR, LH_Ctr_UD);
		}
	}

	lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_72);
	Amo_timer_Start(timer_72, 1000, false, OpDone_Check1);
}


void OpDone_Check1(void)
{
	if(FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag == true) l_bool_wr_LI0_FR_SLLR_OpDone(op_num);
	if(FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag == true) l_bool_wr_LI0_FR_SLUD_OpDone(op_num);
	if(FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag == true) l_bool_wr_LI0_FR_CLLR_OpDone(op_num);
	if(FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag == true) l_bool_wr_LI0_FR_CLUD_OpDone(op_num);

	SLLR_OP_check = l_bool_rd_LI0_FR_SLLR_OpDone();
	SLUD_OP_check = l_bool_rd_LI0_FR_SLUD_OpDone();

	CLLR_OP_check = l_bool_rd_LI0_FR_CLLR_OpDone();
	CLUD_OP_check = l_bool_rd_LI0_FR_CLUD_OpDone();

	Amo_timer_Stop(timer_73);
	Amo_timer_Start(timer_73, 50, false, OpDone_Check1);

	if((SLLR_OP_check == 1) && (SLUD_OP_check == 1) && (CLLR_OP_check == 1) && (CLUD_OP_check == 1))
	{
		lin_FrDrSwingData_Scenario();
	}

}



void lin_FrPsCycleMode_task(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();

	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	isFirstCall = 1;
	lin_FrPsSwingData_Scenario();
	//Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_39);
	Amo_timer_Start(timer_39, 50, true, Set_LinToCan_Disp_FrPsCycle_XY);
}

void lin_FrPsCycleMode_task_Backup(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl)
{
	lin_FrDrCycleMode_TimerStop();
	lin_FrPsCycleMode_TimerStop();
	
	isFirstCall = 1;
	SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_10);

	lin_FrPsSwingData_Scenario();
	Set_LinToCan_Disp_FrPs_Mode(&Can_Tx_Evnt_4);
	Amo_timer_Stop(timer_39);
	Amo_timer_Start(timer_39, 50, true, Set_LinToCan_Disp_FrPsCycle_XY);
}


void lin_FrPsSwingData_Scenario(void)
{
  static int scenario = 1;

	if (isFirstCall == 1) {
  	scenario = 1;
		isFirstCall = 0;
  }
  switch(scenario) {
      case 1:
          lin_FrPsSwingData(0, 0, 0, 0);
          scenario = 2;
          break;
          
      case 2:
					SetLIN_EVNT_FrPsSPEED(&LIN_RH_EVNT_MASTER_CMD, ACT_RPM_3);
          lin_FrPsSwingData((uint16_t)Max_LR, 0, (uint16_t)Min_LR, 0);
          scenario = 3;
          break;
          
      case 3:
          lin_FrPsSwingData((uint16_t)Max_LR, (uint16_t)Min_UD, (uint16_t)Min_LR, (uint16_t)Min_UD);
          scenario = 4;
          break;
          
      case 4:
          lin_FrPsSwingData((uint16_t)Min_LR, (uint16_t)Min_UD, (uint16_t)Max_LR, (uint16_t)Min_UD);
          scenario = 5;
          break;
          
      case 5:
          lin_FrPsSwingData((uint16_t)Min_LR, (uint16_t)Max_UD, (uint16_t)Max_LR, (uint16_t)Max_UD);
          scenario = 6;
          break;
          
      case 6:
          lin_FrPsSwingData((uint16_t)Max_LR, (uint16_t)Max_UD, (uint16_t)Min_LR, (uint16_t)Max_UD);
          scenario = 3;  
          break;

			default:
				/* default state */
				break;
  }
}

void lin_FrPsSwingData(uint16_t RH_Side_LR, uint16_t RH_Side_UD, uint16_t RH_Ctr_LR, uint16_t RH_Ctr_UD)
{
	lin_FrPsCycleMode_TimerStop();

	if(ATCU_DriverSideType == LHD)
	{
		if(Hu_FrPs_Vent_Side_sig_backup != 0x02)
		{
				FrPs_SetLinTargetPosition_side(RH_Side_LR, RH_Side_UD);
		}
		if(Hu_FrPs_Vent_Center_sig_backup != 0x02)
		{
				FrPs_SetLinTargetPosition_ctr(RH_Ctr_LR, RH_Ctr_UD);
		}
	}
	if(ATCU_DriverSideType == RHD)
	{
		if(Hu_Frdr_Vent_Side_sig_backup != 0x02)
		{
				FrPs_SetLinTargetPosition_side(RH_Side_LR, RH_Side_UD);
		}
		if(Hu_Frdr_Vent_Center_sig_backup != 0x02)
		{
				FrPs_SetLinTargetPosition_ctr(RH_Ctr_LR, RH_Ctr_UD);
		}
	}
	
	lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);

	Amo_timer_Stop(timer_74);
	Amo_timer_Start(timer_74, 1000, false, OpDone_Check2);
}


void OpDone_Check2(void)
{
	if(FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag == true) l_bool_wr_LI1_FR_SRLR_OpDone(op_num);
	if(FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag == true) l_bool_wr_LI1_FR_SRUD_OpDone(op_num);
	if(FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag == true) l_bool_wr_LI1_FR_CRLR_OpDone(op_num);
	if(FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag == true) l_bool_wr_LI1_FR_CRUD_OpDone(op_num);

	SRLR_OP_check = l_bool_rd_LI1_FR_SRLR_OpDone();
	SRUD_OP_check = l_bool_rd_LI1_FR_SRUD_OpDone();

	CRLR_OP_check = l_bool_rd_LI1_FR_CRLR_OpDone();
	CRUD_OP_check = l_bool_rd_LI1_FR_CRUD_OpDone();

	Amo_timer_Stop(timer_75);
	Amo_timer_Start(timer_75, 50, false, OpDone_Check2);

	if((SRLR_OP_check == 1) && (SRUD_OP_check == 1) && (CRLR_OP_check == 1) && (CRUD_OP_check == 1))
	{
		lin_FrPsSwingData_Scenario();
	}

}


void lin_FrDrCycleMode_TimerStop(void)
{
	Amo_timer_Stop(timer_70);
	Amo_timer_Stop(timer_71);
	Amo_timer_Stop(timer_72);
	Amo_timer_Stop(timer_73);
}

void lin_FrPsCycleMode_TimerStop(void)
{
	Amo_timer_Stop(timer_70);
	Amo_timer_Stop(timer_71);
	Amo_timer_Stop(timer_74);
	Amo_timer_Stop(timer_75);

}

void lin_CycleMode_CanDisp_TimerStop(void)
{
	Amo_timer_Stop(timer_38);
	Amo_timer_Stop(timer_39);
}

#ifdef AMO_GN7_PE_SETTING_NONE
void lin_RrCycleMode_TimerStop(void)
{
}
#endif


///////////////////////////////////////////////////////////////////////////////////


void Set_LinToCan_Disp_FrDrCycle_XY(void)
{

	uint64_t FR_SLLR_ActuatorState = l_u16_rd_LI0_FR_SLLR_ActuatorState();
	uint64_t FR_SLUD_ActuatorState = l_u16_rd_LI0_FR_SLUD_ActuatorState();
	uint64_t FR_CLLR_ActuatorState = l_u16_rd_LI0_FR_CLLR_ActuatorState();
	uint64_t FR_CLUD_ActuatorState = l_u16_rd_LI0_FR_CLUD_ActuatorState();

	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = SET_CycleFrDrEVntSidePt_X(FR_SLLR_ActuatorState, &Can_data_23_Info);
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = SET_CycleFrDrEVntSidePt_Y(FR_SLUD_ActuatorState, &Can_data_23_Info);
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = SET_CycleFrDrEVntCtrPt_X(FR_CLLR_ActuatorState, &Can_data_23_Info);
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = SET_CycleFrDrEVntCtrPt_Y(FR_CLUD_ActuatorState, &Can_data_23_Info);

	Hu_FrDrEVntSidePt_X_backup = (uint16_t)Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X;
	Hu_FrDrEVntSidePt_Y_backup = (uint16_t)Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y;
	Hu_FrDrEVntCtrPt_X_backup = (uint16_t)Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X;
	Hu_FrDrEVntCtrPt_Y_backup = (uint16_t)Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y;

	Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_X = Hu_FrDrEVntSidePt_X_backup;
	Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_Y = Hu_FrDrEVntSidePt_Y_backup;
	Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_X = Hu_FrDrEVntCtrPt_X_backup;
	Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_Y = Hu_FrDrEVntCtrPt_Y_backup;
	Hu_FrDrEVntSidePt_X_reserve = (uint16_t)Hu_FrDrEVntSidePt_X_backup;
	Hu_FrDrEVntSidePt_Y_reserve = (uint16_t)Hu_FrDrEVntSidePt_Y_backup;
	Hu_FrDrEVntCtrPt_X_reserve = (uint16_t)Hu_FrDrEVntCtrPt_X_backup;
	Hu_FrDrEVntCtrPt_Y_reserve = (uint16_t)Hu_FrDrEVntCtrPt_Y_backup;
	Evt_queue_add(DEVICE_CAN_EVNT_ATCU_SEND_EVENT);
}

uint16_t SET_CycleFrDrEVntSidePt_X(uint64_t lin_Ctrl, const Atcu_23_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_X;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_CycleFrDrEVntSidePt_Y(uint64_t lin_Ctrl, const Atcu_23_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryUp_Y;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntSideBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Reverse_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_CycleFrDrEVntCtrPt_X(uint64_t lin_Ctrl, const Atcu_23_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_X;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_CycleFrDrEVntCtrPt_Y(uint64_t lin_Ctrl, const Atcu_23_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Reverse_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

void Set_LinToCan_Disp_FrPsCycle_XY(void)
{

	uint64_t FR_SRLR_ActuatorState = l_u16_rd_LI1_FR_SRLR_ActuatorState();
	uint64_t FR_SRUD_ActuatorState = l_u16_rd_LI1_FR_SRUD_ActuatorState();
	uint64_t FR_CRLR_ActuatorState = l_u16_rd_LI1_FR_CRLR_ActuatorState();
	uint64_t FR_CRUD_ActuatorState = l_u16_rd_LI1_FR_CRUD_ActuatorState();

	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = SET_CycleFrPsEVntSidePt_X(FR_SRLR_ActuatorState, &Can_data_24_Info);
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = SET_CycleFrPsEVntSidePt_Y(FR_SRUD_ActuatorState, &Can_data_24_Info);
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = SET_CycleFrPsEVntCtrPt_X(FR_CRLR_ActuatorState, &Can_data_24_Info);
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = SET_CycleFrPsEVntCtrPt_Y(FR_CRUD_ActuatorState, &Can_data_24_Info);

	Hu_FrPsEVntSidePt_X_backup = (uint16_t)Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X;
	Hu_FrPsEVntSidePt_Y_backup = (uint16_t)Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y;
	Hu_FrPsEVntCtrPt_X_backup = (uint16_t)Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X;
	Hu_FrPsEVntCtrPt_Y_backup = (uint16_t)Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y;

	Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_X = Hu_FrPsEVntSidePt_X_backup;
	Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_Y = Hu_FrPsEVntSidePt_Y_backup;	
	Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_X = Hu_FrPsEVntCtrPt_X_backup;
	Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_Y = Hu_FrPsEVntCtrPt_Y_backup;	
	Hu_FrPsEVntSidePt_X_reserve = (uint16_t)Hu_FrPsEVntSidePt_X_backup;
	Hu_FrPsEVntSidePt_Y_reserve = (uint16_t)Hu_FrPsEVntSidePt_Y_backup;
	Hu_FrPsEVntCtrPt_X_reserve = (uint16_t)Hu_FrPsEVntCtrPt_X_backup;
	Hu_FrPsEVntCtrPt_Y_reserve = (uint16_t)Hu_FrPsEVntCtrPt_Y_backup;

	Evt_queue_add(DEVICE_CAN_EVNT_ATCU_SEND_EVENT);
}

uint16_t SET_CycleFrPsEVntSidePt_X(uint64_t lin_Ctrl, const Atcu_24_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_X;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_CycleFrPsEVntSidePt_Y(uint64_t lin_Ctrl, const Atcu_24_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryUp_Y;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntSideBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_CycleFrPsEVntCtrPt_X(uint64_t lin_Ctrl, const Atcu_24_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_X;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_X;

	result = Cal_LinToCan_EVntSidePt_X(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

uint16_t SET_CycleFrPsEVntCtrPt_Y(uint64_t lin_Ctrl, const Atcu_24_data *canBdryDataBuf)
{
	uint16_t result;

	const uint64_t TP_temp = lin_Ctrl;
	const uint64_t data_Bdry_Up_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y;
	const uint64_t data_Bdry_Lo_temp = canBdryDataBuf->data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y;

	result = Cal_LinToCan_EVntSidePt_Y(&TP_temp, &data_Bdry_Up_temp, &data_Bdry_Lo_temp);

	return result;
}

