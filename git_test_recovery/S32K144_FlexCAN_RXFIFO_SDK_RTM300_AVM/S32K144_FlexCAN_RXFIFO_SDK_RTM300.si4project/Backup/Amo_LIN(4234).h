
#ifndef Amo_LIN_H_ /* to interprete header file only once */
#define Amo_LIN_H_

#include <stdio.h>

#include "Cpu.h"
#include "Amo_CAN_Parsing.h"
#include "Amo_main.h"

#define ACT_RPM_10 10U
#define ACT_RPM_3	3U
#define ACT_RPM_1	1U
#define ACT_RPM_2	2U

#define LIN_BUSOFF_DELAY	10U // 5s

//#define ACT_ANGLE_UPDATE
//#define ACT_ANGLE_UPDATE_2
#define ACT_ANGLE_UPDATE_3
//#define OLD_ANGLE
//#define LIN_DTC_LINBUSOFF_TEST

#define LIN_BUSOFF_CHECK_DELAY_50MS


/*
#define CAN_FACTOR 10U
#define LIN_RANGE_PT_X 600U
#define LIN_RANGE_PT_Y 520U
*/

#if defined(ACT_ANGLE_UPDATE)
typedef enum _lin_angle{
	FullClose_max = 2047,  //204.7
	FullClose_min = 2047, //204.7
	FullClose_UD = 1150,  //115
	FullClose_UD_Rev = 2946, //-115
	Half_max_LR = 175,  //150,,  //17.5
	Half_min_LR = 3921, //3946, //-17.5
	Max_LR = 350, //300, //35
	Min_LR = 3746, // 3796, //-35
 	Min_UD = 450, //400, //45
	Max_UD = 3646, //3696, //-45
}Lin_angle;
#elif defined(ACT_ANGLE_UPDATE_2)
typedef enum _lin_angle {
	FullClose_max = 2047,  //204.7
	FullClose_min = 2047, //204.7
	FullClose_UD = 1150,  //115
	FullClose_UD_Rev = 2946, //-115
	Half_max_LR = 150,  //175,,  //17.5
	Half_min_LR = 3946, //3921, //-17.5
	FocusSpread_UD = 225, //22.5
	FocusSpread_UD_Rev = 3871, //-22.5
	Max_LR = 350, //300, //35
	Min_LR = 3746, // 3796, //-35
 	Min_UD = 450, //400, //45
	Max_UD = 3646, //3696, //-45
}Lin_angle;
#elif defined(ACT_ANGLE_UPDATE_3)
typedef enum _lin_angle{
	FullClose_max = 2047,  //204.7
	FullClose_min = 2047, //204.7
	FullClose_UD = 1150,  //115
	FullClose_UD_Rev = 2946, //-115
	Half_max_LR = 3946, //3921, //-17.5
	Half_min_LR = 150, //175,,  //17.5
	FocusSpread_UD = 3871, //22.5
	FocusSpread_UD_Rev = 225, //-22.5
 	Min_UD = 450, //400, //45
	Max_UD = 3646, //3696, //-45

	Min_LR = 350, //300, //35
	Max_LR = 3746, // 3796, //-35

	

}Lin_angle;
	
#elif defined(OLD_ANGLE)
typedef enum _lin_angle{
	FullClose_max = 792,  //79.2
	FullClose_SRLR_max = 791, //79.2
	FullClose_min = 1256, //-79.2
	FullClose_UD = 1023,  //102.3
	FullClose_UD_Rev = 1023,
	Half_max_LR = 150,  //15
	Half_min_LR = 1898, //-15
	Max_LR = 290,  //29  //30 300
	Min_LR = 1758, //-29 //-30 1748
 	Min_UD = 240,  //24  //26 260
	Max_UD = 1808, //-24 //-26 1788
}Lin_angle;
#endif



typedef enum
{
	FrDrSidePt_X = 0,
	FrDrSidePt_Y,
	FrDrCtrPt_X,
	FrDrCtrPt_Y,
	FrPsSidePt_X,
	FrPsSidePt_Y,
	FrPsCtrPt_X,
	FrPsCtrPt_Y,
	#ifdef AMO_GN7_PE_SETTING_NONE		
	RrDrCons_X,
	RrDrCons_Y,
	RrPsCons_X,
	RrPsCons_Y
	#endif
}CanLin_Position_XY_t;


typedef enum
{
	FR_SLLR_STATUS = 0x12,
	FR_SLUD_STATUS = 0x13,
	FR_CLLR_STATUS = 0x14,
	FR_CLUD_STATUS = 0x15,
	
	FR_CRLR_STATUS = 0x22,
	FR_CRUD_STATUS = 0x23,
	FR_SRLR_STATUS = 0x24,
	FR_SRUD_STATUS = 0x25,
	
}Evnt_Status_Frame_ID_t;


typedef struct CanLin_Pt_X_tag
{
	int16_t Can_RangePt_X;
	int16_t Can_EVntPt_X;
	int16_t Can_RangePt_Center_X;
	int16_t CanData_Pt_X;
	int16_t linFactor_Pt_X;
	int16_t LinData_Pt_X;
}CanLin_Pt_X_t;

typedef struct CanLin_Pt_Y_tag
{
	int16_t Can_RangePt_Y;
	int16_t Can_EVntPt_Y;
	int16_t Can_RangePt_Center_Y;
	int16_t CanData_Pt_Y;
	int16_t linFactor_Pt_Y;
	int16_t LinData_Pt_Y;
}CanLin_Pt_Y_t;


typedef struct CanLinValue_tag
{
	CanLin_Pt_X_t Pt_X;
	CanLin_Pt_Y_t Pt_Y;

}CanLinValue_t;


typedef struct SelectEVnt_tag
{
	int16_t selectEVnt;
	int16_t selectEVnt1;
}SelectEVnt_t;


typedef struct CanLin_FrDrEVntPt_XY_tag
{
	uint64_t Hu_FrDrEVntSidePt_X_oldVal;
	uint64_t Hu_FrDrEVntSidePt_X_newVal;
	uint64_t Hu_FrDrEVntSidePt_Y_oldVal;
	uint64_t Hu_FrDrEVntSidePt_Y_newVal;
	uint64_t Hu_FrDrEVntCtrPt_X_oldVal;
	uint64_t Hu_FrDrEVntCtrPt_X_newVal;
	uint64_t Hu_FrDrEVntCtrPt_Y_oldVal;
	uint64_t Hu_FrDrEVntCtrPt_Y_newVal;
}CanLin_FrDrEVntPt_XY_t;

typedef struct CanLin_FrPsEVntPt_XY_tag
{
	uint64_t Hu_FrPsEVntSidePt_X_oldVal;
	uint64_t Hu_FrPsEVntSidePt_X_newVal;
	uint64_t Hu_FrPsEVntSidePt_Y_oldVal;
	uint64_t Hu_FrPsEVntSidePt_Y_newVal;
	uint64_t Hu_FrPsEVntCtrPt_X_oldVal;
	uint64_t Hu_FrPsEVntCtrPt_X_newVal;
	uint64_t Hu_FrPsEVntCtrPt_Y_oldVal;
	uint64_t Hu_FrPsEVntCtrPt_Y_newVal;
}CanLin_FrPsEVntPt_XY_t;

#ifdef AMO_GN7_PE_SETTING_NONE		
typedef struct CanLin_RrEVntPt_XY_tag
{
	uint64_t Hu_RrDrEVntConsPt_X_oldVal;
	uint64_t Hu_RrDrEVntConsPt_X_newVal;
	uint64_t Hu_RrDrEVntConsPt_Y_oldVal;
	uint64_t Hu_RrDrEVntConsPt_Y_newVal;
	uint64_t Hu_RrPsEVntConsPt_X_oldVal;
	uint64_t Hu_RrPsEVntConsPt_X_newVal;
	uint64_t Hu_RrPsEVntConsPt_Y_oldVal;
	uint64_t Hu_RrPsEVntConsPt_Y_newVal;
}CanLin_RrEVntPt_XY_t;
#endif

typedef union
{
	struct FR_DR_SD_Fault_t_
	{
		uint8_t FR_SLLR_ComFlt_Flag : 1;
		uint8_t FR_SLLR_IntFlt_Flag : 1;
		uint8_t FR_SLLR_ExtFlt_Flag : 1;
		uint8_t FR_SLUD_ComFlt_Flag : 1;
		uint8_t FR_SLUD_IntFlt_Flag : 1;
		uint8_t FR_SLUD_ExtFlt_Flag : 1;
		uint8_t dummy : 2;
	} fltBit;
	uint8_t fltByte;
	
}FR_DR_SD_Fault_t;


typedef union
{
	struct FR_DR_CTR_Fault_t_
	{
		uint8_t FR_CLLR_ComFlt_Flag : 1;
		uint8_t FR_CLLR_IntFlt_Flag : 1;
		uint8_t FR_CLLR_ExtFlt_Flag : 1;
		uint8_t FR_CLUD_ComFlt_Flag : 1;
		uint8_t FR_CLUD_IntFlt_Flag : 1;
		uint8_t FR_CLUD_ExtFlt_Flag : 1;
		uint8_t dummy : 2;
	} fltBit;
	uint8_t fltByte;
	
}FR_DR_CTR_Fault_t;

typedef union
{
	struct FR_PS_CTR_Fault_t_
	{
		uint8_t FR_CRLR_ComFlt_Flag : 1;
		uint8_t FR_CRLR_IntFlt_Flag : 1;
		uint8_t FR_CRLR_ExtFlt_Flag : 1;
		uint8_t FR_CRUD_ComFlt_Flag : 1;
		uint8_t FR_CRUD_IntFlt_Flag : 1;
		uint8_t FR_CRUD_ExtFlt_Flag : 1;
		uint8_t dummy : 2;
	} fltBit;
	uint8_t fltByte;
	
}FR_PS_CTR_Fault_t;

typedef union
{
	struct FR_PS_SD_Fault_t_
	{
		uint8_t FR_SRLR_ComFlt_Flag : 1;
		uint8_t FR_SRLR_IntFlt_Flag : 1;
		uint8_t FR_SRLR_ExtFlt_Flag : 1;
		uint8_t FR_SRUD_ComFlt_Flag : 1;
		uint8_t FR_SRUD_IntFlt_Flag : 1;
		uint8_t FR_SRUD_ExtFlt_Flag : 1;
		uint8_t dummy : 2;
	} fltBit;
	uint8_t fltByte;
	
}FR_PS_SD_Fault_t;

typedef union
{
	struct FR_Ext_Fault_t_
	{
		uint8_t FR_SLLR_ExtFlt : 1;
		uint8_t FR_SLUD_ExtFlt : 1;
		uint8_t FR_CLLR_ExtFlt : 1;
		uint8_t FR_CLUD_ExtFlt : 1;
		uint8_t FR_CRLR_ExtFlt : 1;
		uint8_t FR_CRUD_ExtFlt : 1;
		uint8_t FR_SRLR_ExtFlt : 1;
		uint8_t FR_SRUD_ExtFlt : 1;
	} fltBit;
	uint8_t fltByte;
	
}FR_Ext_Fault_t;



typedef struct
{
	FR_DR_SD_Fault_t FR_DR_SD_FLT;
	FR_DR_CTR_Fault_t FR_DR_CTR_FLT;
	FR_PS_CTR_Fault_t FR_PS_CTR_FLT;
	FR_PS_SD_Fault_t FR_PS_SD_FLT;
	FR_Ext_Fault_t FR_EXT_FLT;
}FR_DTC_Flt_flag_t;


typedef struct
{
	uint16_t FR_SLLR_ComFlt_Flag : 1;
	uint16_t FR_SLUD_ComFlt_Flag : 1;
	uint16_t FR_CLLR_ComFlt_Flag : 1;
	uint16_t FR_CLUD_ComFlt_Flag : 1;

	uint16_t FR_CRLR_ComFlt_Flag : 1;
	uint16_t FR_CRUD_ComFlt_Flag : 1;
	uint16_t FR_SRLR_ComFlt_Flag : 1;
	uint16_t FR_SRUD_ComFlt_Flag : 1;

	uint16_t dummy : 8;
}FR_ComFault_t;


typedef struct
{
	uint16_t FR_SLLR_IntFlt_Flag : 1;
	uint16_t FR_SLUD_IntFlt_Flag : 1;
	uint16_t FR_CLLR_IntFlt_Flag : 1;
	uint16_t FR_CLUD_IntFlt_Flag : 1;

	uint16_t FR_CRLR_IntFlt_Flag : 1;
	uint16_t FR_CRUD_IntFlt_Flag : 1;
	uint16_t FR_SRLR_IntFlt_Flag : 1;
	uint16_t FR_SRUD_IntFlt_Flag : 1;

	uint16_t dummy : 8;
}FR_IntFault_t;

typedef struct
{
	uint16_t FR_SLLR_ExtFlt_Flag : 1;
	uint16_t FR_SLUD_ExtFlt_Flag : 1;
	uint16_t FR_CLLR_ExtFlt_Flag : 1;
	uint16_t FR_CLUD_ExtFlt_Flag : 1;
	uint16_t FR_CRLR_ExtFlt_Flag : 1;
	uint16_t FR_CRUD_ExtFlt_Flag : 1;
	uint16_t FR_SRLR_ExtFlt_Flag : 1;
	uint16_t FR_SRUD_ExtFlt_Flag : 1;

	uint16_t dummy : 8;
}FR_ExtFault_t;


typedef struct
{	
	uint16_t FR_SLLR_LIN_BusOff : 1;
	uint16_t FR_SLUD_LIN_BusOff : 1;
	uint16_t FR_CLLR_LIN_BusOff : 1;
	uint16_t FR_CLUD_LIN_BusOff : 1;

	uint16_t FR_CRLR_LIN_BusOff : 1;
	uint16_t FR_CRUD_LIN_BusOff : 1;
	uint16_t FR_SRLR_LIN_BusOff : 1;
	uint16_t FR_SRUD_LIN_BusOff : 1;
	
	uint16_t dummy : 8;
}FR_LinBusOff_flag_t;


typedef struct CanLin_oldNewValue_check_tag
{
	CanLin_FrDrEVntPt_XY_t FrDrPt;
	CanLin_FrPsEVntPt_XY_t FrPsPt;
#ifdef AMO_GN7_PE_SETTING_NONE
	CanLin_RrEVntPt_XY_t RrPt;
#endif
}CanLin_oldNewValue_check_t;


typedef struct 
{
	uint8_t	linEvntAddr_Flag;

}LIN_flag_t;


typedef struct
{
	uint16_t currentPt_FR_SLLR;
	uint16_t prevPt_FR_SLLR;
	uint16_t currentPt_FR_SLUD;
	uint16_t prevPt_FR_SLUD;
	uint16_t currentPt_FR_CLLR;
	uint16_t prevPt_FR_CLLR;
	uint16_t currentPt_FR_CLUD;
	uint16_t prevPt_FR_CLUD;

}Lin_FrDrPt_t;

typedef struct
{
	int16_t deltaPt_FR_SLLR;
	int16_t absDeltaPt_FR_SLLR;
	int16_t deltaPt_FR_SLUD;
	int16_t absDeltaPt_FR_SLUD;
	int16_t deltaPt_FR_CLLR;
	int16_t absDeltaPt_FR_CLLR;
	int16_t deltaPt_FR_CLUD;
	int16_t absDeltaPt_FR_CLUD;
}Lin_FrDrPt_Delta_t;

typedef struct
{
	uint16_t currentPt_FR_CRLR;
	uint16_t prevPt_FR_CRLR;
	uint16_t currentPt_FR_CRUD;
	uint16_t prevPt_FR_CRUD;
	uint16_t currentPt_FR_SRLR;
	uint16_t prevPt_FR_SRLR;
	uint16_t currentPt_FR_SRUD;
	uint16_t prevPt_FR_SRUD;
}Lin_FrPsPt_t;

typedef struct
{
	int16_t deltaPt_FR_CRLR;
	int16_t absDeltaPt_FR_CRLR;
	int16_t deltaPt_FR_CRUD;
	int16_t absDeltaPt_FR_CRUD;
	int16_t deltaPt_FR_SRLR;
	int16_t absDeltaPt_FR_SRLR;
	int16_t deltaPt_FR_SRUD;
	int16_t absDeltaPt_FR_SRUD;
}Lin_FrPsPt_Delta_t;

#ifdef AMO_GN7_PE_SETTING_NONE		
typedef struct
{
	int16_t currentPt_RR_CLLR;
	int16_t prevPt_RR_CLLR;
	int16_t currentPt_RR_CLUD;
	int16_t prevPt_RR_CLUD;
	int16_t currentPt_RR_CRLR;
	int16_t prevPt_RR_CRLR;
	int16_t currentPt_RR_CRUD;
	int16_t prevPt_RR_CRUD;

}Lin_RrPt_t;

typedef struct
{
	int16_t deltaPt_RR_CLLR;
	int16_t absDeltaPt_RR_CLLR;
	int16_t deltaPt_RR_CLUD;
	int16_t absDeltaPt_RR_CLUD;
	int16_t deltaPt_RR_CRLR;
	int16_t absDeltaPt_RR_CRLR;
	int16_t deltaPt_RR_CRUD;
	int16_t absDeltaPt_RR_CRUD;
}Lin_RrPt_Delta_t;
#endif

typedef struct
{
	Lin_FrDrPt_t FrDrPt;
	Lin_FrDrPt_Delta_t FrDrDeltaPt;
	Lin_FrPsPt_t FrPsPt;
	Lin_FrPsPt_Delta_t FrPsDeltaPt;
#ifdef AMO_GN7_PE_SETTING_NONE
	Lin_RrPt_t RrPt;
	Lin_RrPt_Delta_t RrDeltaPt;
#endif
}Lin_EVntPt_t;

typedef struct
{
	uint8_t FR_SL_Mode_Flag : 1;
	uint8_t FR_CL_Mode_Flag : 1;
	uint8_t FR_CR_Mode_Flag : 1;
	uint8_t FR_SR_Mode_Flag : 1;
	uint8_t reserved17 : 4;
}EvntMode_flag_t;


typedef union  
{
  struct lin_LH_EVNT_MASTER_CMD_t
  {
  	uint16_t	EVNT_ADDr: 7;
		uint16_t	EVNT_Broad: 1;
		uint16_t	EVNT_SPEED: 4;
		uint16_t	EVNT_Front_Side_LH_LeftRight_TargetPosition: 12;
		uint16_t	EVNT_Front_Side_LH_UpDown_TargetPosition: 12;	
		uint16_t	EVNT_Front_Center_LH_LeftRight_TargetPosition: 12;
		uint16_t	EVNT_Front_Center_LH_UpDown_TargetPosition: 12;
		uint16_t	dummy: 12;
  } LDATA;
  uint8_t byte[9];
  
}lin_LH_EVNT_MASTER_CMD_11;

typedef union  
{
  struct FR_SLLR_STATUS_t
  {
  	uint32_t	FR_SLLR_SPEED: 4;
		uint32_t	FR_SLLR_ActuatorState: 12;
		uint32_t	FR_SLLR_OpDone: 1;
		uint32_t	FR_SLLR_VddReset: 1;
		uint32_t	FR_SLLR_Steploss: 1;
		uint32_t	FR_SLLR_EIDef: 1;
		uint32_t	FR_SLLR_TSD: 1;
		uint32_t	FR_SLLR_TW: 1;
		uint32_t	FR_SLLR_Tinfo: 2;
		uint32_t	FR_SLLR_STATE: 1;
		uint32_t	FR_SLLR_Sensor: 1;
		uint32_t	FR_SLLR_OV: 1;
		uint32_t	FR_SLLR_UV: 1;
		uint32_t	FR_SLLR_OPEN2: 1;
		uint32_t	FR_SLLR_OPEN1: 1;
		uint32_t	FR_SLLR_OVC2: 1;
		uint32_t	FR_SLLR_OVC1: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_SLLR_STATUS_12;


typedef union  
{
  struct FR_SLUD_STATUS_t
  {
		uint32_t	FR_SLUD_SPEED: 4;
		uint32_t	FR_SLUD_ActuatorState: 12;
		uint32_t	FR_SLUD_OpDone: 1;
		uint32_t	FR_SLUD_VddReset: 1;
		uint32_t	FR_SLUD_Steploss: 1;
		uint32_t	FR_SLUD_EIDef: 1;
		uint32_t	FR_SLUD_TSD: 1;
		uint32_t	FR_SLUD_TW: 1;
		uint32_t	FR_SLUD_Tinfo: 2;
		uint32_t	FR_SLUD_STATE: 1;
		uint32_t	FR_SLUD_Sensor: 1;
		uint32_t	FR_SLUD_OV: 1;
		uint32_t	FR_SLUD_UV: 1;
		uint32_t	FR_SLUD_OPEN2: 1;
		uint32_t	FR_SLUD_OPEN1: 1;
		uint32_t	FR_SLUD_OVC2: 1;
		uint32_t	FR_SLUD_OVC1: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_SLUD_STATUS_13;

typedef union  
{
  struct FR_CLLR_STATUS_t
  {
		uint32_t	FR_CLLR_SPEED: 4;
		uint32_t	FR_CLLR_ActuatorState: 12;
		uint32_t	FR_CLLR_OpDone: 1;
		uint32_t	FR_CLLR_VddReset: 1;
		uint32_t	FR_CLLR_Steploss: 1;
		uint32_t	FR_CLLR_EIDef: 1;
		uint32_t	FR_CLLR_TSD: 1;
		uint32_t	FR_CLLR_TW: 1;
		uint32_t	FR_CLLR_Tinfo: 2;
		uint32_t	FR_CLLR_STATE: 1;
		uint32_t	FR_CLLR_Sensor: 1;
		uint32_t	FR_CLLR_OV: 1;
		uint32_t	FR_CLLR_UV: 1;
		uint32_t	FR_CLLR_OPEN2: 1;
		uint32_t	FR_CLLR_OPEN1: 1;
		uint32_t	FR_CLLR_OVC2: 1;
		uint32_t	FR_CLLR_OVC1: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_CLLR_STATUS_14;

typedef union  
{
  struct FR_CLUD_STATUS_t
  {
		uint32_t	FR_CLUD_SPEED: 4;
		uint32_t	FR_CLUD_ActuatorState: 12;
		uint32_t	FR_CLUD_OpDone: 1;
		uint32_t	FR_CLUD_VddReset: 1;
		uint32_t	FR_CLUD_Steploss: 1;
		uint32_t	FR_CLUD_EIDef: 1;
		uint32_t	FR_CLUD_TSD: 1;
		uint32_t	FR_CLUD_TW: 1;
		uint32_t	FR_CLUD_Tinfo: 2;
		uint32_t	FR_CLUD_STATE: 1;
		uint32_t	FR_CLUD_Sensor: 1;
		uint32_t	FR_CLUD_OV: 1;
		uint32_t	FR_CLUD_UV: 1;
		uint32_t	FR_CLUD_OPEN2: 1;
		uint32_t	FR_CLUD_OPEN1: 1;
		uint32_t	FR_CLUD_OVC2: 1;
		uint32_t	FR_CLUD_OVC1: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_CLUD_STATUS_15;


typedef union  
{
  struct lin_LH_EVNT_SPECIAL_CMD_t
  {
  	uint16_t	EVNT_CMD: 8;
		uint16_t	EVNT_Broad: 1;
		uint16_t	EVNT_ADDr: 7;
  } LDATA;
  uint8_t byte[2];
  
}lin_LH_EVNT_SPECIAL_CMD_0;


typedef union  
{
  struct lin_RH_EVNT_MASTER_CMD_t
  {
  	uint16_t	EVNT_ADDr: 7;
		uint16_t	EVNT_Broad: 1;
		uint16_t	EVNT_SPEED: 4;
		uint16_t	EVNT_Front_Center_RH_LeftRight_TargetPosition: 12;
		uint16_t	EVNT_Front_Center_RH_UpDown_TargetPosition: 12;
		uint16_t	EVNT_Front_Side_RH_LeftRight_TargetPosition: 12;
		uint16_t	EVNT_Front_Side_RH_UpDown_TargetPosition: 12;	
		uint16_t	dummy: 12;
  } LDATA;
  uint8_t byte[9];
  
}lin_RH_EVNT_MASTER_CMD_21;


typedef union  
{
  struct FR_CRLR_STATUS_t
  {
  	uint32_t	FR_CRLR_SPEED: 4;
		uint32_t	FR_CRLR_ActuatorState: 12;
		uint32_t	FR_CRLR_OpDone: 1;
		uint32_t	FR_CRLR_VddReset: 1;
		uint32_t	FR_CRLR_Steploss: 1;
		uint32_t	FR_CRLR_EIDef: 1;
		uint32_t	FR_CRLR_TSD: 1;
		uint32_t	FR_CRLR_TW: 1;
		uint32_t	FR_CRLR_Tinfo: 2;
		uint32_t	FR_CRLR_STATE: 1;
		uint32_t	FR_CRLR_Sensor: 1;
		uint32_t	FR_CRLR_OV: 1;
		uint32_t	FR_CRLR_UV: 1;
		uint32_t	FR_CRLR_OPEN2: 1;
		uint32_t	FR_CRLR_OPEN1: 1;
		uint32_t	FR_CRLR_OVC2: 1;
		uint32_t	FR_CRLR_OVC1: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_CRLR_STATUS_22;


typedef union  
{
  struct FR_CRUD_STATUS_t
  {
		uint32_t	FR_CRUD_SPEED: 4;
		uint32_t	FR_CRUD_ActuatorState: 12;
		uint32_t	FR_CRUD_OpDone: 1;
		uint32_t	FR_CRUD_VddReset: 1;
		uint32_t	FR_CRUD_Steploss: 1;
		uint32_t	FR_CRUD_EIDef: 1;
		uint32_t	FR_CRUD_TSD: 1;
		uint32_t	FR_CRUD_TW: 1;
		uint32_t	FR_CRUD_Tinfo: 2;
		uint32_t	FR_CRUD_STATE: 1;
		uint32_t	FR_CRUD_Sensor: 1;
		uint32_t	FR_CRUD_OV: 1;
		uint32_t	FR_CRUD_UV: 1;
		uint32_t	FR_CRUD_OPEN2: 1;
		uint32_t	FR_CRUD_OPEN1: 1;
		uint32_t	FR_CRUD_OVC2: 1;
		uint32_t	FR_CRUD_OVC1: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_CRUD_STATUS_23;

typedef union  
{
  struct FR_SRLR_STATUS_t
  {
		uint32_t	FR_SRLR_SPEED: 4;
		uint32_t	FR_SRLR_ActuatorState: 12;
		uint32_t	FR_SRLR_OpDone: 1;
		uint32_t	FR_SRLR_VddReset: 1;
		uint32_t	FR_SRLR_Steploss: 1;
		uint32_t	FR_SRLR_EIDef: 1;
		uint32_t	FR_SRLR_TSD: 1;
		uint32_t	FR_SRLR_TW: 1;
		uint32_t	FR_SRLR_Tinfo: 2;
		uint32_t	FR_SRLR_STATE: 1;
		uint32_t	FR_SRLR_Sensor: 1;
		uint32_t	FR_SRLR_OV: 1;
		uint32_t	FR_SRLR_UV: 1;
		uint32_t	FR_SRLR_OPEN2: 1;
		uint32_t	FR_SRLR_OPEN1: 1;
		uint32_t	FR_SRLR_OVC2: 1;
		uint32_t	FR_SRLR_OVC1: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_SRLR_STATUS_24;

typedef union  
{
  struct FR_SRUD_STATUS_t
  {
		uint32_t	FR_SRUD_SPEED: 4;
		uint32_t	FR_SRUD_ActuatorState: 12;
		uint32_t	FR_SRUD_OpDone: 1;
		uint32_t	FR_SRUD_VddReset: 1;
		uint32_t	FR_SRUD_Steploss: 1;
		uint32_t	FR_SRUD_EIDef: 1;
		uint32_t	FR_SRUD_TSD: 1;
		uint32_t	FR_SRUD_TW: 1;
		uint32_t	FR_SRUD_Tinfo: 2;
		uint32_t	FR_SRUD_STATE: 1;
		uint32_t	FR_SRUD_Sensor: 1;
		uint32_t	FR_SRUD_OV: 1;
		uint32_t	FR_SRUD_UV: 1;
		uint32_t	FR_SRUD_OPEN2: 1;
		uint32_t	FR_SRUD_OPEN1: 1;
		uint32_t	FR_SRUD_OVC2: 1;
		uint32_t	FR_SRUD_OVC1: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_SRUD_STATUS_25;

typedef union  
{
  struct lin_RH_EVNT_SPECIAL_CMD_t
  {
  	uint16_t	EVNT_CMD: 8;
		uint16_t	EVNT_Broad: 1;
		uint16_t	EVNT_ADDr: 7;
  } LDATA;
  uint8_t byte[2];
  
}lin_RH_EVNT_SPECIAL_CMD_0;


#ifdef AMO_GN7_PE_SETTING_NONE		
typedef union  
{
  struct lin_RC_EVNT_MASTER_CMD_t
  {
  	uint64_t	EVNT_ADDr: 7;
		uint64_t	EVNT_Broad: 1;
		uint64_t	EVNT_REAR_RH_SPEED: 4;
		uint64_t	EVNT_REAR_LH_SPEED: 4;
		uint64_t	EVNT_Rear_Center_LH_LeftRight_TargetPosition: 12;
		uint64_t	EVNT_Rear_Center_LH_UpDown_TargetPosition: 12;	
		uint64_t	EVNT_Rear_Center_RH_LeftRight_TargetPosition: 12;
		uint64_t	EVNT_Rear_Center_RH_UpDown_TargetPosition: 12;
		uint64_t	dummy: 12;
  } LDATA;
  uint8_t byte[9];
  
}lin_RC_EVNT_MASTER_CMD_31;


typedef union  
{
  struct RR_CLLR_STATUS_t
  {
  	uint32_t	RR_CLLR_SPEED: 4;
		uint32_t	RR_CLLR_ActuatorState: 12;
		uint32_t	RR_CLLR_OpDone: 1;
		uint32_t	RR_CLLR_VddReset: 1;
		uint32_t	RR_CLLR_Steploss: 1;
		uint32_t	RR_CLLR_EIDef: 1;
		uint32_t	RR_CLLR_TSD: 1;
		uint32_t	RR_CLLR_TW: 1;
		uint32_t	RR_CLLR_Tinfo: 2;
		uint32_t	RR_CLLR_STATE: 1;
		uint32_t	RR_CLLR_Sensor: 1;
		uint32_t	RR_CLLR_OV: 1;
		uint32_t	RR_CLLR_UV: 1;
		uint32_t	RR_CLLR_OPEN2: 1;
		uint32_t	RR_CLLR_OPEN1: 1;
		uint32_t	RR_CLLR_OVC2: 1;
		uint32_t	RR_CLLR_OVC1: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_RR_CLLR_STATUS_32;;


typedef union  
{
  struct RR_CLUD_STATUS_t
  {
		uint32_t	RR_CLUD_SPEED: 4;
		uint32_t	RR_CLUD_ActuatorState: 12;
		uint32_t	RR_CLUD_OpDone: 1;
		uint32_t	RR_CLUD_VddReset: 1;
		uint32_t	RR_CLUD_Steploss: 1;
		uint32_t	RR_CLUD_EIDef: 1;
		uint32_t	RR_CLUD_TSD: 1;
		uint32_t	RR_CLUD_TW: 1;
		uint32_t	RR_CLUD_Tinfo: 2;
		uint32_t	RR_CLUD_STATE: 1;
		uint32_t	RR_CLUD_Sensor: 1;
		uint32_t	RR_CLUD_OV: 1;
		uint32_t	RR_CLUD_UV: 1;
		uint32_t	RR_CLUD_OPEN2: 1;
		uint32_t	RR_CLUD_OPEN1: 1;
		uint32_t	RR_CLUD_OVC2: 1;
		uint32_t	RR_CLUD_OVC1: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_RR_CLUD_STATUS_33;

typedef union  
{
  struct RR_CRLR_STATUS_t
  {
		uint32_t	RR_CRLR_SPEED: 4;
		uint32_t	RR_CRLR_ActuatorState: 12;
		uint32_t	RR_CRLR_OpDone: 1;
		uint32_t	RR_CRLR_VddReset: 1;
		uint32_t	RR_CRLR_Steploss: 1;
		uint32_t	RR_CRLR_EIDef: 1;
		uint32_t	RR_CRLR_TSD: 1;
		uint32_t	RR_CRLR_TW: 1;
		uint32_t	RR_CRLR_Tinfo: 2;
		uint32_t	RR_CRLR_STATE: 1;
		uint32_t	RR_CRLR_Sensor: 1;
		uint32_t	RR_CRLR_OV: 1;
		uint32_t	RR_CRLR_UV: 1;
		uint32_t	RR_CRLR_OPEN2: 1;
		uint32_t	RR_CRLR_OPEN1: 1;
		uint32_t	RR_CRLR_OVC2: 1;
		uint32_t	RR_CRLR_OVC1: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_RR_CRLR_STATUS_34;

typedef union  
{
  struct RR_CRUD_STATUS_t
  {
		uint32_t	RR_CRUD_SPEED: 4;
		uint32_t	RR_CRUD_ActuatorState: 12;
		uint32_t	RR_CRUD_OpDone: 1;
		uint32_t	RR_CRUD_VddReset: 1;
		uint32_t	RR_CRUD_Steploss: 1;
		uint32_t	RR_CRUD_EIDef: 1;
		uint32_t	RR_CRUD_TSD: 1;
		uint32_t	RR_CRUD_TW: 1;
		uint32_t	RR_CRUD_Tinfo: 2;
		uint32_t	RR_CRUD_STATE: 1;
		uint32_t	RR_CRUD_Sensor: 1;
		uint32_t	RR_CRUD_OV: 1;
		uint32_t	RR_CRUD_UV: 1;
		uint32_t	RR_CRUD_OPEN2: 1;
		uint32_t	RR_CRUD_OPEN1: 1;
		uint32_t	RR_CRUD_OVC2: 1;
		uint32_t	RR_CRUD_OVC1: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_RR_CRUD_STATUS_35;

typedef union  
{
  struct lin_RC_EVNT_SPECIAL_CMD_t
  {
  	uint16_t	EVNT_CMD: 8;
		uint16_t	EVNT_Broad: 1;
		uint16_t	EVNT_ADDr: 7;
  } LDATA;
  uint8_t byte[2];
  
}lin_RC_EVNT_SPECIAL_CMD_0;
#endif


typedef struct
{
	t_FR_SLLR_STATUS_12	fr_sllr_status;
	t_FR_SLUD_STATUS_13	fr_slud_status;
	t_FR_CLLR_STATUS_14	fr_cllr_status;
	t_FR_CLUD_STATUS_15	fr_clud_status;
}t_lin_LH_FR_STATUS;

typedef struct
{
	t_FR_CRLR_STATUS_22	fr_crlr_status;
	t_FR_CRUD_STATUS_23	fr_crud_status;
	t_FR_SRLR_STATUS_24	fr_srlr_status;
	t_FR_SRUD_STATUS_25	fr_srud_status;
}t_lin_RH_FR_STATUS;

#ifdef AMO_GN7_PE_SETTING_NONE		
typedef struct
{
	t_RR_CLLR_STATUS_32	rr_cllr_status;
	t_RR_CLUD_STATUS_33	rr_clud_status;
	t_RR_CRLR_STATUS_34	rr_crlr_status;
	t_RR_CRUD_STATUS_35	rr_crud_status;
}t_lin_RC_RR_STATUS;
#endif

extern lin_LH_EVNT_MASTER_CMD_11 LIN_LH_EVNT_MASTER_CMD;
extern lin_RH_EVNT_MASTER_CMD_21	LIN_RH_EVNT_MASTER_CMD;
#ifdef AMO_GN7_PE_SETTING_NONE		
extern lin_RC_EVNT_MASTER_CMD_31	LIN_RC_EVNT_MASTER_CMD;
#endif

extern lin_LH_EVNT_SPECIAL_CMD_0 LIN_LH_EVNT_SPECIAL_CMD;
extern lin_RH_EVNT_SPECIAL_CMD_0 LIN_RH_EVNT_SPECIAL_CMD;
#ifdef AMO_GN7_PE_SETTING_NONE		
extern lin_RC_EVNT_SPECIAL_CMD_0 LIN_RC_EVNT_SPECIAL_CMD;
#endif


extern t_lin_LH_FR_STATUS	LIN_LH_FR_STATUS;
extern t_lin_RH_FR_STATUS	LIN_RH_FR_STATUS;
#ifdef AMO_GN7_PE_SETTING_NONE		
extern t_lin_RC_RR_STATUS	LIN_RC_RR_STATUS;
#endif

extern Lin_EVntPt_t LIN_EVNT_PT;

extern LIN_flag_t LIN_FLAG;

extern FR_LinBusOff_flag_t FR_LINBUSOff_FLAG;
extern FR_DTC_Flt_flag_t	FR_DTC_Flt_flag;

extern uint8_t FR_LH_Target_Pt_act_flag;
extern uint8_t FR_RH_Target_Pt_act_flag;

extern uint8_t lin_WakeupFlag;

extern CanLin_oldNewValue_check_t CanValue;

extern SelectEVnt_t CurrentEVnt;

extern EvntMode_flag_t evntModeflags;

extern volatile bool backup_Flag;



#ifdef	Amo_LIN_C_


#if (defined ACT_ANGLE_UPDATE || defined ACT_ANGLE_UPDATE_2 || defined ACT_ANGLE_UPDATE_3)
extern const uint16_t canFactor;
extern const uint16_t linRangePt_X; 
extern const uint16_t linRangePt_Y;

#elif defined(OLD_ANGLE)
extern const uint16_t canFactor;
extern const uint16_t linRangePt_X; //29dgree //600U 30dgree 
extern const uint16_t linRangePt_Y; //24dgree //500U 25dgree //520U 26dgree
#endif

extern CanLinValue_t CanLinValue[20];

void Lin_Init(void);
void lin_sleep_task(void);
void Set_LinToCan_L_EVNT_HU_FrDrEvntPtDisp_XY(Lvnt_1_data *L1_DataBuf, const Atcu_20_data *canPtDataBuf);
void Set_LinToCan_L_EVNT_HU_FrPsEvntPtDisp_XY(Lvnt_2_data *L2_DataBuf, const Atcu_21_data *canPtDataBuf);
#ifdef AMO_GN7_PE_SETTING_NONE		
void Set_LinToCan_L_EVNT_HU_RrEvntPtDisp_XY(Lvnt_3_data *L3_DataBuf, const Atcu_22_data *canPtDataBuf);
#endif
void lin_master_task(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_Write_FrDrEVntPt_XY(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Lvnt_3_data *L3_DataBuf);
void lin_Write_FrPsEVntPt_XY(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Lvnt_3_data *L3_DataBuf);
#ifdef AMO_GN7_PE_SETTING_NONE		
void lin_Write_RrEVntConsPt_XY(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
#endif
void lin_FrDrmanualMode_task(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_FrPsmanualMode_task(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
#ifdef AMO_GN7_PE_SETTING_NONE		
void lin_RrmanualMode_task(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
#endif
void CanData_Update_Check_FrDrPt(const Atcu_20_data *canPtDataBuf);
void CanData_Update_Check_FrPsPt(const Atcu_21_data *canPtDataBuf);
#ifdef AMO_GN7_PE_SETTING_NONE		
void CanData_Update_Check_RrPt(const Atcu_22_data *canPtDataBuf);
#endif
void FrDrLinData_Parsing(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RHD_FrDrLinData_Parsing(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void FrPsLinData_Parsing(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void RHD_FrPsLinData_Parsing(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);

void FrDrLinData_Parsing_Key(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void FrPsLinData_Parsing_Key(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);

#ifdef AMO_GN7_PE_SETTING_NONE		
void RrLinData_Parsing(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
#endif
//void lin_FrDrVentMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_FrDrVent_FullClose_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RHD_lin_FrDrVent_FullClose_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_FrDrVent_Focus_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RHD_lin_FrDrVent_Focus_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_FrDrVent_Spread_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RHD_lin_FrDrVent_Spread_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
//void lin_FrPsVentMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void lin_FrPsVent_FullClose_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void RHD_lin_FrPsVent_FullClose_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void lin_FrPsVent_Focus_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void RHD_lin_FrPsVent_Focus_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void lin_FrPsVent_Spread_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void RHD_lin_FrPsVent_Spread_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
#ifdef AMO_GN7_PE_SETTING_NONE
void lin_RrVentMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
#endif
uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_X(const Atcu_20_data *canPtDataBuf, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_Y(const Atcu_20_data *canPtDataBuf, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_X(const Atcu_20_data *canPtDataBuf, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_Y(const Atcu_20_data *canPtDataBuf, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_X(const Atcu_21_data *canPtDataBuf, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_X(const Atcu_21_data *canPtDataBuf, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_Y(const Atcu_21_data *canPtDataBuf, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_Y(const Atcu_21_data *canPtDataBuf, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_X(const Atcu_22_data *canPtDataBuf, const Atcu_25_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_X(const Atcu_22_data *canPtDataBuf, const Atcu_25_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_Y(const Atcu_22_data *canPtDataBuf, const Atcu_25_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_Y(const Atcu_22_data *canPtDataBuf, const Atcu_25_data *canBdryDataBuf);

uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_X_Key(const Atcu_20_data_Key *canPtDataBuf, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_Y_Key(const Atcu_20_data_Key *canPtDataBuf, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_X_Key(const Atcu_20_data_Key *canPtDataBuf, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_Y_Key(const Atcu_20_data_Key *canPtDataBuf, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_X_Key(const Atcu_21_data_Key *canPtDataBuf, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_X_Key(const Atcu_21_data_Key *canPtDataBuf, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_Y_Key(const Atcu_21_data_Key *canPtDataBuf, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_Y_Key(const Atcu_21_data_Key *canPtDataBuf, const Atcu_24_data *canBdryDataBuf);

void SetLIN_EVNT_FrDrSPEED(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, uint8_t rpm);
void SetLIN_EVNT_FrPsSPEED(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, uint8_t rpm);
#ifdef AMO_GN7_PE_SETTING_NONE		
void SetLIN_EVNT_RrSPEED(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, uint8_t rpm);
#endif
void Set_LinToCan_Disp_FrDr_XY(Lvnt_1_data *L1_DataBuf);
uint16_t SET_L_EVNT_HU_FrDrEVntSidePt_X(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_FrDrEVntSidePt_Y(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_FrDrEVntCtrPt_X(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_FrDrEVntCtrPt_Y(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
void Set_LinToCan_Disp_FrPs_XY(Lvnt_2_data *L2_DataBuf);
uint16_t SET_L_EVNT_HU_FrPsEVntSidePt_X(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_FrPsEVntSidePt_Y(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_FrPsEVntCtrPt_X(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_FrPsEVntCtrPt_Y(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
#ifdef AMO_GN7_PE_SETTING_NONE	
void Set_LinToCan_Disp_Rr_XY(Lvnt_3_data *L3_DataBuf);
uint16_t SET_L_EVNT_HU_RrDrEVntSidePt_X(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, const Atcu_25_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_RrDrEVntSidePt_Y(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, const Atcu_25_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_RrPsEVntCtrPt_X(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, const Atcu_25_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_RrPsEVntCtrPt_Y(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, const Atcu_25_data *canBdryDataBuf);
#endif
void Set_LinToCan_Disp_FrDr_Mode(Lvnt_4_data *L4_DataBuf);
void Set_LinToCan_Disp_FrPs_Mode(Lvnt_4_data *L4_DataBuf);
#ifdef AMO_GN7_PE_SETTING_NONE		
void Set_LinToCan_Disp_RrDr_Mode(Lvnt_4_data *L4_DataBuf);
void Set_LinToCan_Disp_RrPs_Mode(Lvnt_4_data *L4_DataBuf);
#endif

void FrDrLinData_Parsing_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void FrPsLinData_Parsing_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void RHD_FrDrLinData_Parsing_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void RHD_FrPsLinData_Parsing_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);


void lin_FrDrVent_FullClose_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RHD_lin_FrDrVent_FullClose_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_FrDrVent_Focus_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RHD_lin_FrDrVent_Focus_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_FrDrVent_Spread_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RHD_lin_FrDrVent_Spread_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);

void lin_FrPsVent_FullClose_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void RHD_lin_FrPsVent_FullClose_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void lin_FrPsVent_Focus_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void RHD_lin_FrPsVent_Focus_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void lin_FrPsVent_Spread_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void RHD_lin_FrPsVent_Spread_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);

void lin_FrPsmanualMode_task_Backup(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void lin_FrDrmanualMode_task_Backup(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);

void Lin_GoTo_Sleep(void);
void Lin_GoTo_WakeUp(void);
void Lin_FR_DR_BusOff_Check(FR_LinBusOff_flag_t *fault_flag);
void Lin_FR_PS_BusOff_Check(FR_LinBusOff_flag_t *fault_flag);
void lin_BusOff(void);



void FrDr_FullCloseMode_task(uint8_t side_sig, uint8_t center_sig);
void FrDr_FullCloseCycleControl(void);
void FrPs_FullCloseMode_task(uint8_t side_sig, uint8_t center_sig);
void FrPs_FullCloseCycleControl(void);
void Set_LinToCan_Disp_FrDr_XY_Backup(void);
void Set_LinToCan_Disp_FrPs_XY_Backup(void);
void FrDr_SetLinTargetPosition_side(uint16_t LH_Side_LR, uint16_t LH_Side_UD);
void FrDr_SetLinTargetPosition_ctr(uint16_t LH_Ctr_LR, uint16_t LH_Ctr_UD);
void FrPs_SetLinTargetPosition_side(uint16_t RH_Side_LR, uint16_t RH_Side_UD);
void FrPs_SetLinTargetPosition_ctr(uint16_t RH_Ctr_LR, uint16_t RH_Ctr_UD);
void FrDr_FocusSpreadMode_task(void);
void FrPs_FocusSpreadMode_task(void);










#else

extern const uint16_t canFactor;
extern const uint16_t linRangePt_X;
extern const uint16_t linRangePt_Y;

extern CanLinValue_t CanLinValue[20];

extern void Lin_Init(void);
extern void lin_sleep_task(void);
extern void Set_LinToCan_L_EVNT_HU_FrDrEvntPtDisp_XY(Lvnt_1_data *L1_DataBuf, const Atcu_20_data *canPtDataBuf);
extern void Set_LinToCan_L_EVNT_HU_FrPsEvntPtDisp_XY(Lvnt_2_data *L2_DataBuf, const Atcu_21_data *canPtDataBuf);
#ifdef AMO_GN7_PE_SETTING_NONE		
extern void Set_LinToCan_L_EVNT_HU_RrEvntPtDisp_XY(Lvnt_3_data *L3_DataBuf, const Atcu_22_data *canPtDataBuf);
#endif
extern void lin_Write_FrDrEVntPt_XY(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Lvnt_3_data *L3_DataBuf);
extern void lin_master_task(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void lin_Write_FrPsEVntPt_XY(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Lvnt_3_data *L3_DataBuf);
#ifdef AMO_GN7_PE_SETTING_NONE		
extern void lin_Write_RrEVntConsPt_XY(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
#endif
extern void lin_FrDrmanualMode_task(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void lin_FrPsmanualMode_task(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
#ifdef AMO_GN7_PE_SETTING_NONE		
extern void lin_RrmanualMode_task(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
#endif
extern void CanData_Update_Check_FrDrPt(const Atcu_20_data *canPtDataBuf);
extern void CanData_Update_Check_FrPsPt(const Atcu_21_data *canPtDataBuf);
#ifdef AMO_GN7_PE_SETTING_NONE		
extern void CanData_Update_Check_RrPt(const Atcu_22_data *canPtDataBuf);
#endif
//extern void FrDrLinData_Parsing(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void FrDrLinData_Parsing(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void RHD_FrDrLinData_Parsing(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void FrPsLinData_Parsing(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void RHD_FrPsLinData_Parsing(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void FrDrLinData_Parsing_Key(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void FrPsLinData_Parsing_Key(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);

#ifdef AMO_GN7_PE_SETTING_NONE		
extern void RrLinData_Parsing(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
#endif
extern void lin_FrDrVent_FullClose_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void RHD_lin_FrDrVent_FullClose_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void lin_FrDrVent_Focus_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void RHD_lin_FrDrVent_Focus_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void lin_FrDrVent_Spread_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void RHD_lin_FrDrVent_Spread_Mode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
//extern void lin_FrDrVentMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
//extern void lin_FrPsVentMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void lin_FrPsVent_FullClose_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void RHD_lin_FrPsVent_FullClose_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void lin_FrPsVent_Focus_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void RHD_lin_FrPsVent_Focus_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void lin_FrPsVent_Spread_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void RHD_lin_FrPsVent_Spread_Mode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
#ifdef AMO_GN7_PE_SETTING_NONE
extern void lin_RrVentMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
#endif
extern uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_X(const Atcu_20_data *canPtDataBuf, const Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_Y(const Atcu_20_data *canPtDataBuf, const Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_X(const Atcu_20_data *canPtDataBuf, const Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_Y(const Atcu_20_data *canPtDataBuf, const Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_X(const Atcu_21_data *canPtDataBuf, const Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_X(const Atcu_21_data *canPtDataBuf, const Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_Y(const Atcu_21_data *canPtDataBuf, const Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_Y(const Atcu_21_data *canPtDataBuf, const Atcu_24_data *canBdryDataBuf);
#ifdef AMO_GN7_PE_SETTING_NONE		
extern uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_X(const Atcu_22_data *canPtDataBuf, const Atcu_25_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_X(const Atcu_22_data *canPtDataBuf, const Atcu_25_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_Y(const Atcu_22_data *canPtDataBuf, const Atcu_25_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_Y(const Atcu_22_data *canPtDataBuf, const Atcu_25_data *canBdryDataBuf);
#endif
extern void SetLIN_EVNT_FrDrSPEED(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, uint8_t rpm);
extern void SetLIN_EVNT_FrPsSPEED(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, uint8_t rpm);
#ifdef AMO_GN7_PE_SETTING_NONE		
extern void SetLIN_EVNT_RrSPEED(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, uint8_t rpm);
#endif
extern void Set_LinToCan_Disp_FrDr_XY(Lvnt_1_data *L1_DataBuf);
extern uint16_t SET_L_EVNT_HU_FrDrEVntSidePt_X(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_FrDrEVntSidePt_Y(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_FrDrEVntCtrPt_X(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_FrDrEVntCtrPt_Y(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
extern void Set_LinToCan_Disp_FrPs_XY(Lvnt_2_data *L2_DataBuf);
extern uint16_t SET_L_EVNT_HU_FrPsEVntSidePt_X(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_FrPsEVntSidePt_Y(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_FrPsEVntCtrPt_X(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_FrPsEVntCtrPt_Y(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
#ifdef AMO_GN7_PE_SETTING_NONE		
extern void Set_LinToCan_Disp_Rr_XY(Lvnt_3_data *L3_DataBuf);
extern uint16_t SET_L_EVNT_HU_RrDrEVntSidePt_X(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, const Atcu_25_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_RrDrEVntSidePt_Y(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, const Atcu_25_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_RrPsEVntCtrPt_X(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, const Atcu_25_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_RrPsEVntCtrPt_Y(const lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl, const Atcu_25_data *canBdryDataBuf);
#endif
extern void Set_LinToCan_Disp_FrDr_Mode(Lvnt_4_data *L4_DataBuf);
extern void Set_LinToCan_Disp_FrPs_Mode(Lvnt_4_data *L4_DataBuf);
#ifdef AMO_GN7_PE_SETTING_NONE		
extern void Set_LinToCan_Disp_RrDr_Mode(Lvnt_4_data *L4_DataBuf);
extern void Set_LinToCan_Disp_RrPs_Mode(Lvnt_4_data *L4_DataBuf);
#endif

extern void FrDrLinData_Parsing_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void FrPsLinData_Parsing_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void RHD_FrDrLinData_Parsing_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void RHD_FrPsLinData_Parsing_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);

extern void lin_FrDrVent_FullClose_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void RHD_lin_FrDrVent_FullClose_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void lin_FrDrVent_Focus_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void RHD_lin_FrDrVent_Focus_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void lin_FrDrVent_Spread_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void RHD_lin_FrDrVent_Spread_Backup(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);

extern void lin_FrPsVent_FullClose_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void RHD_lin_FrPsVent_FullClose_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void lin_FrPsVent_Focus_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void RHD_lin_FrPsVent_Focus_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void lin_FrPsVent_Spread_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void RHD_lin_FrPsVent_Spread_Backup(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);

extern void lin_FrPsmanualMode_task_Backup(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void lin_FrDrmanualMode_task_Backup(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);

extern void Lin_GoTo_Sleep(void);
extern void Lin_GoTo_WakeUp(void);
extern void Lin_FR_DR_BusOff_Check(FR_LinBusOff_flag_t *fault_flag);
extern void Lin_FR_PS_BusOff_Check(FR_LinBusOff_flag_t *fault_flag);
extern void lin_BusOff(void);







extern void FrDr_FullCloseMode_task(uint8_t side_sig, uint8_t center_sig);
extern void FrDr_FullCloseCycleControl(void);
extern void FrPs_FullCloseMode_task(uint8_t side_sig, uint8_t center_sig);
extern void FrPs_FullCloseCycleControl(void);
extern void Set_LinToCan_Disp_FrDr_XY_Backup(void);
extern void Set_LinToCan_Disp_FrPs_XY_Backup(void);
extern void FrDr_SetLinTargetPosition_side(uint16_t LH_Side_LR, uint16_t LH_Side_UD);
extern void FrDr_SetLinTargetPosition_ctr(uint16_t LH_Ctr_LR, uint16_t LH_Ctr_UD);
extern void FrPs_SetLinTargetPosition_side(uint16_t RH_Side_LR, uint16_t RH_Side_UD);
extern void FrPs_SetLinTargetPosition_ctr(uint16_t RH_Ctr_LR, uint16_t RH_Ctr_UD);
extern void FrDr_FocusSpreadMode_task(void);
extern void FrPs_FocusSpreadMode_task(void);






#endif /* Amo_LIN_C_ */


#endif /* Amo_LIN_H_ */



