#ifndef Amo_LIN_H_ /* to interprete header file only once */
#define Amo_LIN_H_

#include <stdio.h>

#include "Cpu.h"
#include "Amo_CAN_Parsing.h"

extern Atcu_20_data Can_data_20_Info;
extern Atcu_23_data Can_data_23_Info;
extern Atcu_21_data Can_data_21_Info;
extern Atcu_24_data Can_data_24_Info;
extern Atcu_22_data Can_data_22_Info;
extern Atcu_25_data Can_data_25_Info;

extern Evnt_mode Hu_Fr_Dr_Vent_Mode;
extern Evnt_mode Hu_Fr_Ps_Vent_Mode;
extern Evnt_mode Hu_Rr_Dr_Vent_Mode;
extern Evnt_mode Hu_Rr_Ps_Vent_Mode;

extern uint8_t Hu_Fr_dr_Vent_Side_signal;
extern uint8_t Hu_Fr_dr_Vent_Center_signal;
extern uint8_t Hu_Fr_Ps_Vent_Side_signal;
extern uint8_t Hu_Fr_Ps_Vent_Center_signal;
extern uint8_t Hu_Rr_Dr_Vent_signal;
extern uint8_t Hu_Rr_Ps_Vent_signal;

/*
#define CAN_FACTOR 10U
#define LIN_RANGE_PT_X 600U
#define LIN_RANGE_PT_Y 520U
*/

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
	RrDrCons_X,
	RrDrCons_Y,
	RrPsCons_X,
	RrPsCons_Y
}CanLin_Position_XY_t;

typedef enum
{
	DefaultPt = 0,
	FrDrSidePt_XY,
	FrDrCtrPt_XY,
	FrPsSidePt_XY,
	FrPsCtrPt_XY,
	RrDrCons_XY,
	RrPsCons_XY
}CanLin_selectEVnt_t;


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
	int16_t Hu_FrDrEVntSidePt_X_oldVal;
	int16_t Hu_FrDrEVntSidePt_X_newVal;
	int16_t Hu_FrDrEVntSidePt_Y_oldVal;
	int16_t Hu_FrDrEVntSidePt_Y_newVal;
	int16_t Hu_FrDrEVntCtrPt_X_oldVal;
	int16_t Hu_FrDrEVntCtrPt_X_newVal;
	int16_t Hu_FrDrEVntCtrPt_Y_oldVal;
	int16_t Hu_FrDrEVntCtrPt_Y_newVal;
}CanLin_FrDrEVntPt_XY_t;

typedef struct CanLin_FrPsEVntPt_XY_tag
{
	int16_t Hu_FrPsEVntSidePt_X_oldVal;
	int16_t Hu_FrPsEVntSidePt_X_newVal;
	int16_t Hu_FrPsEVntSidePt_Y_oldVal;
	int16_t Hu_FrPsEVntSidePt_Y_newVal;
	int16_t Hu_FrPsEVntCtrPt_X_oldVal;
	int16_t Hu_FrPsEVntCtrPt_X_newVal;
	int16_t Hu_FrPsEVntCtrPt_Y_oldVal;
	int16_t Hu_FrPsEVntCtrPt_Y_newVal;
}CanLin_FrPsEVntPt_XY_t;

typedef struct CanLin_RrEVntPt_XY_tag
{
	int16_t Hu_RrDrEVntConsPt_X_oldVal;
	int16_t Hu_RrDrEVntConsPt_X_newVal;
	int16_t Hu_RrDrEVntConsPt_Y_oldVal;
	int16_t Hu_RrDrEVntConsPt_Y_newVal;
	int16_t Hu_RrPsEVntConsPt_X_oldVal;
	int16_t Hu_RrPsEVntConsPt_X_newVal;
	int16_t Hu_RrPsEVntConsPt_Y_oldVal;
	int16_t Hu_RrPsEVntConsPt_Y_newVal;
}CanLin_RrEVntPt_XY_t;




typedef struct CanLin_oldNewValue_check_tag
{
	CanLin_FrDrEVntPt_XY_t FrDrPt;
	CanLin_FrPsEVntPt_XY_t FrPsPt;
	CanLin_RrEVntPt_XY_t RrPt;
}CanLin_oldNewValue_check_t;


typedef struct 
{
	uint8_t	linEvntAddr_Flag;

}LIN_flag_t;


typedef union  
{
  struct lin_LH_EVNT_MASTER_CMD_t
  {
  	uint64_t	EVNT_ADDr: 7;
		uint64_t	EVNT_Broad: 1;
		uint64_t	EVNT_SPEED: 4;
		uint64_t	EVNT_Front_Side_LH_LeftRight_TargetPosition: 11;
		uint64_t	EVNT_Front_Side_LH_UpDown_TargetPosition: 11;	
		uint64_t	EVNT_Front_Center_LH_LeftRight_TargetPosition: 11;
		uint64_t	EVNT_Front_Center_LH_UpDown_TargetPosition: 11;
		uint64_t	reserved1: 8;
  } LDATA;
  uint8_t byte[8];
  
}lin_LH_EVNT_MASTER_CMD_11;

typedef union  
{
  struct FR_SLLR_STATUS_t
  {
  	uint32_t	FR_SLLR_SPEED: 4;
		uint32_t	FR_SLLR_ActuatorState: 11;
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
		uint32_t	reserved2: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_SLLR_STATUS_12;


typedef union  
{
  struct FR_SLUD_STATUS_t
  {
		uint32_t	FR_SLUD_SPEED: 4;
		uint32_t	FR_SLUD_ActuatorState: 11;
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
		uint32_t	reserved3: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_SLUD_STATUS_13;

typedef union  
{
  struct FR_CLLR_STATUS_t
  {
		uint32_t	FR_CLLR_SPEED: 4;
		uint32_t	FR_CLLR_ActuatorState: 11;
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
		uint32_t	reserved4: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_CLLR_STATUS_14;

typedef union  
{
  struct FR_CLUD_STATUS_t
  {
		uint32_t	FR_CLUD_SPEED: 4;
		uint32_t	FR_CLUD_ActuatorState: 11;
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
		uint32_t	reserved5: 1;
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
  	uint64_t	EVNT_ADDr: 7;
		uint64_t	EVNT_Broad: 1;
		uint64_t	EVNT_SPEED: 4;
		uint64_t	EVNT_Front_Center_RH_LeftRight_TargetPosition: 11;
		uint64_t	EVNT_Front_Center_RH_UpDown_TargetPosition: 11;
		uint64_t	EVNT_Front_Side_RH_LeftRight_TargetPosition: 11;
		uint64_t	EVNT_Front_Side_RH_UpDown_TargetPosition: 11;	
		uint64_t	reserved6: 8;
  } LDATA;
  uint8_t byte[8];
  
}lin_RH_EVNT_MASTER_CMD_21;


typedef union  
{
  struct FR_CRLR_STATUS_t
  {
  	uint32_t	FR_CRLR_SPEED: 4;
		uint32_t	FR_CRLR_ActuatorState: 11;
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
		uint32_t	reserved7: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_CRLR_STATUS_22;


typedef union  
{
  struct FR_CRUD_STATUS_t
  {
		uint32_t	FR_CRUD_SPEED: 4;
		uint32_t	FR_CRUD_ActuatorState: 11;
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
		uint32_t	reserved8: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_CRUD_STATUS_23;

typedef union  
{
  struct FR_SRLR_STATUS_t
  {
		uint32_t	FR_SRLR_SPEED: 4;
		uint32_t	FR_SRLR_ActuatorState: 11;
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
		uint32_t	reserved9: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_FR_SRLR_STATUS_24;

typedef union  
{
  struct FR_SRUD_STATUS_t
  {
		uint32_t	FR_SRUD_SPEED: 4;
		uint32_t	FR_SRUD_ActuatorState: 11;
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
		uint32_t	reserved10: 1;
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


typedef union  
{
  struct lin_RC_EVNT_MASTER_CMD_t
  {
  	uint64_t	EVNT_ADDr: 7;
		uint64_t	EVNT_Broad: 1;
		uint64_t	EVNT_REAR_RH_SPEED: 4;
		uint64_t	EVNT_REAR_LH_SPEED: 4;
		uint64_t	EVNT_Rear_Center_LH_LeftRight_TargetPosition: 11;
		uint64_t	EVNT_Rear_Center_LH_UpDown_TargetPosition: 11;	
		uint64_t	EVNT_Rear_Center_RH_LeftRight_TargetPosition: 11;
		uint64_t	EVNT_Rear_Center_RH_UpDown_TargetPosition: 11;
		uint64_t	reserved11: 4;
  } LDATA;
  uint8_t byte[8];
  
}lin_RC_EVNT_MASTER_CMD_31;


typedef union  
{
  struct RR_CLLR_STATUS_t
  {
  	uint32_t	RR_CLLR_SPEED: 4;
		uint32_t	RR_CLLR_ActuatorState: 11;
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
		uint32_t	reserved12: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_RR_CLLR_STATUS_32;;


typedef union  
{
  struct RR_CLUD_STATUS_t
  {
		uint32_t	RR_CLUD_SPEED: 4;
		uint32_t	RR_CLUD_ActuatorState: 11;
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
		uint32_t	reserved13: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_RR_CLUD_STATUS_33;

typedef union  
{
  struct RR_CRLR_STATUS_t
  {
		uint32_t	RR_CRLR_SPEED: 4;
		uint32_t	RR_CRLR_ActuatorState: 11;
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
		uint32_t	reserved14: 1;
  } LDATA;
  uint8_t byte[4];
  
}t_RR_CRLR_STATUS_34;

typedef union  
{
  struct RR_CRUD_STATUS_t
  {
		uint32_t	RR_CRUD_SPEED: 4;
		uint32_t	RR_CRUD_ActuatorState: 11;
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
		uint32_t	reserved15: 1;
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

typedef struct
{
	t_RR_CLLR_STATUS_32	fr_cllr_status;
	t_RR_CLUD_STATUS_33	fr_clud_status;
	t_RR_CRLR_STATUS_34	fr_crlr_status;
	t_RR_CRUD_STATUS_35	fr_crud_status;
}t_lin_RC_RR_STATUS;


lin_LH_EVNT_MASTER_CMD_11 LIN_LH_EVNT_MASTER_CMD;
lin_RH_EVNT_MASTER_CMD_21	LIN_RH_EVNT_MASTER_CMD;
lin_RC_EVNT_MASTER_CMD_31	LIN_RC_EVNT_MASTER_CMD;

lin_LH_EVNT_SPECIAL_CMD_0 LIN_LH_EVNT_SPECIAL_CMD;
lin_RH_EVNT_SPECIAL_CMD_0 LIN_RH_EVNT_SPECIAL_CMD;
lin_RC_EVNT_SPECIAL_CMD_0 LIN_RC_EVNT_SPECIAL_CMD;


t_lin_LH_FR_STATUS	LIN_LH_FR_STATUS;
t_lin_RH_FR_STATUS	LIN_RH_FR_STATUS;
t_lin_RC_RR_STATUS	LIN_RC_RR_STATUS;

LIN_flag_t LIN_FLAG;


#ifdef	Amo_LIN_C_

const uint16_t canFactor = 10U;
const uint16_t linRangePt_X = 600U;
const uint16_t linRangePt_Y = 520U;

CanLinValue_t CanLinValue[20];

void Lin_Init(void);
void lin_sleep_task(void);
void Set_LinToCan_L_EVNT_HU_FrDrEvntPtDisp_XY(Lvnt_1_data *L1_DataBuf, Atcu_20_data *canPtDataBuf);
void Set_LinToCan_L_EVNT_HU_FrPsEvntPtDisp_XY(Lvnt_2_data *L2_DataBuf, Atcu_21_data *canPtDataBuf);
void Set_LinToCan_L_EVNT_HU_RrEvntPtDisp_XY(Lvnt_3_data *L3_DataBuf, Atcu_22_data *canPtDataBuf);
void lin_FrDrmanualMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_FrPsmanualMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void lin_RrmanualMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
void CanData_Update_Check_FrDrPt(Atcu_20_data *canPtDataBuf);
void CanData_Update_Check_FrPsPt(Atcu_21_data *canPtDataBuf);
void CanData_Update_Check_RrPt(Atcu_22_data *canPtDataBuf);
void FrDrLinData_Parsing(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void FrPsLinData_Parsing(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void RrLinData_Parsing(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
void lin_FrDrVentMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_FrPsVentMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void lin_RrVentMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_X(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_Y(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_X(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_Y(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_X(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_X(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_Y(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_Y(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_X(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_X(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_Y(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf);
uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_Y(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf);
uint16_t CONV_CanToLin_TargetPos_void(void);
void SetLIN_EVNT_FrDrSPEED(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void SetLIN_EVNT_FrPsSPEED(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void SetLIN_EVNT_RrSPEED(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
void lin_CycleMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_SwingData_Sinario0(void);
void lin_SwingData_Sinario1(void);
void lin_SwingData_Sinario2(void);
void lin_SwingData_Sinario3(void);
void lin_SwingData_Sinario4(void);
void lin_SwingData_Sinario5(void);


#else

extern const uint16_t canFactor;
extern const uint16_t linRangePt_X;
extern const uint16_t linRangePt_Y;

extern CanLinValue_t CanLinValue[20];

extern void Lin_Init(void);
extern void lin_sleep_task(void);
extern void Set_LinToCan_L_EVNT_HU_FrDrEvntPtDisp_XY(Lvnt_1_data *L1_DataBuf, Atcu_20_data *canPtDataBuf);
extern void Set_LinToCan_L_EVNT_HU_FrPsEvntPtDisp_XY(Lvnt_2_data *L2_DataBuf, Atcu_21_data *canPtDataBuf);
extern void Set_LinToCan_L_EVNT_HU_RrEvntPtDisp_XY(Lvnt_3_data *L3_DataBuf, Atcu_22_data *canPtDataBuf);
extern void lin_FrDrmanualMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void lin_FrPsmanualMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void lin_RrmanualMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
extern void CanData_Update_Check_FrDrPt(Atcu_20_data *canPtDataBuf);
extern void CanData_Update_Check_FrPsPt(Atcu_21_data *canPtDataBuf);
extern void CanData_Update_Check_RrPt(Atcu_22_data *canPtDataBuf);
extern void FrDrLinData_Parsing(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void FrPsLinData_Parsing(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void RrLinData_Parsing(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
extern void lin_FrDrVentMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void lin_FrPsVentMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void lin_RrVentMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
extern uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_X(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrDrEVntSidePt_Y(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_X(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrDrEVntCtrPt_Y(Atcu_20_data *canPtDataBuf, Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_X(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_X(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrPsEVntSidePt_Y(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_FrPsEVntCtrPt_Y(Atcu_21_data *canPtDataBuf, Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_X(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_X(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_RrDrEVntConsPt_Y(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf);
extern uint16_t SET_L_ATCU_HU_RrPsEVntConsPt_Y(Atcu_22_data *canPtDataBuf, Atcu_25_data *canBdryDataBuf);
extern void SetLIN_EVNT_FrDrSPEED(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void SetLIN_EVNT_FrPsSPEED(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void SetLIN_EVNT_RrSPEED(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
extern void lin_CycleMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void lin_SwingData_Sinario0(void);
extern void lin_SwingData_Sinario1(void);
extern void lin_SwingData_Sinario2(void);
extern void lin_SwingData_Sinario3(void);
extern void lin_SwingData_Sinario4(void);
extern void lin_SwingData_Sinario5(void);


#endif /* Amo_LIN_C_ */


#endif /* Amo_LIN_H_ */



