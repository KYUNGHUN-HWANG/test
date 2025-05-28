#ifndef __AMO_CAN_H
#define __AMO_CAN_H

#include "Amo_STATE_Event.h"
/*===========================================================================*/
/* Project   :  AMOSENSE CAN driver Software                                                                */
/* File name :  Amo_CAN_Parsing.h                                                                             */
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
/* Header file for CAB DBC.                                                                               */
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

#define FLEXCAN_BusOffFlag 0x00000100						//Bus-off
#define FLEXCAN_ErrorPassiveFlag 0x00000200			//Error Passive
#define FLEXCAN_StuffingErrorFlag 0x00000400		//Stuffing Error
#define FLEXCAN_CrcErrorFlag 0x00001000					//CRC ERROR
#define FLEXCAN_AckErrorFlag 0x00002000					//Acknowledgement error
#define FLEXCAN_FormErrorFlag 0x00004000				//Form Error
#define FLEXCAN_BitErrorFlag 0x00008000					//Bit error

#define MAX_SN_NUMBER 0x0F
#define CRC32_POLYNOMIAL 0x04C11DB7

typedef enum _taskpriority{
	NONE_SELECT,
	P_OIP_SELECT,
	CTS_SELECT,	
}TaskProiority;

typedef struct _l_atcu_2{
	uint64_t reserved : 32;
	uint64_t L_ATCU_FrDrModDis : 4;	
	uint64_t L_ATCU_FrPassModDis : 4;
	uint64_t reserved1 : 24;
}L_ATCU_2;

typedef union _atcu2_data{
	L_ATCU_2 data;
	uint8_t reg_data[8];
}Atcu_2_data;

typedef struct _l_atcu_3{
	uint64_t reserved : 21;
	uint64_t reserved1 : 21;
	uint64_t L_ATCU_RhstaLvlSta : 5;	
	uint64_t reserved2 : 17;
}L_ATCU_3;

typedef union _atcu3_data{
	L_ATCU_3 data;
	uint8_t reg_data[8];
}Atcu_3_data;

typedef struct _l_atcu_4{
	uint64_t reserved : 27;
	uint64_t reserved1 : 27;
	uint64_t L_ATCU_OpSta : 3;	
	uint64_t reserved3 : 7;
}L_ATCU_4;

typedef union _atcu4_data{
	L_ATCU_4 data;
	uint8_t reg_data[8];
}Atcu_4_data;

typedef struct _l_atcu_5{
	uint64_t L_ATCU_RrOpSta : 3;
	uint64_t reserved : 16;
	uint64_t L_ATCU_RrDrModDis : 4;	
	uint64_t reserved1 : 30;
	uint64_t L_ATCU_RrPsModDis : 4;
	uint64_t reserved2 : 7;
}L_ATCU_5;

typedef union _atcu5_data{
	L_ATCU_5 data;
	uint8_t reg_data[8];
}Atcu_5_data;

typedef struct _l_atcu_8{
	uint64_t reserved : 28;
	uint64_t L_ATCU_CamModeSta : 2;	
	uint64_t L_ATCU_UtilModeUsmSta : 2;
	uint64_t reserved1 : 16;
	uint64_t reserved2 : 16;
}L_ATCU_8;

typedef union _atcu8_data{
	L_ATCU_8 data;
	uint8_t reg_data[8];
}Atcu_8_data;

typedef struct _l_atcu_9{
	uint64_t reserved : 16;	
	uint64_t L_ATCU_DtntOutSta : 1;
	uint64_t L_ATCU_AutoBrightSta : 8;
	uint64_t reserved1 : 10;
	uint64_t L_ATCU_NotMiniModeBrightSta : 8;
	uint64_t L_ATCU_IAUPrfleVal : 4;
	uint64_t L_ATCU_AVNPrfleVal : 4;
	uint64_t reserved2 : 13;
}L_ATCU_9;

typedef union _atcu9_data{
	L_ATCU_9 data;
	uint8_t reg_data[8];
}Atcu_9_data;

typedef struct _l_atcu_12{
	uint64_t reserved : 4;	
	uint64_t L_ATCU_DrvSdTyp : 1;
	uint64_t reserved1 : 21;
	uint64_t L_ATCU_FrBlwLvl : 4;
	uint64_t reserved2 : 17;
	uint64_t L_ATCU_PassBlwLvl : 4;
	uint64_t reserved3 : 13;
}L_ATCU_12;

typedef union _atcu12_data{
	L_ATCU_12 data;
	uint8_t reg_data[8];
}Atcu_12_data;

typedef struct _l_atcu_13{
	uint64_t reserved : 25;	
	uint64_t L_ATCU_RrBlwLvl : 4;
	uint64_t reserved1 : 10;
	uint64_t reserved2 : 25;
}L_ATCU_13;

typedef union _atcu13_data{
	L_ATCU_13 data;
	uint8_t reg_data[8];
}Atcu_13_data;

typedef struct _l_atcu_16{
	uint64_t reserved : 24;	
	uint64_t L_ATCU_2ndDrBlwLvl : 4;
	uint64_t L_ATCU_2ndPsBlwLvl : 4;
	uint64_t reserved1 : 16;
	uint64_t reserved2 : 16;
}L_ATCU_16;

typedef union _atcu16_data{
	L_ATCU_16 data;
	uint8_t reg_data[8];
}Atcu_16_data;

typedef struct _l_atcu_20{			//Front Mode define
	uint64_t L_ATCU_HU_FrDrEVntSidePt_X : 11;
	uint64_t L_ATCU_HU_FrDrEVntSidePt_Y : 11;
	uint64_t L_ATCU_HU_FrDrEVntCtrPt_X : 11;
	uint64_t L_ATCU_HU_FrDrEVntCtrPt_Y : 11;
	
	uint64_t reserved : 20;
}L_ATCU_20;

typedef union _atcu20_data{
	L_ATCU_20 data;
	uint8_t reg_data[8];
}Atcu_20_data;

typedef struct _l_atcu_21{			//Front Mode define
	uint64_t L_ATCU_HU_FrPsEVntSidePt_X : 11;
	uint64_t L_ATCU_HU_FrPsEVntSidePt_Y : 11;
	uint64_t L_ATCU_HU_FrPsEVntCtrPt_X : 11;
	uint64_t L_ATCU_HU_FrPsEVntCtrPt_Y : 11;
	
	uint64_t reserved : 20;
}L_ATCU_21;

typedef union _atcu21_data{
	L_ATCU_21 data;
	uint8_t reg_data[8];
}Atcu_21_data;

typedef struct _l_atcu_20_Key{			//Front Mode define
	uint64_t L_ATCU_HU_FrDrEVntSidePt_X_Key : 11;
	uint64_t L_ATCU_HU_FrDrEVntSidePt_Y_Key : 11;
	uint64_t L_ATCU_HU_FrDrEVntCtrPt_X_Key : 11;
	uint64_t L_ATCU_HU_FrDrEVntCtrPt_Y_Key : 11;
	uint64_t reserved : 20;
}L_ATCU_20_Key;
typedef union _atcu20_data_Key{
	L_ATCU_20_Key data;
	uint8_t reg_data[8];
}Atcu_20_data_Key;

typedef struct _l_atcu_21_Key{			//Front Mode define
	uint64_t L_ATCU_HU_FrPsEVntSidePt_X_Key : 11;
	uint64_t L_ATCU_HU_FrPsEVntSidePt_Y_Key : 11;
	uint64_t L_ATCU_HU_FrPsEVntCtrPt_X_Key : 11;
	uint64_t L_ATCU_HU_FrPsEVntCtrPt_Y_Key : 11;
	uint64_t reserved : 20;
}L_ATCU_21_Key;
typedef union _atcu21_data_Key{
	L_ATCU_21_Key data;
	uint8_t reg_data[8];
}Atcu_21_data_Key;


typedef struct _l_atcu_22{			//Rear Mode define from P-OIP
	uint64_t L_ATCU_HU_RrDrEVntConsPt_X : 11;
	uint64_t L_ATCU_HU_RrDrEVntConsPt_Y : 11;
	uint64_t L_ATCU_HU_RrPsEVntConsPt_X : 11;
	uint64_t L_ATCU_HU_RrPsEVntConsPt_Y : 11;
	uint64_t L_ATCU_HU_RrDrEVntConsOpnCls : 2;
	uint64_t L_ATCU_HU_RrPsEVNTConsOpnCls : 2;
	uint64_t L_ATCU_HU_RrDrEVntModeSet : 4;
	uint64_t L_ATCU_HU_RrPsEVNTModeSet : 4;	
	
	uint64_t reserved : 8;
}L_ATCU_22;

typedef union _atcu22_data{
	L_ATCU_22 data;
	uint8_t reg_data[8];
}Atcu_22_data;

typedef struct _l_atcu_23{			//Front Mode define from P-OIP position
	uint64_t L_ATCU_HU_FrDrEVntSideBdryUp_X : 8;
	uint64_t L_ATCU_HU_FrDrEVntSideBdryUp_Y : 8;
	uint64_t L_ATCU_HU_FrDrEVntSideBdryLo_X : 8;
	uint64_t L_ATCU_HU_FrDrEVntSideBdryLo_Y : 8;
	uint64_t L_ATCU_HU_FrDrEVntCtrBdryUP_X : 8;
	uint64_t L_ATCU_HU_FrDrEVntCtrBdryUP_Y : 8;
	uint64_t L_ATCU_HU_FrDrEVntCtrBdryLo_X : 8;
	uint64_t L_ATCU_HU_FrDrEVntCtrBdryLo_Y : 8;	
}L_ATCU_23;

typedef union _atcu23_data{
	L_ATCU_23 data;
	uint8_t reg_data[8];
}Atcu_23_data;

typedef struct _l_atcu_24{			//Front Mode define from P-OIP position
	uint64_t L_ATCU_HU_FrPsEVntSideBdryUp_X : 8;
	uint64_t L_ATCU_HU_FrPsEVntSideBdryUp_Y : 8;
	uint64_t L_ATCU_HU_FrPsEVntSideBdryLo_X : 8;
	uint64_t L_ATCU_HU_FrPsEVntSideBdryLo_Y : 8;
	uint64_t L_ATCU_HU_FrPsEVntCtrBdryUP_X : 8;
	uint64_t L_ATCU_HU_FrPsEVntCtrBdryUP_Y : 8;
	uint64_t L_ATCU_HU_FrPsEVntCtrBdryLo_X : 8;
	uint64_t L_ATCU_HU_FrPsEVntCtrBdryLo_Y : 8;	
}L_ATCU_24;

typedef union _atcu24_data{
	L_ATCU_24 data;
	uint8_t reg_data[8];
}Atcu_24_data;

typedef struct _l_atcu_25{			//Rear Mode define from P-OIP position
	uint64_t L_ATCU_HU_RrDrEVntConsBdryUp_X : 8;
	uint64_t L_ATCU_HU_RrDrEVntConsBdryUp_Y : 8;
	uint64_t L_ATCU_HU_RrDrEVntConsBdryLo_X : 8;
	uint64_t L_ATCU_HU_RrDrEVntConsBdryLo_Y : 8;
	uint64_t L_ATCU_HU_RrPsEVntConsBdryUP_X : 8;
	uint64_t L_ATCU_HU_RrPsEVntConsBdryUP_Y : 8;
	uint64_t L_ATCU_HU_RrPsEVntConsBdryLo_X : 8;
	uint64_t L_ATCU_HU_RrPsEVntConsBdryLo_Y : 8;	
}L_ATCU_25;

typedef union _atcu25_data{
	L_ATCU_25 data;
	uint8_t reg_data[8];
}Atcu_25_data;

typedef struct _l_atcu_26{			//Front Mode define from CTS position
	uint64_t L_ATCU_CTS_FrDrEVntSidePt_X : 11;
	uint64_t L_ATCU_CTS_FrDrEVntSidePt_Y : 11;
	uint64_t L_ATCU_CTS_FrDrEVntCtrPt_X : 11;
	uint64_t L_ATCU_CTS_FrDrEVntCtrPt_Y : 11;

	uint64_t reserved : 20;
}L_ATCU_26;

typedef union _atcu26_data{
	L_ATCU_26 data;
	uint8_t reg_data[8];
}Atcu_26_data;

typedef struct _l_atcu_27{			//Front Mode define from CTS position
	uint64_t L_ATCU_CTS_FrPsEVntCtrPt_X : 11;
	uint64_t L_ATCU_CTS_FrPsEVntCtrPt_Y : 11;
	uint64_t L_ATCU_CTS_FrPsEVntSidePt_X : 11;
	uint64_t L_ATCU_CTS_FrPsEVntSidePt_Y : 11;

	uint64_t reserved : 20;
}L_ATCU_27;

typedef union _atcu27_data{
	L_ATCU_27 data;
	uint8_t reg_data[8];
}Atcu_27_data;


typedef struct _l_atcu_28{			//Rear Mode define from CTS
	uint64_t L_ATCU_CTS_RrDrEVntConsPt_X : 11;
	uint64_t L_ATCU_CTS_RrDrEVntConsPt_Y : 11;
	uint64_t L_ATCU_CTS_RrPsEVntConsPt_X : 11;
	uint64_t L_ATCU_CTS_RrPsEVntConsPt_Y : 11;
	uint64_t L_ATCU_CTS_RrDrEVntModeSet : 4;
	uint64_t L_ATCU_CTS_RrPsEVntModeSet : 4;
	uint64_t L_ATCU_CTS_RrDrEVntConsOpnCls : 2;
	uint64_t L_ATCU_CTS_RrPsEVntConsOpnCls : 2;	
	
	uint64_t reserved : 8;
}L_ATCU_28;

typedef union _atcu28_data{
	L_ATCU_28 data;
	uint8_t reg_data[8];
}Atcu_28_data;

typedef struct _l_atcu_29{			//front Mode define from CTS
	uint64_t L_ATCU_CTS_FrDrEVntSideBdryUp_X : 8;
	uint64_t L_ATCU_CTS_FrDrEVntSideBdryUp_Y : 8;
	uint64_t L_ATCU_CTS_FrDrEVntSideBdryLo_X : 8;
	uint64_t L_ATCU_CTS_FrDrEVntSideBdryLo_Y : 8;
	uint64_t L_ATCU_CTS_FrDrEVntCtrBdryUp_X : 8;
	uint64_t L_ATCU_CTS_FrDrEVntCtrBdryUp_Y : 8;
	uint64_t L_ATCU_CTS_FrDrEVntCtrBdryLo_X : 8;
	uint64_t L_ATCU_CTS_FrDrEVntCtrBdryLo_Y : 8;	
	
	uint64_t reserved : 8;
}L_ATCU_29;

typedef union _atcu29_data{
	L_ATCU_29 data;
	uint8_t reg_data[8];
}Atcu_29_data;

typedef struct _l_atcu_30{			//front Mode define from CTS
	uint64_t L_ATCU_CTS_FrPsEVntSideBdryUp_X : 8;
	uint64_t L_ATCU_CTS_FrPsEVntSideBdryUp_Y : 8;
	uint64_t L_ATCU_CTS_FrPsEVntSideBdryLo_X : 8;
	uint64_t L_ATCU_CTS_FrPsEVntSideBdryLo_Y : 8;
	uint64_t L_ATCU_CTS_FrPsEVntCtrBdryUp_X : 8;
	uint64_t L_ATCU_CTS_FrPsEVntCtrBdryUp_Y : 8;
	uint64_t L_ATCU_CTS_FrPsEVntCtrBdryLo_X : 8;
	uint64_t L_ATCU_CTS_FrPsEVntCtrBdryLo_Y : 8;	
	
	uint64_t reserved : 8;
}L_ATCU_30;

typedef union _atcu30_data{
	L_ATCU_30 data;
	uint8_t reg_data[8];
}Atcu_30_data;

typedef struct _l_atcu_31{			//rear Mode define from CTS
	uint64_t L_ATCU_CTS_RrDrEVntConsBdryUp_X : 8;
	uint64_t L_ATCU_CTS_RrDrEVntConsBdryUp_Y : 8;
	uint64_t L_ATCU_CTS_RrDrEVntConsBdryLo_X : 8;
	uint64_t L_ATCU_CTS_RrDrEVntConsBdryLo_Y : 8;
	uint64_t L_ATCU_CTS_RrPsEVntConsBdryUp_X : 8;
	uint64_t L_ATCU_CTS_RrPsEVntConsBdryUp_Y : 8;
	uint64_t L_ATCU_CTS_RrPsEVntConsBdryLo_X : 8;
	uint64_t L_ATCU_CTS_RrPsEVntConsBdryLo_Y : 8;	
	
	uint64_t reserved : 8;
}L_ATCU_31;

typedef union _atcu31_data{
	L_ATCU_31 data;
	uint8_t reg_data[8];
}Atcu_31_data;

typedef struct _l_atcu_32{			//Front Mode define from P-OIP
	uint64_t L_ATCU_HU_FrDrEVntModeSet : 4;
	uint64_t L_ATCU_HU_FrPsEVntModeSet : 4;
	uint64_t L_ATCU_HU_FrDrEVntSlideOpnCls : 2;
	uint64_t L_ATCU_HU_FrDrEVntCtrOpnCls : 2;
	uint64_t L_ATCU_HU_FrPsEVntSideOpnCls : 2;
	uint64_t L_ATCU_HU_FrPsEVntCtrOpnCls : 2;
	
	uint64_t reserved : 24;
	uint64_t reserved1 : 24;
}L_ATCU_32;

typedef union _atcu32_data{
	L_ATCU_32 data;
	uint8_t reg_data[8];
}Atcu_32_data;

typedef struct _l_atcu_33{			//Front Mode define from CTS
	uint64_t reserved : 20;
	uint64_t reserved1 : 24;
	uint64_t L_ATCU_CTS_FrDrEVntModeSet : 4;
	uint64_t L_ATCU_CTS_FrPsEVntModeSet : 4;
	uint64_t L_ATCU_CTS_FrDrEVntSideOpenCls : 2;
	uint64_t L_ATCU_CTS_FrDrEVntCtrOpnCls : 2;
	uint64_t L_ATCU_CTS_FrPsEVntSideOpnCls : 2;
	uint64_t L_ATCU_CTS_FrPsEVntCtrOpnCls : 2;
	uint64_t reserved2 : 4;
}L_ATCU_33;

typedef union _atcu33_data{
	L_ATCU_33 data;
	uint8_t reg_data[8];
}Atcu_33_data;

///////////////////////////////////////////////////////
//CAN TX
typedef struct _tx_1_evnt{
	uint64_t L_EVNT_HU_FrDrEvntSidePtDisp_X : 11;
	uint64_t L_EVNT_HU_FrDrEvntSidePtDisp_Y : 11;
	uint64_t L_EVNT_HU_FrDrEvntCtrPtDisp_X : 11;
	uint64_t L_EVNT_HU_FrDrEvntCtrPtDisp_Y : 11;
	uint64_t reserved : 20;
}Tx_1_Evnt;

typedef union _lvnt_1_data{
	Tx_1_Evnt data;
	uint8_t reg_data[8];
}Lvnt_1_data;

typedef struct _tx_2_evnt{
	uint64_t L_EVNT_HU_FrPsEvntSidePtDisp_X : 11;
	uint64_t L_EVNT_HU_FrPsEvntSidePtDisp_Y : 11;
	uint64_t L_EVNT_HU_FrPsEvntCtrPtDisp_X : 11;
	uint64_t L_EVNT_HU_FrPsEvntCtrPtDisp_Y : 11;
	uint64_t reserved : 20;
}Tx_2_Evnt;

typedef union _lvnt_2_data{
	Tx_2_Evnt data;
	uint8_t reg_data[8];
}Lvnt_2_data;

typedef struct _tx_3_evnt{
	uint64_t L_EVNT_HU_RrDrEvntConsPtDisp_X : 11;
	uint64_t L_EVNT_HU_RrDrEvntConsPtDisp_Y : 11;
	uint64_t L_EVNT_HU_RrPsEvntConsPtDisp_X : 11;
	uint64_t L_EVNT_HU_RrPsEvntConsPtDisp_Y : 11;

	uint64_t L_EVNT_FrDrEvntSideFlt : 2;
	uint64_t L_EVNT_FrDrEvntCtrFlt : 2;
	uint64_t L_EVNT_FrPsEvntSideFlt : 2;
	uint64_t L_EVNT_FrPsEvntCtrFlt : 2;
	uint64_t L_EVNT_RrDrEvntConsFlt : 2;
	uint64_t L_EVNT_RrPsEvntConsFlt : 2;
	uint64_t reserved : 8;
}Tx_3_Evnt;

typedef union _lvnt_3_data{
	Tx_3_Evnt data;
	uint8_t reg_data[8];
}Lvnt_3_data;

typedef struct _tx_4_evnt{
	uint64_t L_EVNT_FrDrEvntModeSet : 4;
	uint64_t L_EVNT_FrPsEvntModeSet : 4;
	uint64_t L_EVNT_RrDrEvntModeSet : 4;
	uint64_t L_EVNT_RrPsEvntModeSet : 4;

	uint64_t L_EVNT_FrDrEvntSideOpnCls : 2;
	uint64_t L_EVNT_FrDrEvntCtrOpnCls : 2;
	uint64_t L_EVNT_FrPsEvntSideOpnCls : 2;
	uint64_t L_EVNT_FrPsEvntCtrOpnCls : 2;
	uint64_t L_EVNT_RrDrEvntConsOpnCls : 2;
	uint64_t L_EVNT_RrPsEvntConsOpnCls : 2;

	uint64_t L_EVNT_EvntTouchInform_HU : 2;
	uint64_t L_EVNT_EvntTouchInform_CTS : 2;
	uint64_t reserved : 16;
	uint64_t reserved1 : 16;
}Tx_4_Evnt;

typedef union _lvnt_4_data{
	Tx_4_Evnt data;
	uint8_t reg_data[8];
}Lvnt_4_data;

typedef struct _tx_5_evnt{
	uint64_t L_EVNT_CTS_FrDrEvntSidePtDisp_X : 11;
	uint64_t L_EVNT_CTS_FrDrEvntSidePtDisp_Y : 11;
	uint64_t L_EVNT_CTS_FrDrEvntCtrPtDisp_X : 11;
	uint64_t L_EVNT_CTS_FrDrEvntCtrPtDisp_Y : 11;
	uint64_t reserved : 20;
}Tx_5_Evnt;

typedef union _lvnt_5_data{
	Tx_5_Evnt data;
	uint8_t reg_data[8];
}Lvnt_5_data;

typedef struct _tx_6_evnt{
	uint64_t L_EVNT_CTS_FrPsEvntSidePtDisp_X : 11;
	uint64_t L_EVNT_CTS_FrPsEvntSidePtDisp_Y : 11;
	uint64_t L_EVNT_CTS_FrPsEvntCtrPtDisp_X : 11;
	uint64_t L_EVNT_CTS_FrPsEvntCtrPtDisp_Y : 11;
	uint64_t reserved : 20;
}Tx_6_Evnt;

typedef union _lvnt_6_data{
	Tx_6_Evnt data;
	uint8_t reg_data[8];
}Lvnt_6_data;

typedef struct _tx_7_evnt{
	uint64_t L_EVNT_CTS_RrDrEvntConsPtDisp_X : 11;
	uint64_t L_EVNT_CTS_RrDrEvntConsPtDisp_Y : 11;
	uint64_t L_EVNT_CTS_RrPsEvntConsPtDisp_X : 11;
	uint64_t L_EVNT_CTS_RrPsEvntConsPtDisp_Y : 11;
	uint64_t reserved : 20;
}Tx_7_Evnt;

typedef union _lvnt_7_data{
	Tx_7_Evnt data;
	uint8_t reg_data[8];
}Lvnt_7_data;

typedef struct _tx_8_evnt{
	uint64_t L_EVNT_ATCUComFlt : 2;
	uint64_t L_EVNT_EVNTFlt : 2;
	uint64_t L_EVNT_DrvSdVertComFlt : 2;  //LIN-BUS OFF 
	uint64_t L_EVNT_DrvSdHorComFlt : 2;
	uint64_t L_EVNT_DrvCntrVertComFlt : 2;
	uint64_t L_EVNT_DrvCntrHorComFlt : 2;
	uint64_t L_EVNT_AstSdVertComFlt : 2;
	uint64_t L_EVNT_AstSdHorComFlt : 2;
	uint64_t L_EVNT_AstCntrVertComFlt : 2;
	uint64_t L_EVNT_AstCntrHorComFlt : 2;
	uint64_t L_EVNT_RrDrvCnsVertComFlt : 2;
	uint64_t L_EVNT_RrDrvCnsHorComFlt : 2;
	uint64_t L_EVNT_RrAstCnsVertComFlt : 2;
	uint64_t L_EVNT_RrAstCnsHorComFlt : 2;
	uint64_t L_EVNT_DrvSdVertIntFlt : 2;
	uint64_t L_EVNT_DrvSdHorIntFlt : 2;
	uint64_t L_EVNT_DrvCntrVertIntFlt : 2;
	uint64_t L_EVNT_DrvCntrHorIntFlt : 2;
	uint64_t L_EVNT_AstSdVertIntFlt : 2;
	uint64_t L_EVNT_AstSdHorIntFlt : 2;
	uint64_t L_EVNT_AstCntrVertIntFlt : 2;
	uint64_t L_EVNT_AstCntrHorIntFlt : 2;
	uint64_t L_EVNT_RrDrCnsVertIntFlt : 2;
	uint64_t L_EVNT_RrDrCnsHorIntFlt : 2;
	uint64_t L_EVNT_RrAstCnsVertIntFlt : 2;
	uint64_t L_EVNT_RrAstCnsHorIntFlt : 2;
	uint64_t L_EVNT_DrvSdVertExtFlt : 2;
	uint64_t L_EVNT_DrvSdHorExtFlt : 2;
	uint64_t L_EVNT_DrvCntrVertExtFlt : 2;
	uint64_t L_EVNT_DrvCntrHorExtFlt : 2;
	uint64_t L_EVNT_AstSdVertExtFlt : 2;
	uint64_t L_EVNT_AstSdHorExtFlt : 2;
}Tx_8_Evnt;

typedef union _lvnt_8_data{
	Tx_8_Evnt data;
	uint8_t reg_data[8];
}Lvnt_8_data;

typedef struct _tx_9_evnt{
	uint64_t L_EVNT_AstCntrVertExtFlt : 2;
	uint64_t L_EVNT_AstCntrHorExtFlt : 2;
	uint64_t L_EVNT_RrDrvCnsVertExtFlt : 2;
	uint64_t L_EVNT_RrDrvCnsHorExtFlt : 2;
	uint64_t L_EVNT_RrAstCnsVertExtFlt : 2;
	uint64_t L_EVNT_RrAstCnsHorExtFlt : 2;

	uint64_t reserved : 27;
	uint64_t reserved1 : 27;
}Tx_9_Evnt;

typedef union _lvnt_9_data{
	Tx_9_Evnt data;
	uint8_t reg_data[8];
}Lvnt_9_data;

///////////////////////////////////////////////////////

typedef enum _drivermode{
	OFF,
	VENT,
	B_L,
	FLOOR,
	MODE7_DEF_FLOOR,
	MAX_DEF,
	AUTO_DEFOG,
	MODE7_DEF_VENT,
	MODE_AUTO,
	MODE7_DEF_VENT_FLOOR,
	MODE7_DEF,
	DEF_VENT_ARROW,
	DEF_FLOOR_ARROW,
	DEF_VENT_FLOOR_ARROW,
	DEF,
	ERROR_INDICATOR,	
}Drivermode;

typedef enum _amotimer{
	timer_0,
	timer_1,
	timer_2,
	timer_3,
	timer_4,
	timer_5,
	timer_6,
	timer_7,
	timer_8,
	timer_9,
	timer_10,
	timer_11,
	timer_12,
	timer_13,
	timer_14,
	timer_15,
	timer_16,
	timer_17,
	timer_18,
	timer_19,
	timer_20,
	timer_21,
	timer_22,
	timer_23,
	timer_24,
	timer_25,
	timer_26,
	timer_27,
	timer_28,
	timer_29,
	timer_30,	
	timer_31,
	timer_32,
	timer_33,
	timer_34,
	timer_35,
	timer_36,
	timer_37,
	timer_38,
	timer_39,
	timer_40,	
	timer_41,
	timer_42,
	timer_43,
	timer_44,
	timer_45,
	timer_46,
	timer_47,
	timer_48,
	timer_49,
	timer_50,
	timer_51,
	timer_52,
	timer_53,
	timer_54,
	timer_55,
	timer_56,
	timer_57,
	timer_58,
	timer_59,
	timer_60,
	timer_61,
	timer_62,
	timer_63,
	timer_64,
	timer_65,
	timer_66,
	timer_67,
	timer_68,
	timer_69, //not used
	timer_70,
	timer_71,
	timer_72,
	timer_73,
	timer_74,
	timer_75,
	timer_76,
	timer_77,
	timer_78,
	timer_79,
	timer_80,
	timer_81,
	timer_82,
	timer_83,
	timer_84,
	timer_85,
	timer_86,
	timer_87,
	timer_88,
	timer_89,
	timer_90, //sleep
	timer_91, //cllr
	timer_92,
	timer_93, //cllr
	timer_94, //cllr
	timer_95, //cllr
	timer_96, //clud
	timer_97, //clud
	timer_98, //clud
	timer_99,  //clud
	timer_100, //sllr
	timer_101,  //slud
	timer_102, //crlr
	timer_103, //crud
	timer_104, //srlr
	timer_105, //srud
	timer_106,//sllr
	timer_107,//sllr
	timer_108,//sllr
	timer_109,//sllr
	timer_110,//sllr
	timer_111,//sllr
	timer_112,//slud
	timer_113,//slud
	timer_114,//slud
	timer_115,//slud
	timer_116,//slud
	timer_117,//slud
	timer_118,//crlr
	timer_119,//crlr
	timer_120,//crlr
	timer_121,//crlr
	timer_122,//crlr
	timer_123,//crlr
	timer_124,//crud
	timer_125,//crud
	timer_126,//crud
	timer_127,//crud
	timer_128,//crud
	timer_129,//crud
	timer_130,//srlr
  timer_131,//srlr
  timer_132,//srlr
  timer_133,//srlr
  timer_134,//srlr
  timer_135,//srlr
  timer_136,//srud
  timer_137,//srud
  timer_138,//srud
  timer_139,//srud
  timer_140,//srud
  timer_141,//srud
  timer_142,
  timer_143,
  timer_144,
  timer_145,
  timer_146,
  timer_147,
  timer_148,
  timer_149,
  timer_150,
  timer_151,
  timer_152,
  timer_153,
  timer_154,
  timer_155,
  timer_156,
  timer_157,
  timer_158,
  timer_159,
  timer_160,
  timer_161,
  timer_162,
  timer_163,
  timer_164,
  timer_165,
  timer_166,
  timer_167,
  timer_168,
  timer_169,
  timer_170
}Amotimer;

typedef enum _can_tx_id{
	L_EVNT_01 = 0x401,
	L_EVNT_02 = 0x402,
	L_EVNT_03,
	L_EVNT_04,
	L_EVNT_05,
	L_EVNT_06,
	L_EVNT_07,
	L_EVNT_08,
	L_EVNT_09,
}Can_tx_id;

typedef enum _profile_mode{
	DEFAULT_PROFILE = 0,
	GUEST,
	PROFILE_1,
	PROFILE_2,
	PROFILE_3,
	PROFILE_4,
	PROFILE_5,
	PROFILE_6,
	PROFILE_7,
	PROFILE_8,
	PROFILE_9,	
}Profile_mode;

typedef enum _drv_type{
	LHD,
	RHD,
}Drv_type;

#if 1 //def AMO_DTC_SETTING
typedef struct{
	uint8_t high_byte;
	uint8_t middle_byte;
	uint8_t low_byte;
}DTC_Format;

typedef struct{
	uint8_t first_byte;
	uint8_t second_byte;
	uint8_t third_byte;
	uint8_t fourth_byte;
}SW_Format;

typedef enum _dtc_id{
	U035188 = 0xC35188,
	U035288 = 0xC35288,
	U035388 = 0xC35388,
	U035488 = 0xC35488,
	U035588 = 0xC35588,
	U035688 = 0xC35688,
	U035788 = 0xC35788,
	U035888 = 0xC35888,
	U035988 = 0xC35988,
	U036088 = 0xC36088,
	U036188 = 0xC36188,
	U036288 = 0xC36288,
	B188996 = 0x988996,
	B188A96 = 0x988A96,
	B188B96 = 0x988B96,
	B188C96 = 0x988C96,
	B188D96 = 0x988D96,
	B188E96 = 0x988E96,
	B190A96 = 0x990A96,
	B190B96 = 0x990B96,
	B190C96 = 0x990C96,
	B190D96 = 0x990D96,
	B190E96 = 0x990E96,
	B190F96 = 0x990F96,
	B188907 = 0x988907,
	B188A07 = 0x988A07,
	B188B07 = 0x988B07,
	B188C07 = 0x988C07,
	B188D07 = 0x988D07,
	B188E07 = 0x988E07,
	B190A07 = 0x990A07,
	B190B07 = 0x990B07,
	B190C07 = 0x990C07,
	B190D07 = 0x990D07,
	B190E07 = 0x990E07,
	B190F07 = 0x990F07,
	B188804 = 0x988804,
}DTC_ID;
#endif

typedef struct _can_rx_info{
	uint8_t start_info;
	uint8_t rx_off;
}Can_rx_info;

typedef struct{
	uint32_t signature;
	uint32_t app_key;
	uint32_t version;
	uint32_t app_size;
	uint32_t crc;
}Fw_header;
/******************************************************************************
 * Vent mode structure
 ******************************************************************************/
 

/******************************************************************************
 * Function prototypes
 ******************************************************************************/

//void Amo_Can_Init();
void Evt_queue_init();
char Evt_queue_full();
char Evt_queue_full();
char Evt_queue_empty();
char Evt_queue_add(unsigned long long m_data);
unsigned long long Evt_queue_value();
void SendCANData(uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len);
void Amo_Can_Init();
void Init_CanTx_Parameter(void);
void Can_Rx_TimeOut(void);
void Evnt_Atcu_tx_data(void);
void CAN_Recovery_TX(void);

//////////////////////////////////////////////////////////////////
//CTS
void Cts_Front_mode_setting(void);
void Cts_Rear_mode_setting(void);

void Last_Setting_Save(Profile_mode Profile_mode);
void Last_Setting_Setup(Profile_mode Profile_mode);
void Last_Mode_dr_fr_set_func(void);

void Default_Setting_Setup(Profile_mode Profile_mode);
void Hu_Front_Focus_mode_setting_Backup();
void Hu_Front_Spread_mode_setting_Backup();
void Hu_Front_FullClose_mode_setting_Backup();

#if 1 //def AMO_DTC_SETTING
void DTC_Save_Code(DTC_ID dtc);
void DTC_Erase_Code(DTC_ID dtc);
void DTC_Read_Code(void);
#endif
void Evnt_Atcu_Diagmsg(void);
void DTC_Send_event(void);
void Current_Setting_Setup();
void DTC_OTA_event(void);
#endif /* __AMO_CAN_H */


