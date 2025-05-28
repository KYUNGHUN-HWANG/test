#ifndef AMO_CAN_C
#define AMO_CAN_C

/*===========================================================================*/
/* Project   :  AMOSENSE CAN driver Software                                                                */
/* File name :  Amo_CAN_Parsing.c                                                                             */
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
//============================================================================ 

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

#include "Amo_CAN_Parsing.h"
#include "Amo_Mode_Setting.h"
#include "ewl_misra_types.h"
#include "Amo_STATE_Event.h"
#include "Amo_main.h"
#include "Amo_timer.h"
#include "Amo_nvm.h"
#include "Amo_Lin.h"
#include "Amo_Cycle.h"
#include "Amo_Sleep.h"
#include "Amo_Adc.h"
#include "Amo_Flash.h"
#include "Amo_status.h"
#include <errno.h>
#include <limits.h>
#include "Amo_Vent_mode.h"
#include "Amo_Last_mode.h"
/******************************************************************************
 * Groval variable prototypes
 ******************************************************************************/
static uint8_t SW_VERSION[] = "A.00";
static uint8_t HW_VERSION[] = "A.00";

unsigned long long vent_evt[256];
unsigned int q_front = 0, q_rear = 0;

uint8_t rxMBdone = 0;
volatile uint8_t rxFIFOdone = 0;
uint8_t Can_rx_success = 0;

/* Define user receive buffer */
flexcan_msgbuff_t recvBuff1, recvBuff2;
uint8_t Evnt_Network_Release = 0;
uint32_t nv_max_cnt = 0;
uint8_t rx_id_count = 0;
/******************************************************************************
 * Local variable Define
 ******************************************************************************/

/* ID Filter table */
flexcan_id_table_t filterTable[24];

uint16_t IDlist[24] = {0x22,0x23,0x24,0x25,0x28,0x29,0x111,/*0x112,0x115,*/0x119,0x131,0x132,0x122,0x123,0x124,0x125,0x126,0x127,0x128,0x129,0x130,0x161,0x162,0x505,0,0};
//uint16_t IDmask[10] = {0x7E0,0x7F0,0x7FC,0x7F0,0x7FF,0x7FC,0x7E0,0x7FC,0x7F8,0x7F8};
uint8_t Tx_candata[8];
unsigned int can_error_data = 0;
uint8_t dtc_serviceId = 0;
uint8_t dtc_data_length = 0;
uint16_t dtc_Total_length = 0;
uint16_t sn_number = 0;
uint16_t sn_index = 0;
uint8_t dtc_subFunc = 0;		
uint8_t dtcstatus = 0;
uint8_t Rx_candata[8];
uint8_t rx_len = 0;

Can_rx_info Rx_info_data[20];
uint8_t RX_Status[20];
uint8_t Write_Buffer[4096];
uint8_t expected_sn_number = 0;
uint16_t chksum_data = 0;
uint8_t AIR_VENT_DL_MODE = 0;

volatile uint32_t flash_new_address = 0x00000000;
volatile uint32_t flash_address = 0x00000000;
volatile uint32_t new_header_address = 0x00000000;
volatile uint32_t old_header_address = 0x00000000;

uint8_t P_type = 0;

/******************************************************************************
 * CAN DATA Groval variable prototypes
 ******************************************************************************/
uint8_t Vent_Mode_FrDrOnOff = 0;
uint8_t Vent_Mode_FrPassOnOff = 0;
#ifdef AMO_GN7_PE_SETTING_NONE
uint8_t Vent_Mode_RrDrOnOff = 0;
uint8_t Vent_Mode_RrPassOnOff = 0;
#endif

uint8_t can_rx_2_expire = 0, can_rx_3_expire = 0, can_rx_4_expire = 0, can_rx_5_expire = 0, can_rx_8_expire = 0, can_rx_9_expire = 0, can_rx_12_expire = 0, can_rx_13_expire = 0, can_rx_16_expire = 0;
uint8_t can_rx_23_expire = 0, can_rx_24_expire = 0, can_rx_25_expire = 0, can_rx_29_expire = 0, can_rx_30_expire = 0, can_rx_31_expire = 0;

extern uint8_t Evnt_IGN2_Onoff;
#ifdef AMO_GN7_PE_SETTING_NONE
extern uint8_t Evnt_IGN3_Onoff;
#endif
extern uint8_t key_full_left_close;
extern uint8_t key_full_right_close;

uint8_t P_Oip_Cts_priority = 0;

uint8_t Datc_onoff = 0;

volatile TaskProiority P_Oip_Cts_Select = P_OIP_SELECT;	//NONE_SELECT;			//P_OIP, CTS Priority
volatile Touch_mode_Dr Touch_mode_Dr_set = NONE_DRIVER;
volatile Touch_mode_Ps Touch_mode_Ps_set = NONE_PASSENGER;
#ifdef AMO_GN7_PE_SETTING_NONE
volatile Touch_mode_Rr Touch_mode_Rr_set = NONE_REAR;
#endif

int16_t Fr_Dr_side_leftright_target_backup = 0;
int16_t Fr_Dr_side_updown_target_backup = 0;
int16_t Fr_Dr_ctr_leftright_target_backup = 0;
int16_t Fr_Dr_ctr_updown_target_backup = 0;
int16_t Fr_Ps_side_leftright_target_backup = 0;
int16_t Fr_Ps_side_updown_target_backup = 0;
int16_t Fr_Ps_ctr_leftright_target_backup = 0;
int16_t Fr_Ps_ctr_updown_target_backup = 0;
#ifdef AMO_GN7_PE_SETTING_NONE
int16_t Rr_Dr_side_leftright_target_backup = 0;
int16_t Rr_Dr_side_updown_targe_backup = 0;
int16_t Rr_Ps_ctr_leftright_target_backup = 0;
int16_t Rr_Ps_ctr_updown_target_backup = 0;
#endif

volatile Profile_mode last_mode_save = DEFAULT_PROFILE;
Profile_mode lvnt_current_mode = DEFAULT_PROFILE;
uint8_t evnt_mode_setting = 1;
#ifdef AMO_DTC_SETTING
uint8_t Dtc_data[200];
#endif
uint8_t pci = 0;
uint32_t Fw_total_size = 0;
uint8_t verString[10];
uint8_t HexString[10];
///////////////////////////////////////////////////////////////////////////////
Atcu_2_data Can_data_2_Info;

Drivermode ATCU_Fr_Dr_Mode_display = OFF;
Drivermode ATCU_Fr_Ps_Mode_display = OFF;

///////////////////////////////////////////////////////////////////////////////
Atcu_3_data Can_data_3_Info;

uint16_t ATCU_RheosatLevelStatue = 20;		//reserved, step 1,2,~ detent.... Init value (step 20)

///////////////////////////////////////////////////////////////////////////////

Atcu_4_data Can_data_4_Info;

uint8_t ATCU_OperationStatus = 0;			//0 : DATC off(Power down), 1: DATC Normal, 2:DATC System Off(Blower Off), 3:DATC Self Diagnosis, 4: Vesion Display, 5:Not used , 6: Not used, 7: Error Indicator  

///////////////////////////////////////////////////////////////////////////////
#ifdef AMO_GN7_PE_SETTING_NONE
Atcu_5_data Can_data_5_Info;

uint8_t ATCU_RearOperationStatus = 0;
Drivermode ATCU_Rr_Dr_Mode_display = OFF;
Drivermode ATCU_Rr_Ps_Mode_display = OFF;
#endif
///////////////////////////////////////////////////////////////////////////////
Atcu_8_data Can_data_8_Info;

uint8_t ATCU_CamModeStatus = 0;
uint8_t ATCU_UtilityModeUSMStatus = 0;

///////////////////////////////////////////////////////////////////////////////
Atcu_9_data Can_data_9_Info;

uint8_t ATCU_DetentOutStatus = 0;			//Detent off = 0, Detent On = 1
uint8_t ATCU_AutoBrightStatus = 0;
uint8_t ATCU_NotMinimumModeBrightStatus = 0;
volatile uint8_t L_ATCU_IAUProfileValue = 0;
volatile uint8_t L_ATCU_AVNProfileValue = 0;

///////////////////////////////////////////////////////////////////////////////
Atcu_12_data Can_data_12_Info;

volatile Drv_type ATCU_DriverSideType = LHD;			//0x00 : LHD, 0x01 : RHD
uint8_t ATCU_FrBlwLvl = 0;
uint8_t ATCU_PassBlwLvl = 0;

///////////////////////////////////////////////////////////////////////////////
Atcu_20_data Can_data_20_Info;	//P-OIP RECEIVED Front driver
Atcu_20_data Can_data_20_Info_invalid;

uint16_t Hu_FrDrEVntSidePt_X = 0;
uint16_t Hu_FrDrEVntSidePt_Y = 0;
uint16_t Hu_FrDrEVntCtrPt_X = 0;
uint16_t Hu_FrDrEVntCtrPt_Y = 0;

uint16_t Hu_FrDrEVntSidePt_X_invalid = 2047;
uint16_t Hu_FrDrEVntSidePt_Y_invalid = 2047;
uint16_t Hu_FrDrEVntCtrPt_X_invalid = 2047;
uint16_t Hu_FrDrEVntCtrPt_Y_invalid = 2047;

uint16_t Hu_FrDrEVntPt_success = 0;

uint16_t Hu_FrDrEVntSidePt_X_backup = 0;
uint16_t Hu_FrDrEVntSidePt_Y_backup = 0;
uint16_t Hu_FrDrEVntCtrPt_X_backup = 0;
uint16_t Hu_FrDrEVntCtrPt_Y_backup = 0;

uint16_t Hu_FrDrEVntSidePt_X_reserve = 100;
uint16_t Hu_FrDrEVntSidePt_Y_reserve = 125;
uint16_t Hu_FrDrEVntCtrPt_X_reserve = 100;
uint16_t Hu_FrDrEVntCtrPt_Y_reserve = 125;

uint16_t Hu_FrDrEVntSideCurrentPt_X = 0;
uint16_t Hu_FrDrEVntSideCurrentPt_Y = 0;
uint16_t Hu_FrDrEVntCtrCurrentPt_X = 0;
uint16_t Hu_FrDrEVntCtrCurrentPt_Y = 0;
///////////////////////////////////////////////////////////////////////////////
Atcu_21_data Can_data_21_Info; //P-OIP RECEIVED Front passenger
Atcu_21_data Can_data_21_Info_invalid;

uint16_t Hu_FrPsEVntSidePt_X = 0;
uint16_t Hu_FrPsEVntSidePt_Y = 0;
uint16_t Hu_FrPsEVntCtrPt_X = 0;
uint16_t Hu_FrPsEVntCtrPt_Y = 0;

uint16_t Hu_FrPsEVntSidePt_X_invalid = 2047;
uint16_t Hu_FrPsEVntSidePt_Y_invalid = 2047;
uint16_t Hu_FrPsEVntCtrPt_X_invalid = 2047;
uint16_t Hu_FrPsEVntCtrPt_Y_invalid = 2047;

uint16_t Hu_FrPsEVntSidePt_success = 0;

uint16_t Hu_FrPsEVntSidePt_X_backup = 0;
uint16_t Hu_FrPsEVntSidePt_Y_backup = 0;
uint16_t Hu_FrPsEVntCtrPt_X_backup = 0;
uint16_t Hu_FrPsEVntCtrPt_Y_backup = 0;

uint16_t Hu_FrPsEVntSidePt_X_reserve = 100;
uint16_t Hu_FrPsEVntSidePt_Y_reserve = 125;
uint16_t Hu_FrPsEVntCtrPt_X_reserve = 100;
uint16_t Hu_FrPsEVntCtrPt_Y_reserve = 125;

uint16_t Hu_FrPsEVntSideCurrentPt_X = 0;
uint16_t Hu_FrPsEVntSideCurrentPt_Y = 0;
uint16_t Hu_FrPsEVntCtrCurrentPt_X = 0;
uint16_t Hu_FrPsEVntCtrCurrentPt_Y = 0;

///////////////////////////////////////////////////////////////////////////////

Atcu_20_data_Key Can_data_20_Info_Key;	//P-OIP RECEIVED Front driver _KEY
uint16_t Hu_FrDrEVntSidePt_X_Key = 0;
uint16_t Hu_FrDrEVntSidePt_Y_Key = 0;
uint16_t Hu_FrDrEVntCtrPt_X_Key = 0;
uint16_t Hu_FrDrEVntCtrPt_Y_Key = 0;

///////////////////////////////////////////////////////////////////////////////

Atcu_21_data_Key Can_data_21_Info_Key; //P-OIP RECEIVED Front passenger _KEY
uint16_t Hu_FrPsEVntSidePt_X_Key = 0;
uint16_t Hu_FrPsEVntSidePt_Y_Key = 0;
uint16_t Hu_FrPsEVntCtrPt_X_Key = 0;
uint16_t Hu_FrPsEVntCtrPt_Y_Key = 0;

uint8_t button_LH_cycle_check;
uint8_t button_RH_cycle_check;

///////////////////////////////////////////////////////////////////////////////
#ifdef AMO_GN7_PE_SETTING_NONE
Atcu_22_data Can_data_22_Info; //P-OIP RECEIVED Rear driver/passenger

volatile Evnt_mode Hu_Rr_Dr_Vent_Mode = MODE_OFF;
volatile Evnt_mode Hu_Rr_Ps_Vent_Mode = MODE_OFF;
Evnt_mode Hu_Rr_Dr_Vent_Mode_Remember = DEFAULT_MODE;
Evnt_mode Hu_Rr_Ps_Vent_Mode_Remember = DEFAULT_MODE;

uint8_t Hu_Rr_Dr_Vent_signal = 0;
uint8_t Hu_Rr_Ps_Vent_signal = 0;

uint16_t Hu_RrDrEVntConsPt_X = 0;
uint16_t Hu_RrDrEVntConsPt_Y = 0;
uint16_t Hu_RrPsEVntConsPt_X = 0;
uint16_t Hu_RrPsEVntConsPt_Y = 0;

uint16_t Hu_RrDrEVntConsPt_X_backup = 0;
uint16_t Hu_RrDrEVntConsPt_Y_backup = 0;
uint16_t Hu_RrPsEVntConsPt_X_backup = 0;
uint16_t Hu_RrPsEVntConsPt_Y_backup = 0;

uint16_t Hu_RrDrEVntConsCurrentPt_X = 0;
uint16_t Hu_RrDrEVntConsCurrentPt_Y = 0;
uint16_t Hu_RrPsEVntConsCurrentPt_X = 0;
uint16_t Hu_RrPsEVntConsCurrentPt_Y = 0;
#endif

///////////////////////////////////////////////////////////////////////////////
Atcu_23_data Can_data_23_Info;

uint16_t Hu_Fr_Dr_Vent_Side_Bdry_Up_X = 0;
uint16_t Hu_Fr_Dr_Vent_Side_Bdry_Lo_X = 0;
uint16_t Hu_Fr_Dr_Vent_Side_Bdry_Up_Y = 0;
uint16_t Hu_Fr_Dr_Vent_Side_Bdry_Lo_Y = 0;
uint16_t Hu_Fr_Dr_Vent_Ctr_Bdry_Up_X = 0;
uint16_t Hu_Fr_Dr_Vent_Ctr_Bdry_Lo_X = 0;
uint16_t Hu_Fr_Dr_Vent_Ctr_Bdry_Up_Y = 0;
uint16_t Hu_Fr_Dr_Vent_Ctr_Bdry_Lo_Y = 0;

///////////////////////////////////////////////////////////////////////////////
Atcu_24_data Can_data_24_Info;

uint16_t Hu_Fr_Ps_Vent_Side_Bdry_Up_X = 0;
uint16_t Hu_Fr_Ps_Vent_Side_Bdry_Lo_X = 0;
uint16_t Hu_Fr_Ps_Vent_Side_Bdry_Up_Y = 0;
uint16_t Hu_Fr_Ps_Vent_Side_Bdry_Lo_Y = 0;
uint16_t Hu_Fr_Ps_Vent_Ctr_Bdry_Up_X = 0;
uint16_t Hu_Fr_Ps_Vent_Ctr_Bdry_Lo_X = 0;
uint16_t Hu_Fr_Ps_Vent_Ctr_Bdry_Up_Y = 0;
uint16_t Hu_Fr_Ps_Vent_Ctr_Bdry_Lo_Y = 0;

///////////////////////////////////////////////////////////////////////////////
#ifdef AMO_GN7_PE_SETTING_NONE
Atcu_25_data Can_data_25_Info;

uint16_t Hu_Rr_Dr_Vent_Cons_Bdry_Up_X = 0;
uint16_t Hu_Rr_Dr_Vent_Cons_Bdry_Lo_X = 0;
uint16_t Hu_Rr_Dr_Vent_Cons_Bdry_Up_Y = 0;
uint16_t Hu_Rr_Dr_Vent_Cons_Bdry_Lo_Y = 0;
uint16_t Hu_Rr_Ps_Vent_Cons_Bdry_Up_X = 0;
uint16_t Hu_Rr_Ps_Vent_Cons_Bdry_Lo_X = 0;
uint16_t Hu_Rr_Ps_Vent_Cons_Bdry_Up_Y = 0;
uint16_t Hu_Rr_Ps_Vent_Cons_Bdry_Lo_Y = 0;
#endif

///////////////////////////////////////////////////////////////////////////////
Atcu_26_data Can_data_26_Info;  //CTS RECEIVED      

uint16_t Cts_FrDrEVntSidePt_X = 0;
uint16_t Cts_FrDrEVntSidePt_Y = 0;
uint16_t Cts_FrDrEVntCtrPt_X = 0;
uint16_t Cts_FrDrEVntCtrPt_Y = 0;

///////////////////////////////////////////////////////////////////////////////
Atcu_27_data Can_data_27_Info;  //CTS RECEIVED 

uint16_t Cts_FrPsEVntSidePt_X = 0;
uint16_t Cts_FrPsEVntSidePt_Y = 0;
uint16_t Cts_FrPsEVntCtrPt_X = 0;
uint16_t Cts_FrPsEVntCtrPt_Y = 0;

///////////////////////////////////////////////////////////////////////////////
#ifdef AMO_GN7_PE_SETTING_NONE
Atcu_28_data Can_data_28_Info;  //CTS RECEIVED 

Evnt_mode Cts_Rr_Dr_Vent_Mode = DEFAULT_MODE;
Evnt_mode Cts_Rr_Ps_Vent_Mode = DEFAULT_MODE;
Evnt_mode Cts_Rr_Dr_Vent_Mode_remember = DEFAULT_MODE;
Evnt_mode Cts_Rr_Ps_Vent_Mode_remember = DEFAULT_MODE;

uint8_t Cts_Rr_Dr_Vent_siganl = 0;
uint8_t Cts_Rr_Ps_Vent_siganl = 0;

uint16_t Cts_RrDrEVntConsPt_X = 0;
uint16_t Cts_RrDrEVntConsPt_Y = 0;
uint16_t Cts_RrPsEVntConsPt_X = 0;
uint16_t Cts_RrPsEVntConsPt_Y = 0;
#endif

///////////////////////////////////////////////////////////////////////////////
Atcu_29_data Can_data_29_Info;

uint16_t Cts_Fr_Dr_Vent_Side_Bdry_Up_X = 0;
uint16_t Cts_Fr_Dr_Vent_Side_Bdry_Lo_X = 0;
uint16_t Cts_Fr_Dr_Vent_Side_Bdry_Up_Y = 0;
uint16_t Cts_Fr_Dr_Vent_Side_Bdry_Lo_Y = 0;
uint16_t Cts_Fr_Dr_Vent_Ctr_Bdry_Up_X = 0;
uint16_t Cts_Fr_Dr_Vent_Ctr_Bdry_Lo_X = 0;
uint16_t Cts_Fr_Dr_Vent_Ctr_Bdry_Up_Y = 0;
uint16_t Cts_Fr_Dr_Vent_Ctr_Bdry_Lo_Y = 0;

///////////////////////////////////////////////////////////////////////////////
Atcu_30_data Can_data_30_Info;

uint16_t Cts_Fr_Ps_Vent_Side_Bdry_Up_X = 0;
uint16_t Cts_Fr_Ps_Vent_Side_Bdry_Lo_X = 0;
uint16_t Cts_Fr_Ps_Vent_Side_Bdry_Up_Y = 0;
uint16_t Cts_Fr_Ps_Vent_Side_Bdry_Lo_Y = 0;
uint16_t Cts_Fr_Ps_Vent_Ctr_Bdry_Up_X = 0;
uint16_t Cts_Fr_Ps_Vent_Ctr_Bdry_Lo_X = 0;
uint16_t Cts_Fr_Ps_Vent_Ctr_Bdry_Up_Y = 0;
uint16_t Cts_Fr_Ps_Vent_Ctr_Bdry_Lo_Y = 0;

///////////////////////////////////////////////////////////////////////////////
#ifdef AMO_GN7_PE_SETTING_NONE
Atcu_31_data Can_data_31_Info;

uint16_t Cts_Rr_Dr_Vent_Cons_Bdry_Up_X = 0;
uint16_t Cts_Rr_Dr_Vent_Cons_Bdry_Lo_X = 0;
uint16_t Cts_Rr_Dr_Vent_Cons_Bdry_Up_Y = 0;
uint16_t Cts_Rr_Dr_Vent_Cons_Bdry_Lo_Y = 0;
uint16_t Cts_Rr_Ps_Vent_Cons_Bdry_Up_X = 0;
uint16_t Cts_Rr_Ps_Vent_Cons_Bdry_Lo_X = 0;
uint16_t Cts_Rr_Ps_Vent_Cons_Bdry_Up_Y = 0;
uint16_t Cts_Rr_Ps_Vent_Cons_Bdry_Lo_Y = 0;

uint8_t Cts_Fr_dr_Vent_Side_siganl = 0;
uint8_t Cts_Fr_dr_Vent_Center_signal = 0;
uint8_t Cts_Fr_Ps_Vent_Side_signal = 0;
uint8_t Cts_Fr_Ps_Vent_Center_signal = 0;
#endif
///////////////////////////////////////////////////////////////////////////////
Atcu_32_data Can_data_32_Info;

Evnt_mode Hu_Fr_Dr_Vent_Mode = MODE_OFF;
Evnt_mode Hu_Fr_Ps_Vent_Mode = MODE_OFF;
Evnt_mode Hu_Fr_Dr_Vent_Mode_invalid = FACING_MODE;
Evnt_mode Hu_Fr_Ps_Vent_Mode_invalid = FACING_MODE;
uint8_t Hu_Fr_Dr_Vent_Mode_success = 0;
uint8_t Hu_Fr_Ps_Vent_Mode_success = 0;

Evnt_mode Hu_Fr_Dr_Vent_Mode_Remember = FACING_MODE;
Evnt_mode Hu_Fr_Ps_Vent_Mode_Remember = FACING_MODE;

uint8_t Hu_Fr_dr_Vent_Side_signal = 0;
uint8_t Hu_Fr_dr_Vent_Center_signal = 0;
uint8_t Hu_Fr_Ps_Vent_Side_signal = 0;
uint8_t Hu_Fr_Ps_Vent_Center_signal = 0;

uint8_t Hu_Fr_dr_Vent_Side_change = 0;
uint8_t Hu_Fr_dr_Vent_Center_change = 0;
uint8_t Hu_Fr_Ps_Vent_Side_change = 0;
uint8_t Hu_Fr_Ps_Vent_Center_change = 0;

uint8_t Hu_Frdr_Vent_Side_sig_backup = 0;
uint8_t Hu_Frdr_Vent_Center_sig_backup = 0;
uint8_t Hu_FrPs_Vent_Side_sig_backup = 0;
uint8_t Hu_FrPs_Vent_Center_sig_backup = 0;

uint8_t Hu_Fr_dr_Vent_Side_signal_invalid = 0;
uint8_t Hu_Fr_dr_Vent_Center_signal_invalid = 0;
uint8_t Hu_Fr_Ps_Vent_Side_signal_invalid = 0;
uint8_t Hu_Fr_Ps_Vent_Center_signal_invalid = 0;
uint8_t Hu_Fr_dr_Vent_Side_signal_success = 0;
uint8_t Hu_Fr_dr_Vent_Center_signal_success = 0;
uint8_t Hu_Fr_Ps_Vent_Side_signal_success = 0;
uint8_t Hu_Fr_Ps_Vent_Center_signal_success = 0;

///////////////////////////////////////////////////////////////////////////////
Atcu_33_data Can_data_33_Info;

Evnt_mode Cts_Fr_Dr_Vent_Mode = MODE_OFF;
Evnt_mode Cts_Fr_Ps_Vent_Mode = MODE_OFF;
Evnt_mode Cts_Fr_Dr_Vent_Mode_remember = DEFAULT_MODE;
Evnt_mode Cts_Fr_Ps_Vent_Mode_remember = DEFAULT_MODE;

///////////////////////////////////////////////////////////////////////////////
//CAN TX

Lvnt_1_data Can_Tx_Evnt_1;
Lvnt_2_data Can_Tx_Evnt_2;
Lvnt_3_data Can_Tx_Evnt_3;
Lvnt_4_data Can_Tx_Evnt_4;
Lvnt_5_data Can_Tx_Evnt_5;
Lvnt_6_data Can_Tx_Evnt_6;
Lvnt_7_data Can_Tx_Evnt_7;
Lvnt_8_data Can_Tx_Evnt_8;
Lvnt_9_data Can_Tx_Evnt_9;
///////////////////////////////////////////////////////////////////////////////
/******************************************************************************
 * Function prototypes
 ******************************************************************************/
void SendCANData(uint32_t mailbox, uint32_t messageId, const uint8_t *data, uint32_t len);
void flexcan0_Callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t buffIdx, flexcan_state_t *flexcanState); 
void flexcan0_ErrorCallback(uint8_t instance, flexcan_event_type_t eventType, flexcan_state_t *flexcanState);
status_t Firmware_Update_Check(void);
unsigned char *parseByte(const uint8_t *at_str);
static uint16_t Calculate_Chksum_2(const uint8_t *data, unsigned int length);
static uint8_t Hex_converter(char c);
static void hexStringToBytes(const char *hexString, uint8_t *byteArray, int length);
static void AsciiToHex(const char *ascii, char *hex);
static void UInt32ToHexString(char *output, uint32_t value);
static void ByteToHex(uint8_t byte, char *output);
static void ConvertRxDataToHexString(const uint8_t *rx_data_buffer, char *hex_string_out);
static void Check_rx_start(void);
static void Rx_id_22(void);
static void Rx_id_24(void);
static void Rx_id_28(void);
static void Rx_id_29(void);
static void Rx_id_119(void);
static void Rx_id_131(void);
static void Rx_id_161(void);
static void Rx_id_505(void);
static void Can_Full_close(void);
static void Can_Vent_mode(void);
static void Can_Full_close_mode(void);
static void Can_Full_close_value(void);
static void Can_Full_close_open(void);
void Can_Open_Facing_mode(void);
void Can_Open_Avoid_mode(void);
void Can_Open_Swing_mode(void);
static void Can_Open_Manual_mode(void);
static void Can_Vent_mode_set(void);
static void Can_Facing_mode_set(void);
static void Can_Avoid_mode_set(void);
static void Can_Cycle_mode_set(void);
static void Can_Manual_mode_set(void);
static void handleCanError(volatile CAN_Type *base, uint32_t error_flag);
static void Can_SubId_34(void);
static void Can_SubId_37(void);
static void Dr_Manual_position(void);
static void Ps_Manual_position(void);
/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/
/**
 * @brief  read_device_nv function
 * @param  Event: None
 * @retval Ack: Return whether the Event has been managed or not
 * @example  None
 * @note This function polls until conversion is complete.
 */
static int read_nv_func(nvmem_data_type *pcBuffer)
{
	return nvmem_read(pcBuffer);
}

/*
* @brief: Send data via CAN to the specified mailbox with the specified message id
* @param mailbox 	: Destination mailbox number
* @param messageId : Message ID
* @param data			: Pointer to the TX data
* @param len 			: Length of the TX data
* @return					: None
*/
void SendCANData(uint32_t mailbox, uint32_t messageId, const uint8_t *data, uint32_t len)
{
	/* Set information about the data to be sent
	*	- 1 byte in length
	*	- Standard message ID
	*	- Bit rate switch enabled to use a different bitrate for the data segment
	*	- Flexible data rate enabled
	*	- Use zeros for FD padding
	*/
	flexcan_data_info_t dataInfo =
	{
	.data_length = len,
	.msg_id_type = FLEXCAN_MSG_ID_STD,
	.enable_brs = false,
	.fd_enable  = false,
	.fd_padding = 0U
	};

	/* Configure TX message buffer with index TX_MSG_ID and TX_MAILBOX*/
	FLEXCAN_DRV_ConfigTxMb(INST_CANCOM1, (uint8_t)mailbox, &dataInfo, messageId);

	/* Execute send non-blocking */
	FLEXCAN_DRV_Send(INST_CANCOM1, (uint8_t)mailbox, &dataInfo, messageId, data);
	//FLEXCAN_DRV_SendBlocking(INST_CANCOM1, mailbox, &dataInfo, messageId, data, 100); 
	while(FLEXCAN_DRV_GetTransferStatus(INST_CANCOM1,TX_MAILBOX)==STATUS_BUSY)
	{
	}
}

/*
* @brief  Evt_queue_init function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Evt_queue_init(void)
{
	q_front = 0; //queue init
	q_rear = 0;

	memset(vent_evt, 0x00, sizeof(vent_evt));
}

/*
* @brief  Evt_queue_full function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
uint8_t Evt_queue_full(void)
{
	return (uint8_t)(((q_rear + 1) % EVT_QUEUE) == q_front);
}

/*
* @brief  Evt_queue_empty function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
uint8_t Evt_queue_empty(void)
{
	return (uint8_t)(q_front == q_rear);
}

/*
* @brief  Evt_queue_add function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
uint8_t Evt_queue_add(unsigned long long m_data)
{
	uint8_t ret = 0;

	if(Evt_queue_full() == 1) //queue check
	{
	 ret = 1;
	}

	q_rear = (q_rear + 1) % EVT_QUEUE; //queue add
	vent_evt[q_rear] = m_data;

	return ret;
}

/*
* @brief  Evt_queue_value function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
unsigned long long Evt_queue_value(void)
{
	unsigned long long ret;

	if(Evt_queue_empty() == 1) //empty check
	{
		ret = 0x0000000000000000ULL; 		
	}
	else
	{
		q_front = (q_front + 1u) % EVT_QUEUE;
		ret = vent_evt[q_front];
	}

	return ret;
}

/*
* @brief  CAN_Recovery_TX function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void CAN_Recovery_TX(void)
{
	/* Can recovery Init*/
	Can_Tx_Evnt_9.data.L_EVNT_AstCntrHorExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_AstCntrVertExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsHorExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsVertExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsHorExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsVertExtFlt = 0x01;		
}

/*
* @brief  Can_rx_2000_timer_recovery function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_rx_2000_timer_recovery(void)
{
	/*start Can rx 2000 timer check */
	if(((((((can_rx_2_expire == 0) && (can_rx_3_expire == 0)) && (can_rx_4_expire == 0)) && (can_rx_5_expire == 0)) && (can_rx_8_expire == 0)) && (can_rx_9_expire == 0)) && (can_rx_12_expire == 0))
	{
		rx_id_count = 0;
		Amo_timer_Stop(timer_14);	
		Amo_timer_Stop(timer_89);
		Amo_timer_Stop(timer_83);
		Amo_timer_Stop(timer_84);
		Amo_timer_Stop(timer_85);
		Amo_timer_Stop(timer_86);
		Amo_timer_Stop(timer_87);
		Evt_queue_add(DEVICE_CAN_RXDATA_RECOVERY_EVENT);
	}
	else
	{
		//CAN RX ERROR
		//Can_Tx_Evnt_9.data.L_EVNT_AstCntrHorExtFlt = 0x02;
		//Can_Tx_Evnt_9.data.L_EVNT_AstCntrVertExtFlt = 0x02;
		//Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsHorExtFlt = 0x02;
		//Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsVertExtFlt = 0x02;
		//Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsHorExtFlt = 0x02;
		//Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsVertExtFlt = 0x02;
		//Evnt_Network_Release = 0x00;
		//if(((((((can_rx_2_expire == 1)   && (can_rx_4_expire == 1)) && (can_rx_8_expire == 1)) && (can_rx_9_expire == 1)) && (can_rx_12_expire == 1)) && (can_rx_23_expire == 1)) && (can_rx_24_expire == 1))
		//{
		//Evnt_Network_Release = 0x00;
		//}
		int j = 0;
		rx_id_count = 0;
		for(int i=0; i< 20; i++)
		{
			if(Rx_info_data[i].start_info == 0x01) //recovery check
			{
				RX_Status[j] = Rx_info_data[i].rx_off;
				j++;
				rx_id_count++;
			}
		}

		/*start rx ID Check check */
		for(int k=0; k<j; k++)
		{
			if(RX_Status[k] != 0x01)
			{
				Evnt_Network_Release = 0x01;
				break;
			}
			else
			{
		Evnt_Network_Release = 0x00;
			}
		}
	}
}

/*
* @brief  Can_rx_5000_timer_recovery function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_rx_5000_timer_recovery(void)
{
	/*start rx Can_rx_5000_timer_recovery check */
	if((((((can_rx_23_expire == 0) && (can_rx_24_expire == 0)) && (can_rx_25_expire == 0)) && (can_rx_29_expire == 0)) && (can_rx_30_expire == 0)) && (can_rx_31_expire == 0))
	{
		rx_id_count = 0;
		Amo_timer_Stop(timer_15);	
		Amo_timer_Stop(timer_88);	
		Evt_queue_add(DEVICE_CAN_RXDATA_RECOVERY_EVENT);
	}
	else
	{
		//CAN RX ERROR
		//Can_Tx_Evnt_9.data.L_EVNT_AstCntrHorExtFlt = 0x02;
		//Can_Tx_Evnt_9.data.L_EVNT_AstCntrVertExtFlt = 0x02;
		//Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsHorExtFlt = 0x02;
		//Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsVertExtFlt = 0x02;
		//Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsHorExtFlt = 0x02;
		//Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsVertExtFlt = 0x02;		
		//Evnt_Network_Release = 0x00;
		//if(((((((can_rx_2_expire == 1)   && (can_rx_4_expire == 1)) && (can_rx_8_expire == 1)) && (can_rx_9_expire == 1)) && (can_rx_12_expire == 1)) && (can_rx_23_expire == 1)) && (can_rx_24_expire == 1))
		//{
		//Evnt_Network_Release = 0x00;
		//}		
		int j = 0;
		rx_id_count = 0;
		for(int i=0; i< 20; i++)
		{
			if(Rx_info_data[i].start_info == 0x01) //recovery check
			{
				RX_Status[j] = Rx_info_data[i].rx_off;
				j++;
				rx_id_count++;
			}
		}

		/*start rx id_5000_timer_expired check */
		for(int k=0; k<j; k++)
		{
			if(RX_Status[k] != 0x01)
			{
				Evnt_Network_Release = 0x01;
				break;
			}
			else
			{
			Evnt_Network_Release = 0x00;
			}
		}		
	}
}

/*
* @brief  Can_rx_2_timer function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_rx_2_timer(void)
{
	/* Actuator stop */
	can_rx_2_expire = 1;
	Rx_info_data[0].rx_off = 0x01;
	Amo_timer_Stop(timer_14);
	Amo_timer_Start(timer_14, 2000, true, Can_rx_2000_timer_recovery);	
	Evt_queue_add(DEVICE_ACTUATOR_SECURE_EVENT);
}

/*
* @brief  Can_rx_3_timer function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_rx_3_timer(void)
{
	/* Actuator stop */
	can_rx_3_expire = 1;
	Rx_info_data[1].rx_off = 0x01;
	Amo_timer_Stop(timer_89);
	Amo_timer_Start(timer_89, 2000, true, Can_rx_2000_timer_recovery);	
	Evt_queue_add(DEVICE_ACTUATOR_SECURE_EVENT);
}

/*
* @brief  Can_rx_4_timer function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_rx_4_timer(void)
{
	/* Actuator stop */
	can_rx_4_expire = 1;
	Rx_info_data[2].rx_off = 0x01;
	Amo_timer_Stop(timer_83);
	Amo_timer_Start(timer_83, 2000, true, Can_rx_2000_timer_recovery);	
	Evt_queue_add(DEVICE_ACTUATOR_SECURE_EVENT);
}

#ifdef AMO_GN7_PE_SETTING_NONE
/*
* @brief  Can_rx_5_timer function
* @param  Event: None
* @return None
*/
static void Can_rx_5_timer(void)
{
	/* Actuator stop */
	can_rx_5_expire = 1;
	Rx_info_data[3].rx_off = 0x01;
	Amo_timer_Stop(timer_84);
	Amo_timer_Start(timer_84, 2000, true, Can_rx_2000_timer_recovery);	
	Evt_queue_add(DEVICE_ACTUATOR_SECURE_EVENT);
}
#endif

/*
* @brief  Can_rx_8_timer function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_rx_8_timer(void)
{
	/* Actuator stop */
	can_rx_8_expire = 1;
	Rx_info_data[4].rx_off = 0x01;
	Amo_timer_Stop(timer_85);
	Amo_timer_Start(timer_85, 2000, true, Can_rx_2000_timer_recovery);		
	Evt_queue_add(DEVICE_ACTUATOR_SECURE_EVENT);
}

/*
* @brief  Can_rx_9_timer function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_rx_9_timer(void)
{
	/* Actuator stop */
	can_rx_9_expire = 1;
	Rx_info_data[5].rx_off = 0x01;
	Amo_timer_Stop(timer_86);
	Amo_timer_Start(timer_86, 2000, true, Can_rx_2000_timer_recovery);		
	Evt_queue_add(DEVICE_ACTUATOR_SECURE_EVENT);
}

/*
* @brief  Can_rx_12_timer function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_rx_12_timer(void)
{
	/* Actuator stop */
	can_rx_12_expire = 1;
	Rx_info_data[6].rx_off = 0x01;
	Amo_timer_Stop(timer_87);
	Amo_timer_Start(timer_87, 2000, true, Can_rx_2000_timer_recovery);		
	Evt_queue_add(DEVICE_ACTUATOR_SECURE_EVENT);
}

/*
* @brief  Can_rx_23_timer function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_rx_23_timer(void)
{
	/* Actuator stop */
	can_rx_23_expire = 1;
	Rx_info_data[7].rx_off = 0x01;
	Amo_timer_Stop(timer_88);
	Amo_timer_Start(timer_88, 5000, true, Can_rx_5000_timer_recovery);		
	Evt_queue_add(DEVICE_ACTUATOR_SECURE_EVENT);
}

/*
* @brief  Can_rx_24_timer function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_rx_24_timer(void)
{
	/* Actuator stop */
	can_rx_24_expire = 1;
	Rx_info_data[8].rx_off = 0x01;
	Amo_timer_Stop(timer_15);
	Amo_timer_Start(timer_15, 5000, true, Can_rx_5000_timer_recovery);	
	Evt_queue_add(DEVICE_ACTUATOR_SECURE_EVENT);
}

#if 0
static void Can_rx_25_timer(void)
{
	//Actuator stop !!!
	can_rx_25_expire = 1;
	Amo_timer_Stop(timer_15);
	Amo_timer_Start(timer_15, 5000, true, Can_rx_5000_timer_recovery);
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

static void Can_rx_29_timer(void)
{
	//Actuator stop !!!
	can_rx_29_expire = 1;
	Amo_timer_Stop(timer_15);
	Amo_timer_Start(timer_15, 5000, true, Can_rx_5000_timer_recovery);
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

static void Can_rx_30_timer(void)
{
	//Actuator stop !!!
	can_rx_30_expire = 1;
	Amo_timer_Stop(timer_15);
	Amo_timer_Start(timer_15, 5000, true, Can_rx_5000_timer_recovery);
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

static void Can_rx_31_timer(void)
{
	//Actuator stop !!!
	can_rx_31_expire = 1;
	Amo_timer_Stop(timer_15);
	Amo_timer_Start(timer_15, 5000, true, Can_rx_5000_timer_recovery);	
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}
#endif

/*
* @brief  Init_CanTx_Parameter function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Init_CanTx_Parameter(void)
{
	/*start parameter initial check */
	memset(&Can_Tx_Evnt_1, 0x00, sizeof(Can_Tx_Evnt_1)); //param init
	memset(&Can_Tx_Evnt_2, 0x00, sizeof(Can_Tx_Evnt_2));
	memset(&Can_Tx_Evnt_3, 0x00, sizeof(Can_Tx_Evnt_3));
	memset(&Can_Tx_Evnt_4, 0x00, sizeof(Can_Tx_Evnt_4));
	memset(&Can_Tx_Evnt_5, 0x00, sizeof(Can_Tx_Evnt_5));
	memset(&Can_Tx_Evnt_6, 0x00, sizeof(Can_Tx_Evnt_6));
	memset(&Can_Tx_Evnt_7, 0x00, sizeof(Can_Tx_Evnt_7));
	memset(&Can_Tx_Evnt_8, 0x00, sizeof(Can_Tx_Evnt_8));
	memset(&Can_Tx_Evnt_9, 0x00, sizeof(Can_Tx_Evnt_9));

////////////////////////////////////////////////////////////////////
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = 0x00;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = 0x00;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = 0x00;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = 0x00;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = 0x00;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = 0x00;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = 0x00;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = 0x00;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_3.data.L_EVNT_HU_RrDrEvntConsPtDisp_X = 0x00;
	Can_Tx_Evnt_3.data.L_EVNT_HU_RrDrEvntConsPtDisp_Y = 0x00;
	Can_Tx_Evnt_3.data.L_EVNT_HU_RrPsEvntConsPtDisp_X = 0x00;
	Can_Tx_Evnt_3.data.L_EVNT_HU_RrPsEvntConsPtDisp_Y = 0x00;

	Can_Tx_Evnt_3.data.L_EVNT_FrDrEvntSideFlt = 0x01;			//No error init 0x01 : No error 0x02: Error
	Can_Tx_Evnt_3.data.L_EVNT_FrDrEvntCtrFlt = 0x01;			//Recovery Mode/Actuator(Lin Bus Off/Interior Error Flag/External Error Flag) Error Noti
	Can_Tx_Evnt_3.data.L_EVNT_FrPsEvntSideFlt = 0x01;
	Can_Tx_Evnt_3.data.L_EVNT_FrPsEvntCtrFlt = 0x01;
	Can_Tx_Evnt_3.data.L_EVNT_RrDrEvntConsFlt = 0x01;
	Can_Tx_Evnt_3.data.L_EVNT_RrPsEvntConsFlt = 0x01;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00;	//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00;
	Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0x00;
	Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x00;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
	Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x01;
	Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x01;
	Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x01;
	Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_CTS = 0x01;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntSidePtDisp_X = 0x01;
	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntSidePtDisp_Y = 0x01;
	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntCtrPtDisp_X = 0x01;
	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntCtrPtDisp_Y = 0x01;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntSidePtDisp_X = 0x01;
	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntSidePtDisp_Y = 0x01;
	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntCtrPtDisp_X = 0x01;
	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntCtrPtDisp_Y = 0x01;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_6.data.L_EVNT_CTS_FrPsEvntSidePtDisp_X = 0x01;
	Can_Tx_Evnt_6.data.L_EVNT_CTS_FrPsEvntSidePtDisp_Y = 0x01;
	Can_Tx_Evnt_6.data.L_EVNT_CTS_FrPsEvntCtrPtDisp_X = 0x01;
	Can_Tx_Evnt_6.data.L_EVNT_CTS_FrPsEvntCtrPtDisp_Y = 0x01;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_7.data.L_EVNT_CTS_RrDrEvntConsPtDisp_X = 0x01;
	Can_Tx_Evnt_7.data.L_EVNT_CTS_RrDrEvntConsPtDisp_Y = 0x01;
	Can_Tx_Evnt_7.data.L_EVNT_CTS_RrPsEvntConsPtDisp_X = 0x01;
	Can_Tx_Evnt_7.data.L_EVNT_CTS_RrPsEvntConsPtDisp_Y = 0x01;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_8.data.L_EVNT_ATCUComFlt = 0x01;	//default : 0, No error : 0x01, Error : 0x02, Error Indicator : 0x03
	Can_Tx_Evnt_8.data.L_EVNT_EVNTFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvSdVertComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvSdHorComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvCntrVertComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvCntrHorComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_AstSdVertComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_AstSdHorComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_AstCntrVertComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_AstCntrHorComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_RrDrvCnsVertComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_RrDrvCnsHorComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_RrAstCnsVertComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_RrAstCnsHorComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvSdVertIntFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvSdHorIntFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvCntrVertIntFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvCntrHorIntFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_AstSdVertIntFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_AstSdHorIntFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_AstCntrVertIntFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_AstCntrHorIntFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_RrDrCnsVertIntFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_RrDrCnsHorIntFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_RrAstCnsVertIntFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_RrAstCnsHorIntFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvSdVertExtFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvSdHorExtFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvCntrVertExtFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvCntrHorExtFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_AstSdVertExtFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_AstSdHorExtFlt = 0x01;	
	
////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_9.data.L_EVNT_AstCntrVertExtFlt = 0x01; //default : 0, No error : 0x01, Error : 0x02, Error Indicator : 0x03
	Can_Tx_Evnt_9.data.L_EVNT_AstCntrHorExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsVertExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsHorExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsVertExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsHorExtFlt = 0x01;
}

#if 0
void Can_Rx_TimeOut(void)
{
	printf("CAN Rx timeout \r\n");	
}
#endif

/*
* @brief  Evnt_Atcu_Diagmsg function
* @param  Event: None
* @return None
*/
void Evnt_Atcu_Diagmsg(void)
{
	/*start Diag check */
	Evt_queue_add(DEVICE_CAN_EVNT_ATCU_SEND_EVENT);
}

/*
* @brief  Evnt_Atcu_tx_data function
* @param  Event: None
* @return None
*/
void Evnt_Atcu_tx_data(void)
{
	/*start Tx data initial */
	Can_tx_id tx_id; //tx init
	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_1.reg_data[0], sizeof(Can_Tx_Evnt_1.reg_data));
	tx_id = L_EVNT_01;
	SendCANData(24, (uint32_t)tx_id, Tx_candata, sizeof(Tx_candata));
	
	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_2.reg_data[0], sizeof(Can_Tx_Evnt_2.reg_data));
	tx_id = L_EVNT_02;
	SendCANData(24, (uint32_t)tx_id, Tx_candata, sizeof(Tx_candata));
	
	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_3.reg_data[0], sizeof(Can_Tx_Evnt_3.reg_data));
	tx_id = L_EVNT_03;
	SendCANData(24, (uint32_t)tx_id, Tx_candata, sizeof(Tx_candata)); 

	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_4.reg_data[0], sizeof(Can_Tx_Evnt_4.reg_data));
	tx_id = L_EVNT_04;
	SendCANData(24, (uint32_t)tx_id, Tx_candata, sizeof(Tx_candata));

#ifdef AMO_GN7_PE_SETTING_NONE
	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_5.reg_data[0], sizeof(Can_Tx_Evnt_5.reg_data));
	tx_id = L_EVNT_05;
	SendCANData(24, (uint32_t)tx_id, Tx_candata, sizeof(Tx_candata));

	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_6.reg_data[0], sizeof(Can_Tx_Evnt_6.reg_data));
	tx_id = L_EVNT_06;
	SendCANData(24, (uint32_t)tx_id, Tx_candata, sizeof(Tx_candata));

	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_7.reg_data[0], sizeof(Can_Tx_Evnt_7.reg_data));
	tx_id = L_EVNT_07;
	SendCANData(24, (uint32_t)tx_id, Tx_candata, sizeof(Tx_candata));
#endif

	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_8.reg_data[0], sizeof(Can_Tx_Evnt_8.reg_data));
	tx_id = L_EVNT_08;
	SendCANData(24, (uint32_t)tx_id, Tx_candata, sizeof(Tx_candata));

	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_9.reg_data[0], sizeof(Can_Tx_Evnt_9.reg_data));
	tx_id = L_EVNT_09;
	SendCANData(24, (uint32_t)tx_id, Tx_candata, sizeof(Tx_candata));

}

#ifdef AMO_DTC_SETTING
/*
* @brief  SendDTCMessage function
* @param  Event: None
* @return None
*/
void SendDTCMessage(uint32_t CanId, uint8_t *data, uint16_t length, uint8_t id, uint8_t sub_func, uint8_t dtc_status)
{
	/* SendDTCMessage length check */
	if(length <= 3)
	{	
		memset(Tx_candata, 0xAA, sizeof(Tx_candata));
		/* tx return value setting */
		Tx_candata[0] = (0x0u << 4) | length;
		Tx_candata[1] = id+0x40;
		Tx_candata[2] = sub_func;
		Tx_candata[3] = dtc_status;
		memcpy(&Tx_candata[4], data, length);
		SendCANData(24, CanId, Tx_candata, sizeof(Tx_candata));
	}
	else
	{
		memset(Tx_candata, 0xAA, sizeof(Tx_candata));
		/* tx return value setting */
		//Tx_candata[0] = 0x10 | ((length >> 8) & 0x0F); (0x1 << 4) | ((length >> 8) & 0x0F);
		Tx_candata[0] = (0x1u << 4) | ((length >> 8) & 0x0Fu);
		Tx_candata[1] = length & 0xFF;
		Tx_candata[2] = id+0x40;
		Tx_candata[3] = sub_func;
		Tx_candata[4] = dtc_status;		
		memcpy(&Tx_candata[5], data, 3);
		bytesSent = 3;
		SendCANData(24, CanId, Tx_candata, sizeof(Tx_candata));	
	}
}

/*
* @brief  DTC_Save_Code function
* @param  Event: None
* @return None
*/
void DTC_Save_Code(DTC_ID dtcCode)
{
	nvmem_data_type nv;
	//nvmem_read(&nv);
	read_nv_func(&nv);

	/* DTC_Save_Code Initialize */
	for(int i=0; i< MAX_DTC_COUNT; i++)
	{
		
		if(nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcCode == 0x000000)
		{
			nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcCode = dtcCode;
			nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcStatus = 0x01;
			nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcOccurence = 1;
		}
		else if(nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcCode == dtcCode) 
		{
			nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcOccurence++;
		}
		else
		{
			break;
		}
	}
	nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
	nv_max_cnt = nv.nv_item.write_cnt;
	nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 637);
	nvmem_write(nv.nv_data,PAGE_SIZE);	
}

/*
* @brief  DTC_Erase_Code function
* @param  Event: None
* @return None
*/
void DTC_Erase_Code(DTC_ID dtcCode)
{
	nvmem_data_type nv;
	//nvmem_read(&nv);
	read_nv_func(&nv);

	/*start dtc code erase */
	for(int i=0; i< MAX_DTC_COUNT; i++)
	{
		if(nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcCode == dtcCode)
		{
			nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcCode = 0x000000;
			nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcStatus = 0x00;
			nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcOccurence = 0;
			break;
		}
		else
		{
			break;
		}
	}
	nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
	nv_max_cnt = nv.nv_item.write_cnt;
	nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 637);
	nvmem_write(nv.nv_data,PAGE_SIZE);		
}

/*
* @brief  DTC_Read_Code function
* @param  Event: None
* @return None
*/
void DTC_Read_Code(void)
{
	nvmem_data_type nv;
	//nvmem_read(&nv);
	read_nv_func(&nv);
	//uint8_t temp_data[10] = {0,};
	uint8_t j=0;
	memset(Dtc_data, 0x00, sizeof(Dtc_data));
	DTC_Format dtc_data;

	/*start dtc code read */
	for(int i=0; i< MAX_DTC_COUNT; i++)
	{
		if(nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcCode != 0x000000)
		{
			dtc_data.high_byte = (nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcCode >> 16) & 0xFF;
			dtc_data.middle_byte = (nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcCode >> 8) & 0xFF;
			dtc_data.low_byte = nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcCode & 0xFF;

			Dtc_data[j] = dtc_data.high_byte;
			Dtc_data[j+1] = dtc_data.middle_byte;
			Dtc_data[j+2] = dtc_data.low_byte;
			j=j+3;
		}
		else
		{
			break;
		}
	}
	nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
	nv_max_cnt = nv.nv_item.write_cnt;

	nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 637);
	nvmem_write(nv.nv_data,PAGE_SIZE);		
}
#endif

/*
 * @brief  Calculate_Chksum_2 function
 * @param  Event: None
 * @return None
 * @note This function polls until conversion is complete.
 */
static uint16_t Calculate_Chksum_2(const uint8_t *data, unsigned int length)
{
	unsigned int total = 0;
	uint16_t checksum = 0;

	/*start calculate check */
	for (unsigned int i = 0; i < length; i++) //crc check
	{
		total += data[i];
		
		#if 0
		if(checksum >= 0xFF)
		{
			checksum -= 0xFF; 
		}
		#endif
	}
	checksum = (uint16_t)(total & 0x0000FFFFu);
	return ~checksum;
}

/*
 * @brief  UInt32ToHexString function
 * @param  Event: None
 * @return None
 * @note This function polls until conversion is complete.
 */
static void UInt32ToHexString(char *output, uint32_t value)
{
	/* Hex -> string converter define */
	static const char hex_digits[] = "0123456789abcdef";
	int32_t i;
	for(i = 7; i >= 0; i--)
	{
		output[i] = hex_digits[value & 0x0FU]; //hex converter
		value >>= 4;
	}
	output[8] = '\0';
}

/*
* @brief  ByteToHex function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void ByteToHex(uint8_t byte, char *output)
{
	/* byte -> Hex converter define */
	static const char hex_table[] = "0123456789abcdef";
	output[0] = hex_table[(byte >> 4) & 0x0Fu];  //byte hex
	output[1] = hex_table[byte & 0x0Fu];         
	output[2] = '\0';                      
}


/*
* @brief  ConvertRxDataToHexString function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void ConvertRxDataToHexString(const uint8_t *rx_data_buffer, char *hex_string_out)
{
	/*start ConvertRxDataToHexString function */
	ByteToHex(rx_data_buffer[3], &hex_string_out[0]);
	ByteToHex(rx_data_buffer[4], &hex_string_out[2]);
	ByteToHex(rx_data_buffer[5], &hex_string_out[4]);
	ByteToHex(rx_data_buffer[6], &hex_string_out[6]); //start ConvertRxDataToHexString
	hex_string_out[8] = '\0';
}

/*
* @brief  Firmware_Update_Check function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
status_t Firmware_Update_Check(void)
{
	status_t ret = STATUS_SUCCESS;
	uint8_t rcv_header1[10];
	uint8_t rcv_header2[10];
	memset(rcv_header1, 0x00, sizeof(rcv_header1));
	memset(rcv_header2, 0x00, sizeof(rcv_header2));
	Fw_header *Image1_Header_Info = NULL;
	Fw_header *Image2_Header_Info = NULL;
	static uint32_t Image1_Header_address = 0x00006000U; //FLASH_HEADER_ADDRESS_1;
	static uint32_t Image2_Header_address = FLASH_HEADER_ADDRESS_2;
	Image1_Header_Info = (Fw_header *)Image1_Header_address;
	Image2_Header_Info = (Fw_header *)Image2_Header_address;
	//sprintf((char *)rcv_header1, "%8x", (unsigned int)Image1_Header_Info->signature);
	UInt32ToHexString((char *)rcv_header1, Image1_Header_Info->signature);
	//sprintf((char *)rcv_header2, "%8x", (unsigned int)Image2_Header_Info->signature);
	UInt32ToHexString((char *)rcv_header2, Image2_Header_Info->signature);
	#ifdef DEBUG_ON					
	printf("rcv_header1 = %s, rcv_header2 = %s \r\n", rcv_header1, rcv_header2);
	#endif

	/*start version compare check */
	if(strcmp((char *)SW_VERSION, (char *)verString) > 0) 
	{
		#ifdef DEBUG_ON
		printf("SW_VERSION = %s, veString = %s \r\n", (char *)SW_VERSION, (char *)verString);
		#endif
		ret =  STATUS_ERROR;
	}
	else
	{
#ifdef DEBUG_ON
		printf("SW_VERSION 1 = %s, veString = %s \r\n", (char *)SW_VERSION, (char *)verString);
#endif

		if((strcmp((char *)rcv_header1, "aa55aa55") == 0) || (strcmp((char *)rcv_header1, "55aa55aa") == 0))
		{
			P_type = 1;
			flash_new_address = FLASH_FW2_ADDRESS_2;
			flash_address = FLASH_FW2_ADDRESS_2;
			new_header_address = FLASH_HEADER_ADDRESS_2;
			old_header_address = FLASH_HEADER_ADDRESS_1;	
		}

		if((strcmp((char *)rcv_header2, "55aa55bb") == 0) || (strcmp((char *)rcv_header2, "bb55aa55") == 0))
		{
			P_type = 0;
			flash_new_address = FLASH_FW1_ADDRESS_1;
			flash_address = FLASH_FW1_ADDRESS_1;
			new_header_address = FLASH_HEADER_ADDRESS_1;
			old_header_address = FLASH_HEADER_ADDRESS_2;
		}
	}
	return ret;
}

#if 0
static void ConvertToHexString(char *dest, size_t dest_size, uint32_t value)
{
	static const char hex_chars[] = "0123456789ABCDEF";
	size_t i = dest_size - 1;

	dest[i--] = '\0';

	while(i > 0)
	{
		dest[i--] = hex_chars[value & 0xFu]; 
		value >>= 4;
	}
	dest[0] = '0';
}
#endif

/*
* @brief  Hex_converter function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static uint8_t Hex_converter(char c)
{
	uint8_t ret = 0;

	/*start hex converter check */
	if((c >= '0') && (c <= '9')) 
	{
		ret = (uint8_t)((uint8_t)c -(uint8_t)'0');
	}
	if((c >= 'A') && (c <= 'F'))
	{
		ret = (uint8_t)((uint8_t)c - (uint8_t)'A' + 10u);
	}
	if((c >= 'a') && (c <= 'f'))
	{
		ret = (uint8_t)((uint8_t)c - (uint8_t)'a' + 10u);
	}
	return ret; 
}

/*
* @brief  hexStringToBytes function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void hexStringToBytes(const char *hexString, uint8_t *byteArray, int length)
{
	/*start hexStringToBytes check */
	for (int i = 0; i < length; i++)
	{
		byteArray[i] = (Hex_converter(hexString[i * 2]) << 4) | Hex_converter(hexString[(i * 2) + 1]);
	}
}

/*
* @brief  AsciiToHex function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void AsciiToHex(const char *ascii, char *hex)
{
	static const char hex_table[] = "0123456789ABCDEF";

	//start ascii parsing	
	while (*ascii != '\0')
	{
		uint8_t value = (uint8_t)*ascii;
		*hex = hex_table[(value >> 4) & 0x0Fu];
		hex++;
		*hex = hex_table[value & 0x0Fu];
		hex++;
		ascii++;
	}
	*hex = '\0';
}

/*
* @brief  parseByte function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
unsigned char restore1[20];
unsigned char *parseByte(const uint8_t *at_str)
{
	char tmp[3], *endptr;
	int i;
	int at_str_len = 0 ;//strlen((char *)at_str);
	unsigned char *ret_ptr = restore1;
	unsigned long  p_result;
	errno = 0;

	//start pasing data
	while(at_str[at_str_len] != 0)
	{
		at_str_len ++;
	}
	memset(tmp, 0x00, sizeof(tmp));
	memset(restore1,0x00,sizeof(restore1));

	if((at_str_len % 2) !=0)
	{
		ret_ptr = NULL;
	}
	for(i=0;i<(at_str_len/2);i++) //hex converter
	{
		memcpy(tmp,&at_str[i*2],2);
		//tmp[2]=0;
		//restore1[i] = (unsigned char)strtoul(tmp,NULL,16);
		p_result = strtoul(tmp, &endptr, 16);
		if(((p_result == ULONG_MAX) && (errno == ERANGE)) || (endptr == tmp))
		{
			restore1[i] = 0x00;
		}
		else
		{
			restore1[i] = (unsigned char)p_result;
		}
	}
	return ret_ptr;
}

/*
* @brief  DTC_Send_event function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void DTC_Send_event(void)
{
	uint32_t tx_id = 0;
	tx_id = 0x605;
	uint8_t HexOut[20];
	uint8_t Tx_Ota[8];
	unsigned char *restore = NULL;

	memset(Tx_Ota, 0xAA, sizeof(Tx_Ota));
	memset(HexOut, 0x00, sizeof(HexOut));

	//start dtc command
	//if((pci & 0xF0u) == 0x00u) //20250423_dtc_check
	//{
		switch(dtc_subFunc)
		{
			case 0x01: //SW VERSION
			{	
				AsciiToHex((char *)SW_VERSION, (char *)HexOut);
				restore = parseByte((uint8_t *)HexOut);

				if(dtc_data_length < 5)
				{
					/* Tx ota buffer setting */
					Tx_Ota[0] = (0x0u << 4) | 7u;
					Tx_Ota[1] = dtc_serviceId+0x40;
					Tx_Ota[2] = dtc_subFunc;
					Tx_Ota[3] = dtcstatus;
					if(restore != NULL)
					{
						Tx_Ota[4] = restore[0];
						Tx_Ota[5] = restore[1];
						Tx_Ota[6] = restore[2];
						Tx_Ota[7] = restore[3];
					}
				}
				else
				{

				}
				SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));									
				break;
			}
			case 0x02:
			{
				/*start sw reset check */
				Tx_Ota[0] = (0x0u << 4) | 3u;
				Tx_Ota[1] = dtc_serviceId+0x40;
				Tx_Ota[2] = dtc_subFunc;
				Tx_Ota[3] = dtcstatus;
				
				SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota)); 				
				SystemSoftwareReset();		
				break;
			}
			case 0x03:  //FIRMWARE CHECK
			{					
				uint8_t *data_check = NULL;
				uint8_t *size_check = NULL;
				uint8_t *header_checksum = NULL;
				uint16_t fw_sum = 0;
				uint16_t header_sum = 0;
				uint32_t fw_size = 0;	

				/* size_check setting */
				size_check = (uint8_t *)0x00006040;
				fw_size = ((((uint32_t)size_check[2] << 16) & 0xFF0000u) | (((uint32_t)size_check[1] << 8) & 0x00FF00u) | ((uint32_t)size_check[0] & 0x0000FFu));
				data_check = (uint8_t *)FLASH_FW1_ADDRESS_1;
				header_checksum = (uint8_t *)0x00006060;
				//header_sum = (header_checksum[0] << 8) | header_checksum[1];
				header_sum = (((uint16_t)header_checksum[1] << 8) | (uint16_t)header_checksum[0]);
				WDOG_DRV_Trigger(INST_WATCHDOG1);
				fw_sum = Calculate_Chksum_2(data_check, fw_size);

				if(dtcstatus == 0x01)
				{
					/* Tx ota buffer setting */
					Tx_Ota[0] = (0x0u << 4) | 5u;
					Tx_Ota[1] = dtc_serviceId+0x40;
					Tx_Ota[2] = dtc_subFunc;
					Tx_Ota[3] = dtcstatus;
					Tx_Ota[4] = header_checksum[1];
					Tx_Ota[5] = header_checksum[0];
					SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));				
				}
				else if(dtcstatus == 0x02)
				{
					#ifdef DEBUG_ON
					printf("fw size = %ld, chksum =  %4x header_checksum = %4x\r\n", fw_size, fw_sum, header_sum);	
					printf("%02x %02x %02x %02x\r\n", *((uint8_t *)data_check+0), *((uint8_t *)data_check+1), *((uint8_t *)data_check+2), *((uint8_t *)data_check+3));
					printf("AIR_VENT_DL_MODE = %d \r\n", AIR_VENT_DL_MODE);
					#endif
					
					if((fw_sum == header_sum))
					{
						/* Tx buffer ok value return */
						Tx_Ota[0] = (0x0u << 4) | 5u;
						Tx_Ota[1] = dtc_serviceId+0x40;
						Tx_Ota[2] = dtc_subFunc;
						Tx_Ota[3] = dtcstatus;
						Tx_Ota[4] = (uint8_t)(((fw_sum & 0xFF00u) >> 8u));  
						Tx_Ota[5] = (uint8_t)(fw_sum & 0xFFu);
						SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));
					}
					else
					{
						/* Tx buffer fail value return */
						Tx_Ota[0] = 0x7F;
						Tx_Ota[1] = 0x30;				
						Tx_Ota[2] = dtc_subFunc;	
						Tx_Ota[3] = dtcstatus;
						SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));							
					}
				}
				else
				{
				
				}
				break;
			}
			case 0x04:
			{					
				Amo_timer_Stop(timer_69);		
				Adc_Value();
				/* Tx buffer 0xID 04 value return */
				Tx_Ota[0] = (0x0u << 4) | 7u;
				Tx_Ota[1] = dtc_serviceId+0x40;
				Tx_Ota[2] = dtc_subFunc;
				Tx_Ota[3] = dtcstatus;
				Tx_Ota[4] = (uint8_t)(((Vatt_LimitValue & 0xFF00u) >> 8));
				Tx_Ota[5] = (uint8_t)(Vatt_LimitValue & 0x00FFu);
				Tx_Ota[6] = (uint8_t)(((Ign2_AdcMinValue & 0xFF00u) >> 8));
				Tx_Ota[7] = (uint8_t)(Ign2_AdcMinValue & 0x00FFu); 
				SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));
				Amo_timer_Start(timer_69, 1000, true, Adc_Value);
				break;
			}
			case 0x05:	//PART NUMBER
			{							
				break;
			}
			case 0x06:	//CAN DBC CHECK
			{
				/* Tx buffer 0xID 06 value return */
				Tx_Ota[0] = (0x0u << 4) | 6u;
				Tx_Ota[1] = dtc_serviceId+0x40;
				Tx_Ota[2] = dtc_subFunc;
				Tx_Ota[3] = dtcstatus;
				Tx_Ota[4] = (uint8_t)((CAN_DBC_VER >> 16) & 0xFFu);
				Tx_Ota[5] = (uint8_t)((CAN_DBC_VER >> 8) & 0xFFu);
				Tx_Ota[6] = (uint8_t)(CAN_DBC_VER & 0xFFu);
				SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));		
				break;
			}
			case 0x07:	//LIN DBC CHECK
			{	
				/* Tx buffer 0xID 07 value return */
				Tx_Ota[0] = (0x0u << 4) | 6u;
				Tx_Ota[1] = dtc_serviceId+0x40;
				Tx_Ota[2] = dtc_subFunc;
				Tx_Ota[3] = dtcstatus;
				Tx_Ota[4] = (uint8_t)((LIN_LDF_VER >> 16) & 0xFFu);
				Tx_Ota[5] = (uint8_t)((LIN_LDF_VER >> 8) & 0xFFu);
				Tx_Ota[6] = (uint8_t)(LIN_LDF_VER & 0xFFu);
				SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));				
				break;
			}
			case 0x08:
			{		
				AsciiToHex((char *)HW_VERSION, (char *)HexOut);
				restore = parseByte((uint8_t *)HexOut);

				if(dtc_data_length < 5)
				{
					Tx_Ota[0] = (0x0u << 4) | 7u;
					Tx_Ota[1] = dtc_serviceId+0x40;
					Tx_Ota[2] = dtc_subFunc;
					Tx_Ota[3] = dtcstatus;
					if(restore != NULL)
					{
						/* parsing data none */
						Tx_Ota[4] = restore[0];
						Tx_Ota[5] = restore[1];
						Tx_Ota[6] = restore[2];
						Tx_Ota[7] = restore[3];
					}
				}
				else
				{

				}	
				SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));									
				break;	
			}
			case 0x09:
			{							
				break;
			}		
			case 0x0A:
			{			
				nvmem_data_type nv;
				uint16_t batt_cal = 0;
				uint16_t ign_cal = 0;

				if(dtcstatus == 0x01)
				{
					//nvmem_read(&nv);
					read_nv_func(&nv);
					nv.nv_item.nv_battcal = 0;				
					nv.nv_item.nv_igncal = 0;	
					nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
#ifdef AMO_DTC_SETTING	
					nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 637);
#else
					nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 441);
#endif
					batt_data = 0;
					ign_data = 0;
					nvmem_write(nv.nv_data,PAGE_SIZE);
				}
				//nvmem_read(&nv);
				read_nv_func(&nv);
				nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
				Amo_timer_Stop(timer_69);		
				Adc_Value();
				if(Ign2_AdcMinValue <= BATT_CAL_DATA)
				{
					ign_cal = BATT_CAL_DATA - Ign2_AdcMinValue;
				}
				if(Vatt_LimitValue <= BATT_CAL_DATA)
				{
					batt_cal = BATT_CAL_DATA - Vatt_LimitValue;
				}
				if(Ign2_AdcMinValue > BATT_CAL_DATA)
				{
					ign_cal = Ign2_AdcMinValue - BATT_CAL_DATA;				
				}				
				if(Vatt_LimitValue > BATT_CAL_DATA)
				{
					batt_cal = Vatt_LimitValue - BATT_CAL_DATA;				
				}				
				nv.nv_item.nv_battcal = batt_cal; 			
				nv.nv_item.nv_igncal = ign_cal;
#ifdef AMO_DTC_SETTING	
				nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 637);
#else
				nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 441);
#endif
				nvmem_write(nv.nv_data,PAGE_SIZE);
				Tx_Ota[0] = (0x0u << 4) | 7u;
				Tx_Ota[1] = dtc_serviceId+0x40;
				Tx_Ota[2] = dtc_subFunc;
				Tx_Ota[3] = dtcstatus;
				Tx_Ota[4] = (uint8_t)((batt_cal & 0xFF00u) >> 8);
				Tx_Ota[5] = (uint8_t)(batt_cal & 0x00FFu);
				Tx_Ota[6] = (uint8_t)((ign_cal & 0xFF00u) >> 8);
				Tx_Ota[7] = (uint8_t)(ign_cal & 0x00FFu);
				
				SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));						
				break;
			}	
			case 0x10:
			{			
				//SW_Format version_data;				
				AsciiToHex((char *)SW_VERSION, (char *)HexOut);
				restore = parseByte((uint8_t *)HexOut);

				if(dtc_data_length < 5)
				{
					Tx_Ota[0] = (0x0u << 4) | 7u;
					Tx_Ota[1] = dtc_serviceId+0x40;
					Tx_Ota[2] = dtc_subFunc;
					Tx_Ota[3] = dtcstatus;
					if(restore != NULL)
					{
						Tx_Ota[4] = restore[0];
						Tx_Ota[5] = restore[1];
						Tx_Ota[6] = restore[2];
						Tx_Ota[7] = restore[3];
					}
				}
				else
				{

				}
				SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));		
				break;
			}
			case 0x11:
			{
				if(dtc_data_length < 5)
				{
					Tx_Ota[0] = (0x0u << 4) | 3u;
					Tx_Ota[1] = dtc_serviceId+0x40;
					Tx_Ota[2] = dtc_subFunc;
					Tx_Ota[3] = dtcstatus;

					SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));
					lin_FrDrCycleMode_TimerStop();
					lin_CycleMode_CanDisp_TimerStop();
					lin_FrPsCycleMode_TimerStop();
					Lin_GoTo_Sleep();
					Amo_Sleep();
				}
				else
				{

				}			
				break;
			}			
			default:
			/* default code */
			break;
		}
	//}

	if(((Rx_candata[0] == 0x79u) && (Rx_candata[1] == 0x32u)) && (Rx_candata[2] == 0x84u))
	{
		Tx_Ota[0] = 0x79;
		Tx_Ota[1] = 0x32;
		Tx_Ota[2] = 0x84;
		SendCANData(24, 0x605, Tx_Ota, sizeof(Tx_Ota)); 
	}
}

/*
* @brief  Can_SubId_34 function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_SubId_34(void)
{
	uint32_t tx_id = 0;
	tx_id = 0x605;
	status_t ret = STATUS_SUCCESS;
	uint8_t HexOut[20];
	uint8_t Tx_Ota[8];
	unsigned char *restore = NULL;

	memset(Tx_Ota, 0xAA, sizeof(Tx_Ota));
	memset(HexOut, 0x00, sizeof(HexOut));

	/*start Can_SubId_34 check */
	switch(dtc_subFunc) //start id 34
	{
		case 0x01: /* sw version */
		{
			AsciiToHex((char *)SW_VERSION, (char *)HexOut);
			restore = parseByte((uint8_t *)HexOut);

			if(dtc_data_length < 5)
			{
				Tx_Ota[0] = (0x0u << 4) | 6u;
				Tx_Ota[1] = dtc_serviceId+0x40;
				Tx_Ota[2] = dtc_subFunc;
				if(restore != NULL)
				{
					Tx_Ota[3] = restore[0];
					Tx_Ota[4] = restore[1];
					Tx_Ota[5] = restore[2];
					Tx_Ota[6] = restore[3];
				}
			}
			else
			{

			}
			
			SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota)); 								
		break;	
		}
		case 0x02: /* sw reset */
		{
			Tx_Ota[0] = (0x0u << 4) | 2u;
			Tx_Ota[1] = dtc_serviceId+0x40;
			Tx_Ota[2] = dtc_subFunc;
			
			SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota)); 				
			SystemSoftwareReset();
			break;
		}
		case 0x03:	
		{
			uint8_t temp_data[8];
			
			memset(HexString, 0x00, sizeof(HexString));
			memset(verString, 0x00, sizeof(verString));
			memset(temp_data, 0x00, sizeof(temp_data));
			memcpy(&temp_data[0], &Rx_candata[3], 4);
			
			//sprintf((char *)HexString, "%02x%02x%02x%02x", Rx_candata[3], Rx_candata[4], Rx_candata[5], Rx_candata[6]);
			ConvertRxDataToHexString(Rx_candata, (char *)HexString);
			hexStringToBytes((char *)HexString, verString, 4);
		#ifdef DEBUG_ON
			printf("HexString = %s verstring = %s \r\n", HexString, verString);
		#endif
			ret = Firmware_Update_Check();

			if(ret == STATUS_SUCCESS)
			{
#ifdef DEBUG_ON
				printf("flash_new_address = %08x \r\n", (unsigned int)flash_new_address);
#endif	
				INT_SYS_DisableIRQGlobal();
				FLASH_DRV_EraseSector(&flashSSDConfig, flash_new_address, (FLASH_SECTOR_SIZE*56));
				//DEV_ASSERT(STATUS_SUCCESS == ret);
				INT_SYS_EnableIRQGlobal();
				//AsciiToHex((char *)verString, (char *)HexOut);
				//restore = parseByte((uint8_t *)HexOut);
				Tx_Ota[0] = (0x0u << 4) | 6u;
				Tx_Ota[1] = dtc_serviceId+0x40;
				Tx_Ota[2] = dtc_subFunc;
				Tx_Ota[3] = temp_data[0];
				Tx_Ota[4] = temp_data[1];
				Tx_Ota[5] = temp_data[2];
				Tx_Ota[6] = temp_data[3];
				Tx_Ota[7] = P_type;
				SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota)); 		
			}
			else
			{
				AsciiToHex((char *)SW_VERSION, (char *)HexOut);
				restore = parseByte((uint8_t *)HexOut);
				Tx_Ota[0] = 0x7F;
				Tx_Ota[1] = dtc_serviceId;
				if(restore != NULL)
				{
					Tx_Ota[2] = restore[0];
					Tx_Ota[3] = restore[1];
					Tx_Ota[4] = restore[2];
					Tx_Ota[5] = restore[3];
				}
				SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota)); 			
			}
			break;
		}
		case 0x04: 
		{
			Tx_Ota[0] = (0x0u << 4) | 2u;
			Tx_Ota[1] = 0x7F;
			Tx_Ota[2] = dtc_subFunc;

			SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota)); 				
			break;
		} 								
		default:
			Tx_Ota[0] = (0x0u << 4) | 2u;
			Tx_Ota[1] = 0x7F;
			Tx_Ota[2] = dtc_subFunc;

			SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota)); 				
		break;
	}
}

/*
* @brief  Can_SubId_37 function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_SubId_37(void)
{
	uint32_t tx_id = 0;
	tx_id = 0x605;
	volatile status_t ret = STATUS_SUCCESS;
	uint8_t HexOut[20];
	uint8_t Tx_Ota[8];
	unsigned char *restore = NULL;

	/*start Can_SubId_37 check */
	memset(Tx_Ota, 0xAA, sizeof(Tx_Ota)); //start id 37
	memset(HexOut, 0x00, sizeof(HexOut));

	switch(dtc_subFunc)
	{
		case 0x01: //CRC CHECK & FW Write
		{
		#ifdef DEBUG_ON
			//printf("%02x %02x %02x %02x %02x %02x %02x length = %d \r\n", recvBuff2.data[1], recvBuff2.data[2], recvBuff2.data[3], recvBuff2.data[4], recvBuff2.data[5], recvBuff2.data[6], recvBuff2.data[7], recvBuff2.dataLen);
		#endif
			uint16_t crc_received = (((uint16_t)Rx_candata[3] << 8) | ((uint16_t)Rx_candata[4]));
			chksum_data = Calculate_Chksum_2(Write_Buffer, sn_index);
	
			if(sn_index <= 4)
			{
				sn_index = 4;
			}
			else if(sn_index <= 16)
			{
				sn_index = 16;
			}
			else if(sn_index <= 64)
			{
				sn_index = 64;
			}
			else if(sn_index <= 256)
			{
				sn_index = 256;
			}
			else if(sn_index <= 1024)
			{
				sn_index = 1024;
			}
			else if(sn_index <= 4096)
			{
				sn_index = 4096;
			} 				
			else
			{
				sn_index = 4096;
			}
		#ifdef DEBUG_ON
			printf("crc_received = %4x chksum_data = %4x \r\n", crc_received, chksum_data);
		#endif
			if(crc_received != chksum_data)
			{
				Tx_Ota[0] = 0x7F;
				Tx_Ota[1] = 0x37; 			
				Tx_Ota[2] = 0x01; 
	
				SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota)); 					
			}
			else
			{
				INT_SYS_DisableIRQGlobal(); 					
				FLASH_DRV_EraseSector(&flashSSDConfig, flash_new_address, FLASH_SECTOR_SIZE);
				ret = FLASH_DRV_Program(&flashSSDConfig, flash_new_address, sn_index, (uint8_t *)Write_Buffer);
				DEV_ASSERT(STATUS_SUCCESS == ret);
				INT_SYS_EnableIRQGlobal();
				
				if(ret != STATUS_SUCCESS)
				{
					Tx_Ota[0] = 0x7F;
					Tx_Ota[1] = 0x38;
					Tx_Ota[2] = 0x01;
					SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));
					break;
				}
				flash_new_address += sn_index;
				Tx_Ota[0] = (0x0u << 4) | 6u;
				Tx_Ota[1] = dtc_serviceId+0x40;
				Tx_Ota[2] = dtc_subFunc;
				Tx_Ota[3] = Rx_candata[3];
				Tx_Ota[4] = Rx_candata[4];
				SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota)); 	
			} 			
			break;
		}
		case 0x02: //FW UPDATE DONE
		{
			AsciiToHex((char *)verString, (char *)HexOut);
			restore = parseByte((uint8_t *)HexOut);
	
			Fw_header Fw_header_test = {.signature = 0x00000000, .app_key = APP_KEY, .app_size = 0x00000000, .crc = 0x00000000};
			memset(Fw_header_test.version, 0x00, sizeof(Fw_header_test.version));
	
			if(flash_address == FLASH_FW1_ADDRESS_1)
			{
				//memcpy(Fw_header_test.version, restore, 4);
				Fw_header_test.signature = HEADER_SIGNATURE_1;
				if(restore != NULL)
				{
					Fw_header_test.version[0] = restore[0];
					Fw_header_test.version[1] = restore[1];
					Fw_header_test.version[2] = restore[2];
					Fw_header_test.version[3] = restore[3];
				}
			#ifdef DEBUG_ON
				printf("flash_new_address = %08x old_header_address = %08x\r\n", (unsigned int)flash_new_address, (unsigned int)old_header_address);
			#endif
				INT_SYS_DisableIRQGlobal();
				FLASH_DRV_EraseSector(&flashSSDConfig, new_header_address, FLASH_SECTOR_SIZE);
				DEV_ASSERT(STATUS_SUCCESS == ret);
				FLASH_DRV_Program(&flashSSDConfig, new_header_address, FLASH_SECTOR_SIZE, (uint8_t *)&Fw_header_test);
				DEV_ASSERT(STATUS_SUCCESS == ret);
				INT_SYS_EnableIRQGlobal();		
	
				memset(Fw_header_test.version, 0x00, 4);
				Fw_header_test.signature = HEADER_SIGNATURE_NULL;
				INT_SYS_DisableIRQGlobal();
				FLASH_DRV_EraseSector(&flashSSDConfig, old_header_address, FLASH_SECTOR_SIZE);
				DEV_ASSERT(STATUS_SUCCESS == ret);
				FLASH_DRV_Program(&flashSSDConfig, old_header_address, FLASH_SECTOR_SIZE, (uint8_t *)&Fw_header_test);
				DEV_ASSERT(STATUS_SUCCESS == ret);
				INT_SYS_EnableIRQGlobal();		
				
				if(ret != STATUS_SUCCESS)
				{
					Tx_Ota[0] = 0x7F;
					Tx_Ota[1] = 0x37;
					Tx_Ota[2] = 0x02;
					SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));
					break;
				}
				else
				{
					Tx_Ota[0] = (0x0u << 4) | 2u;
					Tx_Ota[1] = dtc_serviceId+0x40;
					Tx_Ota[2] = dtc_subFunc;
					SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota)); 							
				}
				SystemSoftwareReset();
			}
	
			if(flash_address == FLASH_FW2_ADDRESS_2)
			{
				//memcpy(Fw_header_test.version, restore, 4);
				Fw_header_test.signature = HEADER_SIGNATURE_2;
				if(restore != NULL)
				{
					Fw_header_test.version[0] = restore[0];
					Fw_header_test.version[1] = restore[1];
					Fw_header_test.version[2] = restore[2];
					Fw_header_test.version[3] = restore[3];
				}
			#ifdef DEBUG_ON
				printf("flash_new_address = %08x old_header_address = %08x\r\n", (unsigned int)flash_new_address, (unsigned int)old_header_address);
			#endif
				INT_SYS_DisableIRQGlobal();
				FLASH_DRV_EraseSector(&flashSSDConfig, new_header_address, FLASH_SECTOR_SIZE);
				//DEV_ASSERT(STATUS_SUCCESS == ret);
				FLASH_DRV_Program(&flashSSDConfig, new_header_address, FLASH_SECTOR_SIZE, (uint8_t *)&Fw_header_test);
				//DEV_ASSERT(STATUS_SUCCESS == ret);
				INT_SYS_EnableIRQGlobal();		
	
				memset(Fw_header_test.version, 0x00, 4);
				Fw_header_test.signature = HEADER_SIGNATURE_NULL;
				
				INT_SYS_DisableIRQGlobal();
				FLASH_DRV_EraseSector(&flashSSDConfig, old_header_address, FLASH_SECTOR_SIZE);
				//DEV_ASSERT(STATUS_SUCCESS == ret);
				FLASH_DRV_Program(&flashSSDConfig, old_header_address, FLASH_SECTOR_SIZE, (uint8_t *)&Fw_header_test);
				//DEV_ASSERT(STATUS_SUCCESS == ret);
				INT_SYS_EnableIRQGlobal();								
				if(ret != STATUS_SUCCESS)
				{
					Tx_Ota[0] = 0x7F;
					Tx_Ota[1] = 0x37;
					Tx_Ota[2] = 0x02;
					SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));
					break;
				}
				else
				{
					Tx_Ota[0] = (0x0u << 4) | 2u;
					Tx_Ota[1] = dtc_serviceId+0x40;
					Tx_Ota[2] = dtc_subFunc;
					SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota)); 							
				}
			SystemSoftwareReset();
			}
			break;
		}
		case 0x03:	
		{
			break;
		}
		case 0x04: 
		{
			break;
		} 								
		default:
		/* default code */
		break;
	}
}

/*
* @brief  DTC_OTA_event function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void DTC_OTA_event(void)
{
	uint32_t tx_id = 0;
	tx_id = 0x605;
	uint8_t Tx_Ota[8];

	memset(Tx_Ota, 0xAA, sizeof(Tx_Ota));

	/*start OTA Param data check */
	if((pci & 0xF0u) == 0x00u) //start ota start
	{
		/* Single frame */
		if((pci & 0x0Fu) != 0x00u)
		{
			dtc_serviceId = Rx_candata[1];
			dtc_subFunc = Rx_candata[2];		
			dtc_data_length = pci & 0x0Fu;
		}
		else
		{
			dtc_serviceId = Rx_candata[2];
			dtc_subFunc = Rx_candata[3];		
			dtc_data_length = Rx_candata[1];							
		}

		if(dtc_serviceId == 0x34)
		{
			Can_SubId_34();
		}
		
		if(dtc_serviceId == 0x37)
		{
			Can_SubId_37();
		}		
	}
	else if(((pci & 0xF0u) >> 4) == 0x03u)
	{

	}
	#if 0
	else if(((pci & 0xF0) >> 4) == 0x02)
	{
		sn_number = (pci & 0x0F);
		memset(Tx_Ota, 0xAA, sizeof(Tx_Ota));
		
		if(sn_number != expected_sn_number)
		{
			//Tx_candata[0] = (0x0 << 4) | (dtc_data_length+2);
			Tx_Ota[0] = 0x7F;
			Tx_Ota[1] = 0x36;
			Tx_Ota[2] = sn_number;				
			Tx_Ota[3] = expected_sn_number;	//0x73;	

			SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));
		}
		else
		{
			#ifdef DEBUG_ON
			//printf("sn_number = %d expected_sn_number = %d \r\n", sn_number, expected_sn_number);
			#endif
			memcpy(&Write_Buffer[sn_index], &recvBuff2.data[1], (recvBuff2.dataLen-1));
			sn_index += (recvBuff2.dataLen -1);

			expected_sn_number = (expected_sn_number + 1) & MAX_SN_NUMBER;

			#ifdef DEBUG_ON
			//printf("%02x %02x %02x %02x %02x %02x %02x length = %d \r\n", recvBuff2.data[1], recvBuff2.data[2], recvBuff2.data[3], recvBuff2.data[4], recvBuff2.data[5], recvBuff2.data[6], recvBuff2.data[7], recvBuff2.dataLen);
			#endif
			Tx_Ota[0] = 0x02;
			Tx_Ota[1] = 0x76;
			Tx_Ota[2] = sn_number;				
			
			//memset(Write_Buffer, 0x00, sizeof(Write_Buffer));
			SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));		

			if(sn_index >= dtc_Total_length)
			{
				
			}
		}
	}	
	#endif
	else if(((pci & 0xF0u) >> 4) == 0x01u) //multi frame 0x36
	{
		dtc_serviceId = Rx_candata[2];
		dtc_subFunc = Rx_candata[3];		
		dtc_Total_length = (((((uint16_t)pci & 0x0Fu) << 8) & 0xFF00u) | ((uint16_t)Rx_candata[1] & 0x00FFu));
		memset(Write_Buffer, 0x00, sizeof(Write_Buffer));
		expected_sn_number = 0;	
		sn_index = 0;

		if(dtc_Total_length > 0)
		{
			Tx_Ota[0] = (uint8_t)((0x1u << 4) | ((dtc_Total_length >> 8) & 0x000Fu));
			Tx_Ota[1] = (uint8_t)((dtc_Total_length & 0x00FFu));				
			Tx_Ota[2] = (uint8_t)(dtc_serviceId+0x40);
			//Tx_Ota[3] = dtcstatus; 
			SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));	
		}
		else
		{
			Tx_Ota[0] = 0x7F;				
			Tx_Ota[1] = dtc_serviceId;
			//Tx_Ota[2] = dtcstatus; 
			SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));			
		}
	}					
	else
	{
		
	}	
}

/*
* @brief  Check_rx_start function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Check_rx_start(void)
{
	if(Can_rx_success == 0) //start rx check
	{
		Can_rx_success = 1;
	
		for(int i=0; i<20; i++)
		{
			Rx_info_data[i].start_info = 0x00;
			Rx_info_data[i].rx_off = 0x00;
		}
	
		memset(RX_Status, 0x00, sizeof(RX_Status));
		Evt_queue_add(DEVICE_ATCU_TX_EVENT);
		Amo_timer_Start(timer_7, 1000, true, Evnt_Atcu_Diagmsg);
	
		linbusOff_FltCheck_Start_Flag = TRUE;
	}
}

/*
* @brief  Rx_id_22 function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Rx_id_22(void)
{
	memset(Can_data_2_Info.reg_data, 0x00, sizeof(Can_data_2_Info.reg_data));
	memcpy(&Can_data_2_Info.reg_data[0], &recvBuff2.data[0], 8);
	
	can_rx_2_expire = 0;
	ATCU_Fr_Dr_Mode_display = (Drivermode)Can_data_2_Info.data.L_ATCU_FrDrModDis;
	ATCU_Fr_Ps_Mode_display = (Drivermode)Can_data_2_Info.data.L_ATCU_FrPassModDis;
	
#ifdef AMO_GN7_PE_SETTING_NONE
	if((Evnt_IGN2_Onoff == 1) || (Evnt_IGN3_Onoff == 1))	//IGN2 or IGN3 ON
#else
	if(Evnt_IGN2_Onoff == 1)	//IGN2
#endif
	{
	
		if((ATCU_Fr_Dr_Mode_display == VENT) || (ATCU_Fr_Dr_Mode_display == B_L) || (ATCU_Fr_Dr_Mode_display == MODE7_DEF_VENT) || (ATCU_Fr_Dr_Mode_display == MODE7_DEF_VENT_FLOOR) || (ATCU_Fr_Dr_Mode_display == DEF_VENT_ARROW) || (ATCU_Fr_Dr_Mode_display == DEF_VENT_FLOOR_ARROW))
		{
			Vent_Mode_FrDrOnOff = 1;
		}
		else
		{
			Vent_Mode_FrDrOnOff = 0;
		}
	
		if((ATCU_Fr_Ps_Mode_display == VENT) || (ATCU_Fr_Ps_Mode_display == B_L) || (ATCU_Fr_Ps_Mode_display == MODE7_DEF_VENT) || (ATCU_Fr_Ps_Mode_display == MODE7_DEF_VENT_FLOOR) || (ATCU_Fr_Ps_Mode_display == DEF_VENT_ARROW) || (ATCU_Fr_Ps_Mode_display == DEF_VENT_FLOOR_ARROW))
		{
			Vent_Mode_FrPassOnOff = 1;
		}
		else
		{
			Vent_Mode_FrPassOnOff = 0;
		}
	}
	else
	{
		Vent_Mode_FrDrOnOff=0;
		Vent_Mode_FrPassOnOff=0;
	}
	Rx_info_data[0].start_info = 0x01;
	Rx_info_data[0].rx_off = 0x00;
	Amo_timer_Stop(timer_0);
	Amo_timer_Start(timer_0, 2000, false, Can_rx_2_timer);
}

/*
* @brief  Rx_id_24 function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Rx_id_24(void)
{
	memset(Can_data_4_Info.reg_data, 0x00, sizeof(Can_data_4_Info.reg_data));
	memcpy(&Can_data_4_Info.reg_data[0], &recvBuff2.data[0], 8);
	can_rx_4_expire = 0;
	
	ATCU_OperationStatus = Can_data_4_Info.data.L_ATCU_OpSta;

	/*start Rx_id_24 status check */
	if((ATCU_OperationStatus == 0x00u) || (ATCU_OperationStatus == 0x02u))
	{
		if(Datc_onoff == 1)  //==1
		{

		}
		else
		{
			Datc_onoff = 1;
//							Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_EVENT);							
		}
	}
	else
	{
		if((Datc_onoff == 1) && (ATCU_OperationStatus == 0x01u))
		{
			Datc_onoff = 0;

/*
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltByte = false;
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltByte = false;
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltByte = false;
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltByte = false;
*/
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLLR_ExtFlt_Flag = false;
			FR_DTC_Flt_flag.FR_DR_SD_FLT.fltBit.FR_SLUD_ExtFlt_Flag = false;
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLLR_ExtFlt_Flag = false;
			FR_DTC_Flt_flag.FR_DR_CTR_FLT.fltBit.FR_CLUD_ExtFlt_Flag = false;
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRLR_ExtFlt_Flag = false;
			FR_DTC_Flt_flag.FR_PS_CTR_FLT.fltBit.FR_CRUD_ExtFlt_Flag = false;
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRLR_ExtFlt_Flag = false;
			FR_DTC_Flt_flag.FR_PS_SD_FLT.fltBit.FR_SRUD_ExtFlt_Flag = false;
			
			FR_DTC_Flt_flag.FR_EXT_FLT.fltByte = false;

			FR_LINBUSOff_FLAG.FR_SLLR_LIN_BusOff = false;
			FR_LINBUSOff_FLAG.FR_SLUD_LIN_BusOff = false;
			FR_LINBUSOff_FLAG.FR_CLLR_LIN_BusOff = false;
			FR_LINBUSOff_FLAG.FR_CLUD_LIN_BusOff = false;
			FR_LINBUSOff_FLAG.FR_CRLR_LIN_BusOff = false;
			FR_LINBUSOff_FLAG.FR_CRUD_LIN_BusOff = false;
			FR_LINBUSOff_FLAG.FR_SRLR_LIN_BusOff = false;
			FR_LINBUSOff_FLAG.FR_SRUD_LIN_BusOff = false;

//							Evt_queue_add(DEVICE_ACTUATOR_RECOVERY_RELEASE_EVENT);
		}
	}
	
	#if 0
	if(!event_24)
	{	
		event_22=0, event_28 = 0, event_29 = 0, event_111 = 0, event_122 = 0, event_123 = 0;
	Evt_queue_add(DEVICE_EVNT_LOG_EVENT);
	}
	#endif
	Rx_info_data[2].start_info = 0x01;
	Rx_info_data[2].rx_off = 0x00;
	Amo_timer_Stop(timer_1);
	Amo_timer_Start(timer_1, 2000, false, Can_rx_4_timer);							
}

/*
* @brief  Rx_id_28 function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Rx_id_28(void)
{
	memset(Can_data_8_Info.reg_data, 0x00, sizeof(Can_data_8_Info.reg_data));
	memcpy(&Can_data_8_Info.reg_data[0], &recvBuff2.data[0], 8);
	can_rx_8_expire = 0;
	
	ATCU_CamModeStatus = Can_data_8_Info.data.L_ATCU_CamModeSta;
	ATCU_UtilityModeUSMStatus = Can_data_8_Info.data.L_ATCU_UtilModeUsmSta;

	/*start Rx_id_28 status check */
	if((ATCU_CamModeStatus == 0x02) && (ATCU_UtilityModeUSMStatus == 0x02))
	{
		Evt_queue_add(DEVICE_CAN_EVNT_UTILITY_MODE_EVENT);
	}
	Rx_info_data[4].start_info = 0x01;
	Rx_info_data[4].rx_off = 0x00;
	Amo_timer_Stop(timer_3);
	Amo_timer_Start(timer_3, 2000, false, Can_rx_8_timer);
}

/*
* @brief  Rx_id_29 function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Rx_id_29(void)
{
	memset(Can_data_9_Info.reg_data, 0x00, sizeof(Can_data_9_Info.reg_data));
	memcpy(&Can_data_9_Info.reg_data[0], &recvBuff2.data[0], 8);
	can_rx_9_expire = 0;

	/*start Rx_id_29 setting check */	
	if(Evnt_IGN2_Onoff == 1) //IGN2
	{
		ATCU_DetentOutStatus = Can_data_9_Info.data.L_ATCU_DtntOutSta;
		ATCU_AutoBrightStatus = Can_data_9_Info.data.L_ATCU_AutoBrightSta;
		ATCU_NotMinimumModeBrightStatus = Can_data_9_Info.data.L_ATCU_NotMiniModeBrightSta;
		L_ATCU_IAUProfileValue = Can_data_9_Info.data.L_ATCU_IAUPrfleVal; 	//priority low
#if 0					
#ifdef AMO_NVM_SETTING
		if(L_ATCU_IAUProfileValue != DEFAULT_PROFILE)
		{
			lvnt_current_mode = L_ATCU_IAUProfileValue;  

			if(lvnt_current_mode != last_mode_save)
			{
				last_mode_save = lvnt_current_mode;

				/////////////////////////////////////////////////////////////////////////// 
				nv.profile = last_mode_save;																	//20241217_temp apply delete code
				nvmem_write(NVMEM_PROFILE_ID_IDX,&nv,1);
				////////////////////////////////////////////////////////////////////////////	
				Evt_queue_add(DEVICE_CAN_EVNT_LAST_MODE_EVENT);
			}
		}
#endif		
#endif
		L_ATCU_AVNProfileValue = Can_data_9_Info.data.L_ATCU_AVNPrfleVal; 	//priority high
#if 0 //def AMO_NVM_SETTING
		//if(L_ATCU_AVNProfileValue != DEFAULT_PROFILE)
		//{
			last_mode_save = L_ATCU_AVNProfileValue;	

			if(lvnt_current_mode != last_mode_save)
			{
				lvnt_current_mode = last_mode_save;

				/////////////////////////////////////////////////////////////////////////// 
				//nvmem_read(&nv);
				//nv.nv_item.profile = last_mode_save;																	//20241217_temp apply delete code
				//nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 436);
				//nvmem_write(nv.nv_data,PAGE_SIZE);
				////////////////////////////////////////////////////////////////////////////	
				Evt_queue_add(DEVICE_CAN_EVNT_LAST_MODE_EVENT);
			}
		//}
#endif
		if(evnt_mode_setting == 1)
		{
			evnt_mode_setting = 0;
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
			Evt_queue_add(DEVICE_CAN_EVNT_LAST_MODE_EVENT);
	}
	}
	Rx_info_data[5].start_info = 0x01;
	Rx_info_data[5].rx_off = 0x00;
	Amo_timer_Stop(timer_4);
	Amo_timer_Start(timer_4, 2000, false, Can_rx_9_timer);
}

/*
* @brief  Rx_id_119 function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Dr_Manual_position(void)
{
	Hu_FrDrEVntPt_success = 1;
	//memcpy(&Can_data_20_Info.reg_data[0], &recvBuff2.data[0], 8);
	
	Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_X = Hu_FrDrEVntSidePt_X_backup;
	Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_Y = Hu_FrDrEVntSidePt_Y_backup;
	Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_X = Hu_FrDrEVntCtrPt_X_backup;
	Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_Y = Hu_FrDrEVntCtrPt_Y_backup;
	
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
	if((Hu_FrDrEVntSidePt_X_invalid != 2047) && (Hu_FrDrEVntSidePt_Y_invalid != 2047))
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
		Hu_Fr_dr_Vent_Side_signal = 0x01;
		Hu_Frdr_Vent_Side_sig_backup = 0x01;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_X = Hu_FrDrEVntSidePt_X_invalid;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_Y = Hu_FrDrEVntSidePt_Y_invalid; 					
		Hu_FrDrEVntSidePt_X_reserve = Hu_FrDrEVntSidePt_X_invalid;
		Hu_FrDrEVntSidePt_Y_reserve = Hu_FrDrEVntSidePt_Y_invalid;						
	}
	else if(Hu_FrDrEVntSidePt_X_invalid != 2047)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
		Hu_Fr_dr_Vent_Side_signal = 0x01;
		Hu_Frdr_Vent_Side_sig_backup = 0x01;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_X = Hu_FrDrEVntSidePt_X_invalid; 						
		Hu_FrDrEVntSidePt_X_reserve = Hu_FrDrEVntSidePt_X_invalid;
	}
	else if(Hu_FrDrEVntSidePt_Y_invalid != 2047)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
		Hu_Fr_dr_Vent_Side_signal = 0x01;
		Hu_Frdr_Vent_Side_sig_backup = 0x01;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_Y = Hu_FrDrEVntSidePt_X_invalid; 			
		Hu_FrDrEVntSidePt_Y_reserve = Hu_FrDrEVntSidePt_X_invalid;
	}
	else
	{
	
	}
	if((Hu_FrDrEVntCtrPt_X_invalid != 2047) && (Hu_FrDrEVntCtrPt_Y_invalid != 2047))
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
		Hu_Fr_dr_Vent_Center_signal = 0x01;
		Hu_Frdr_Vent_Center_sig_backup = 0x01;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_X = Hu_FrDrEVntCtrPt_X_invalid;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_Y = Hu_FrDrEVntCtrPt_Y_invalid; 					
		Hu_FrDrEVntCtrPt_X_reserve = Hu_FrDrEVntCtrPt_X_invalid;
		Hu_FrDrEVntCtrPt_Y_reserve = Hu_FrDrEVntCtrPt_Y_invalid;						
	}
	else if(Hu_FrDrEVntCtrPt_X_invalid != 2047)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
		Hu_Fr_dr_Vent_Center_signal = 0x01;
		Hu_Frdr_Vent_Center_sig_backup = 0x01;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_X = Hu_FrDrEVntCtrPt_X_invalid; 							
		Hu_FrDrEVntCtrPt_X_reserve = Hu_FrDrEVntCtrPt_X_invalid;
	}
	else if(Hu_FrDrEVntCtrPt_Y_invalid != 2047)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
		Hu_Fr_dr_Vent_Center_signal = 0x01;
		Hu_Frdr_Vent_Center_sig_backup = 0x01;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_Y = Hu_FrDrEVntCtrPt_Y_invalid; 					
		Hu_FrDrEVntCtrPt_Y_reserve = Hu_FrDrEVntCtrPt_Y_invalid;
	}
	else
	{
		
	}
	Hu_FrDrEVntSidePt_X = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_X;
	Hu_FrDrEVntSidePt_Y = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_Y;
	Hu_FrDrEVntCtrPt_X = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_X;
	Hu_FrDrEVntCtrPt_Y = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_Y;
	
	/* FREE_MODE value setting */
	Hu_Fr_Dr_Vent_Mode = FREE_MODE;
	Hu_Fr_Dr_Vent_Mode_Remember = FREE_MODE;
	Touch_mode_Dr_set = HU_DRIVER_TOUCH;
	Evt_queue_add(DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT);
}

/*
* @brief  Rx_id_119 function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Rx_id_119(void)
{
	memset(Can_data_20_Info.reg_data, 0x00, sizeof(Can_data_20_Info.reg_data));
	//memcpy(&Can_data_20_Info.reg_data[0], &recvBuff2.data[0], 8);

	memset(Can_data_20_Info_invalid.reg_data, 0x00, sizeof(Can_data_20_Info_invalid.reg_data));
	memcpy(&Can_data_20_Info_invalid.reg_data[0], &recvBuff2.data[0], 8);

	Hu_FrDrEVntSidePt_X_invalid = Can_data_20_Info_invalid.data.L_ATCU_HU_FrDrEVntSidePt_X;
	Hu_FrDrEVntSidePt_Y_invalid = Can_data_20_Info_invalid.data.L_ATCU_HU_FrDrEVntSidePt_Y;
	Hu_FrDrEVntCtrPt_X_invalid = Can_data_20_Info_invalid.data.L_ATCU_HU_FrDrEVntCtrPt_X;
	Hu_FrDrEVntCtrPt_Y_invalid = Can_data_20_Info_invalid.data.L_ATCU_HU_FrDrEVntCtrPt_Y;

#ifdef AMO_GN7_PE_SETTING_NONE
	if((Evnt_IGN2_Onoff == 1) || (Evnt_IGN3_Onoff == 1))	//IGN2 or IGN3 ON
#else
	if(Evnt_IGN2_Onoff == 1)	//IGN2
#endif
	{
		if(Vent_Mode_FrDrOnOff == 1)
		{
			if((Hu_FrDrEVntSidePt_X_invalid != 2047) || (Hu_FrDrEVntSidePt_Y_invalid != 2047)  || (Hu_FrDrEVntCtrPt_X_invalid != 2047) || (Hu_FrDrEVntCtrPt_Y_invalid != 2047))
			{ 					
				if(Hu_FrDrEVntPt_success == 0)
				{
					Dr_Manual_position();
				}
			}
			else
			{
				Hu_FrDrEVntPt_success = 0;
			} 						
		}
	}
}

/*
* @brief  Rx_id_131 function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Ps_Manual_position(void)
{
	Hu_FrPsEVntSidePt_success =1;
	memcpy(&Can_data_21_Info.reg_data[0], &recvBuff2.data[0], 8);

	Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_X = Hu_FrPsEVntSidePt_X_backup;
	Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_Y = Hu_FrPsEVntSidePt_Y_backup;
	Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_X = Hu_FrPsEVntCtrPt_X_backup;
	Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_Y = Hu_FrPsEVntCtrPt_Y_backup;

	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x04; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
	if((Hu_FrPsEVntSidePt_X_invalid != 2047) && (Hu_FrPsEVntSidePt_Y_invalid != 2047))
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
		Hu_Fr_Ps_Vent_Side_signal = 0x01;
		Hu_FrPs_Vent_Side_sig_backup = 0x01;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_X = Hu_FrPsEVntSidePt_X_invalid;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_Y = Hu_FrPsEVntSidePt_Y_invalid;
		Hu_FrPsEVntSidePt_X_reserve = Hu_FrPsEVntSidePt_X_invalid;
		Hu_FrPsEVntSidePt_Y_reserve = Hu_FrPsEVntSidePt_Y_invalid;
	}
	else if(Hu_FrPsEVntSidePt_X_invalid != 2047)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
		Hu_Fr_Ps_Vent_Side_signal = 0x01;
		Hu_FrPs_Vent_Side_sig_backup = 0x01;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_X = Hu_FrPsEVntSidePt_X_invalid;			
		Hu_FrPsEVntSidePt_X_reserve = Hu_FrPsEVntSidePt_X_invalid;
	}
	else if(Hu_FrPsEVntSidePt_Y_invalid != 2047)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
		Hu_Fr_Ps_Vent_Side_signal = 0x01;
		Hu_FrPs_Vent_Side_sig_backup = 0x01;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_Y = Hu_FrPsEVntSidePt_Y_invalid;
		Hu_FrPsEVntSidePt_Y_reserve = Hu_FrPsEVntSidePt_Y_invalid;
	}
	else
	{
		
	}
	
	if((Hu_FrPsEVntCtrPt_X_invalid != 2047) && (Hu_FrPsEVntCtrPt_Y_invalid != 2047))
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
		Hu_Fr_Ps_Vent_Center_signal = 0x01;
		Hu_FrPs_Vent_Center_sig_backup = 0x01;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_X = Hu_FrPsEVntCtrPt_X_invalid;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_Y = Hu_FrPsEVntCtrPt_Y_invalid;		
		Hu_FrPsEVntCtrPt_X_reserve = Hu_FrPsEVntCtrPt_X_invalid;
		Hu_FrPsEVntCtrPt_Y_reserve = Hu_FrPsEVntCtrPt_Y_invalid;
	}
	else if(Hu_FrPsEVntCtrPt_X_invalid != 2047)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
		Hu_Fr_Ps_Vent_Center_signal = 0x01;
		Hu_FrPs_Vent_Center_sig_backup = 0x01;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_X = Hu_FrPsEVntCtrPt_X_invalid;	
		Hu_FrPsEVntCtrPt_X_reserve = Hu_FrPsEVntCtrPt_X_invalid;
	}
	else if(Hu_FrPsEVntCtrPt_Y_invalid != 2047)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
		Hu_Fr_Ps_Vent_Center_signal = 0x01;
		Hu_FrPs_Vent_Center_sig_backup = 0x01;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_Y = Hu_FrPsEVntCtrPt_Y_invalid;				
		Hu_FrPsEVntCtrPt_Y_reserve = Hu_FrPsEVntCtrPt_Y_invalid;
	}
	else
	{
	
	}
	
	Hu_FrPsEVntSidePt_X = Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_X;
	Hu_FrPsEVntSidePt_Y = Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_Y;
	Hu_FrPsEVntCtrPt_X = Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_X;
	Hu_FrPsEVntCtrPt_Y = Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_Y;

	/* FREE_MODE value setting */
	Hu_Fr_Ps_Vent_Mode = FREE_MODE;
	Hu_Fr_Ps_Vent_Mode_Remember = FREE_MODE;
	Touch_mode_Ps_set = HU_PASSENGER_TOUCH;
	Evt_queue_add(DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT);	
}

/*
* @brief  Rx_id_131 function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Rx_id_131(void)
{
	memset(Can_data_21_Info.reg_data, 0x00, sizeof(Can_data_21_Info.reg_data));
	//memcpy(&Can_data_21_Info.reg_data[0], &recvBuff2.data[0], 8);

	memset(Can_data_21_Info_invalid.reg_data, 0x00, sizeof(Can_data_21_Info_invalid.reg_data));
	memcpy(&Can_data_21_Info_invalid.reg_data[0], &recvBuff2.data[0], 8);

	Hu_FrPsEVntSidePt_X_invalid = Can_data_21_Info_invalid.data.L_ATCU_HU_FrPsEVntSidePt_X;
	Hu_FrPsEVntSidePt_Y_invalid = Can_data_21_Info_invalid.data.L_ATCU_HU_FrPsEVntSidePt_Y;
	Hu_FrPsEVntCtrPt_X_invalid = Can_data_21_Info_invalid.data.L_ATCU_HU_FrPsEVntCtrPt_X;
	Hu_FrPsEVntCtrPt_Y_invalid = Can_data_21_Info_invalid.data.L_ATCU_HU_FrPsEVntCtrPt_Y; 


#ifdef AMO_GN7_PE_SETTING_NONE
	if((Evnt_IGN2_Onoff == 1) || (Evnt_IGN3_Onoff == 1))	//IGN2 or IGN3 ON
#else
	if(Evnt_IGN2_Onoff == 1) //IGN2
#endif
	{
		if(Vent_Mode_FrPassOnOff == 1)
		{
			if((Hu_FrPsEVntSidePt_X_invalid != 2047) || (Hu_FrPsEVntSidePt_Y_invalid != 2047)  || (Hu_FrPsEVntCtrPt_X_invalid != 2047) || (Hu_FrPsEVntCtrPt_Y_invalid != 2047))
			{
				if(Hu_FrPsEVntSidePt_success == 0)
				{
					Ps_Manual_position();
				}
			}
			else
			{
				Hu_FrPsEVntSidePt_success =0;
			} 						
		}
	}
}

/*
* @brief  Rx_id_161 function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Rx_id_161(void)
{
	memset(Can_data_32_Info.reg_data, 0x00, sizeof(Can_data_32_Info.reg_data));
	memcpy(&Can_data_32_Info.reg_data[0], &recvBuff2.data[0], 8);
#if 0
	if(!P_Oip_Cts_priority)
	{
		P_Oip_Cts_Select = P_OIP_SELECT;
		P_Oip_Cts_priority = 1;
	}
#endif
	/* vent mode  value setting */
	Hu_Fr_Dr_Vent_Mode_invalid = (Evnt_mode)Can_data_32_Info.data.L_ATCU_HU_FrDrEVntModeSet;
	Hu_Fr_Ps_Vent_Mode_invalid = (Evnt_mode)Can_data_32_Info.data.L_ATCU_HU_FrPsEVntModeSet;
	Hu_Fr_dr_Vent_Center_signal_invalid = Can_data_32_Info.data.L_ATCU_HU_FrDrEVntCtrOpnCls;
	Hu_Fr_dr_Vent_Side_signal_invalid = Can_data_32_Info.data.L_ATCU_HU_FrDrEVntSlideOpnCls;
	Hu_Fr_Ps_Vent_Center_signal_invalid = Can_data_32_Info.data.L_ATCU_HU_FrPsEVntCtrOpnCls;
	Hu_Fr_Ps_Vent_Side_signal_invalid = Can_data_32_Info.data.L_ATCU_HU_FrPsEVntSideOpnCls;

#ifdef AMO_GN7_PE_SETTING_NONE
	if((Evnt_IGN2_Onoff == 1) || (Evnt_IGN3_Onoff == 1))	//IGN2 or IGN3 ON
#else
	if(Evnt_IGN2_Onoff == 1) //IGN2
#endif
	{
		if((Vent_Mode_FrDrOnOff == 1) || (Vent_Mode_FrPassOnOff == 1))
		{
			if((Hu_Fr_dr_Vent_Center_signal_invalid != 0x03) || (Hu_Fr_dr_Vent_Side_signal_invalid != 0x03) || (Hu_Fr_Ps_Vent_Center_signal_invalid != 0x03) || (Hu_Fr_Ps_Vent_Side_signal_invalid != 0x03))
			{
				Can_Full_close(); 					
			}
			else
			{
				Hu_Fr_dr_Vent_Center_signal_success = 0;
				Hu_Fr_dr_Vent_Side_signal_success = 0;
				Hu_Fr_Ps_Vent_Center_signal_success = 0;
				Hu_Fr_Ps_Vent_Side_signal_success = 0;
				Hu_Fr_dr_Vent_Side_change = 0;
				Hu_Fr_dr_Vent_Center_change = 0;
				Hu_Fr_Ps_Vent_Side_change = 0;
				Hu_Fr_Ps_Vent_Center_change = 0;						
			}
				
			if((Hu_Fr_Dr_Vent_Mode_invalid != INVALID_VALUE) || (Hu_Fr_Ps_Vent_Mode_invalid != INVALID_VALUE))
			{
				Can_Vent_mode();
			}
			else
			{
				Hu_Fr_Dr_Vent_Mode_invalid = INVALID_VALUE;
				Hu_Fr_Ps_Vent_Mode_invalid = INVALID_VALUE;
				Hu_Fr_Dr_Vent_Mode_success = 0;
				Hu_Fr_Ps_Vent_Mode_success = 0;
			}
		}
		else
		{
			P_Oip_Cts_priority = 0;
		}
	}
	else
	{
		P_Oip_Cts_priority = 0;
	} 	
}

/*
* @brief  Rx_id_505 function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Rx_id_505(void)
{
	/* rcv buffer len return*/
	rx_len = recvBuff2.dataLen;
	memset(Rx_candata, 0x00, sizeof(Rx_candata));
	memcpy(&Rx_candata[0], &recvBuff2.data[0], 8);
		
	pci = Rx_candata[0];
	memset(Tx_candata, 0xAA, sizeof(Tx_candata));

	/* single/multi frame check */
	if((pci & 0xF0u) == 0x00u)
	{
		if((pci & 0x0Fu) != 0x00u)
		{
			dtc_serviceId = Rx_candata[1];
			dtc_subFunc = Rx_candata[2];		
			dtcstatus = Rx_candata[3];	
			dtc_data_length = pci & 0x0Fu;
		}
		else
		{
			dtc_serviceId = Rx_candata[2];
			dtc_subFunc = Rx_candata[3];		
			dtcstatus = Rx_candata[4];	
			dtc_data_length = Rx_candata[1];							
		}
		/* dtc service id check */	
		if(dtc_serviceId == 0x30)
		{ 				
			switch(dtc_subFunc)
			{
				case 0x01: //SW VERSION
				{
					Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
					break;
				}
				case 0x02: //DEVICE RESET
				{
					Evt_queue_add(DEVICE_EVNT_DTC_EVENT); 					
					break;
				}
				case 0x03:	
				{
					Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
					break;
				}
				case 0x04: 
				{
					Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
					break;
				}
				case 0x05: 
				{
					Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
					break;
				}
				case 0x06: 
				{
					Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
					break;
				}
				case 0x07: 
				{
					Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
					break;
				}
				case 0x08: 
				{
					Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
					break;
				} 		
				case 0x09: 
				{
					Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
					break;
				} 	
				case 0x0A: 
				{
					Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
					break;
				} 	
				case 0x10: 
				{
					Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
					break;
				} 
				case 0x11: //SLEEP
				{
					Amo_timer_Stop(timer_7);
					Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
					break;
				} 								
				default:
				Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
				break;
			}
		}
		/* dtc service id 0x34 check */	
		if(dtc_serviceId == 0x34) //OTA Init
		{ 				
			switch(dtc_subFunc)
			{
			case 0x01: //SW VERSION
			{
				Evt_queue_add(DEVICE_EVNT_OTA_EVENT);
				break;
			}
			case 0x02: 
			{
				Evt_queue_add(DEVICE_EVNT_OTA_EVENT); 					
				break;
			} 					
			default:
			Evt_queue_add(DEVICE_EVNT_OTA_EVENT);
			break;
			}
		} 	
		/* dtc service id 0x37 check */	
		if(dtc_serviceId == 0x37) //OTA Checksum & update
		{
			switch(dtc_subFunc)
			{
			case 0x01: //CRC CHECK
			{
				Evt_queue_add(DEVICE_EVNT_OTA_EVENT);
				break;
			}
			case 0x02: //FW UPDATE
			{
				Evt_queue_add(DEVICE_EVNT_OTA_EVENT); 					
				break;
			} 					
			default:
			Evt_queue_add(DEVICE_EVNT_OTA_EVENT);
			break;
			}
		} 						
	}
	
	if(((pci & 0xF0u) >> 4)== 0x03)
	{
		Evt_queue_add(DEVICE_EVNT_OTA_EVENT);
	}
	/* continous fram check */	
	if(((pci & 0xF0u) >> 4) == 0x02)
	{ 				
		WDOG_DRV_Trigger(INST_WATCHDOG1);
		rxFIFOdone = 0;
		FLEXCAN_DRV_RxFifo(INST_CANCOM1,&recvBuff2);
	
		//uint8_t Tx_Ota[8] = {0,};
		/* sn_number return*/
		sn_number = ((uint16_t)pci & 0x0Fu);
		//memset(Tx_Ota, 0xAA, sizeof(Tx_Ota));
		
		if(sn_number != expected_sn_number)
		{
			//Tx_candata[0] = (0x0 << 4) | (dtc_data_length+2);
			//Tx_Ota[0] = 0x7F;
			//Tx_Ota[1] = 0x36;
			//Tx_Ota[2] = sn_number;				
			//Tx_Ota[3] = expected_sn_number; //0x73; 
			//SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota));
		}
		else
		{
		#ifdef DEBUG_ON
			//printf("sn_number = %d expected_sn_number = %d \r\n", sn_number, expected_sn_number);
		#endif
			memcpy(&Write_Buffer[sn_index], &Rx_candata[1], ((size_t)rx_len-1u));
			sn_index += ((uint16_t)rx_len -1);
			expected_sn_number = (expected_sn_number + 1) & MAX_SN_NUMBER;
		#ifdef DEBUG_ON
			//printf("%02x %02x %02x %02x %02x %02x %02x length = %d \r\n", recvBuff2.data[1], recvBuff2.data[2], recvBuff2.data[3], recvBuff2.data[4], recvBuff2.data[5], recvBuff2.data[6], recvBuff2.data[7], recvBuff2.dataLen);
		#endif
			//Tx_Ota[0] = 0x02;
			//Tx_Ota[1] = 0x76;
			//Tx_Ota[2] = sn_number;				
			//memset(Write_Buffer, 0x00, sizeof(Write_Buffer));
			//SendCANData(24, tx_id, Tx_Ota, sizeof(Tx_Ota)); 	
		}
	} 			
	if(((pci & 0xF0u) >> 4) == 0x01u) //multi frame
	{
		Evt_queue_add(DEVICE_EVNT_OTA_EVENT); 
	} 				
	
	if(((Rx_candata[0] == 0x79u) && (Rx_candata[1] == 0x32u)) && (Rx_candata[2] == 0x84u))
	{
		Evt_queue_add(DEVICE_EVNT_DTC_EVENT);
	}
}

/*
* @brief  Can_Full_close_mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_Full_close_mode(void)
{
	VentSignal signals[] = {{&Hu_Fr_dr_Vent_Center_signal_invalid, &Hu_Fr_dr_Vent_Center_signal_success, &Hu_Fr_dr_Vent_Center_signal},
													{&Hu_Fr_dr_Vent_Side_signal_invalid, &Hu_Fr_dr_Vent_Side_signal_success, &Hu_Fr_dr_Vent_Side_signal},
													{&Hu_Fr_Ps_Vent_Center_signal_invalid, &Hu_Fr_Ps_Vent_Center_signal_success, &Hu_Fr_Ps_Vent_Center_signal},
													{&Hu_Fr_Ps_Vent_Side_signal_invalid, &Hu_Fr_Ps_Vent_Side_signal_success, &Hu_Fr_Ps_Vent_Side_signal}};

	const uint32_t num_signal = sizeof(signals)/sizeof(signals[0]);

	/*start Full close mode value check */
	for(uint32_t i = 0; i < num_signal; i++)
	{
		if(*(signals[i].signal_invalid) != 0x03)
		{
			*(signals[i].signal_success) = 1;
			*(signals[i].signal) = *(signals[i].signal_invalid); 
		}
	}
}

/*
* @brief  Can_Full_close_value function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_Full_close_value(void)
{
	if((Hu_Fr_dr_Vent_Side_signal == 0x02) && (Hu_Fr_dr_Vent_Center_signal == 0x02))
	{
		/* dr/pass mode full close */	
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00; 														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x02;						
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x02; 	
		Hu_Frdr_Vent_Side_sig_backup = 0x02;
		Hu_Frdr_Vent_Center_sig_backup = 0x02;
	}
	else if(Hu_Fr_dr_Vent_Side_signal == 0x02)
	{
		/* driver full close */	
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00; 														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x02;
		Hu_Frdr_Vent_Side_sig_backup = 0x02;
		Hu_Fr_dr_Vent_Center_signal = 0x01;
		Hu_Fr_dr_Vent_Center_change = 0x01;
	}
	else if(Hu_Fr_dr_Vent_Center_signal == 0x02)
	{
		/* passenger full close */	
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00; 														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x02;
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x02; 	
		Hu_Frdr_Vent_Center_sig_backup = 0x02;
		Hu_Fr_dr_Vent_Side_signal = 0x01;
		Hu_Fr_dr_Vent_Side_change = 0x01;
	}
	else
	{
	
	}
	
	if((Hu_Fr_Ps_Vent_Side_signal == 0x02) && (Hu_Fr_Ps_Vent_Center_signal == 0x02))
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00; 														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x02;
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x02; 	
		Hu_FrPs_Vent_Side_sig_backup = 0x02;
		Hu_FrPs_Vent_Center_sig_backup = 0x02;
	}
	else if(Hu_Fr_Ps_Vent_Side_signal == 0x02)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00; 														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x02;
		//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x02; 	
		Hu_FrPs_Vent_Side_sig_backup = 0x02;
		Hu_Fr_Ps_Vent_Center_signal = 0x01;
		Hu_Fr_Ps_Vent_Center_change = 0x01;
	}	
	else if(Hu_Fr_Ps_Vent_Center_signal == 0x02)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00; 														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x02;
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x02; 		
		Hu_FrPs_Vent_Center_sig_backup = 0x02;
		Hu_Fr_Ps_Vent_Side_signal = 0x01;
		Hu_Fr_Ps_Vent_Side_change = 0x01;
	} 
	else
	{	
	}
}

/*
* @brief  Can_Full_close_open function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_Full_close_open(void)
{
	if((Hu_Fr_dr_Vent_Side_change == 0x01) || (Hu_Fr_dr_Vent_Side_signal_invalid == 0x01))
	{
		//if(Hu_Fr_Dr_Vent_Mode == 0x00)
		//{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04;	//(uint8_t)Hu_Fr_Dr_Vent_Mode_Remember; 	//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
			Hu_Frdr_Vent_Side_sig_backup = 0x01;
			Hu_Fr_Dr_Vent_Mode = FREE_MODE;
			//memcpy(&Can_data_20_Info.reg_data[0], &Can_Tx_Evnt_1.reg_data[0], sizeof(Can_Tx_Evnt_1.reg_data));
		//}
	}

	if((Hu_Fr_dr_Vent_Center_change == 0x01) || (Hu_Fr_dr_Vent_Center_signal_invalid == 0x01))
	{
		//if(Hu_Fr_Dr_Vent_Mode == 0x00)
		//{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04;	//(uint8_t)Hu_Fr_Dr_Vent_Mode_Remember; 	//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
			Hu_Frdr_Vent_Center_sig_backup = 0x01;
			Hu_Fr_Dr_Vent_Mode = FREE_MODE;
			//memcpy(&Can_data_20_Info.reg_data[0], &Can_Tx_Evnt_1.reg_data[0], sizeof(Can_Tx_Evnt_1.reg_data));
		//}
	}

	if((Hu_Fr_Ps_Vent_Side_change == 0x01) || (Hu_Fr_Ps_Vent_Side_signal_invalid == 0x01))
	{
		//if(Hu_Fr_Ps_Vent_Mode == 0x00)
		//{
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x04;	//(uint8_t)Hu_Fr_Ps_Vent_Mode_Remember; 	//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
			Hu_FrPs_Vent_Side_sig_backup = 0x01;
			Hu_Fr_Ps_Vent_Mode = FREE_MODE;
			//memcpy(&Can_data_21_Info.reg_data[0], &Can_Tx_Evnt_2.reg_data[0], sizeof(Can_Tx_Evnt_2.reg_data));
		//}
	} 			

	if((Hu_Fr_Ps_Vent_Center_change == 0x01) || (Hu_Fr_Ps_Vent_Center_signal_invalid == 0x01))
	{
		//if(Hu_Fr_Ps_Vent_Mode == 0x00)
		//{
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x04;	//(uint8_t)Hu_Fr_Ps_Vent_Mode_Remember; 	//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
			Hu_FrPs_Vent_Center_sig_backup = 0x01;
			Hu_Fr_Ps_Vent_Mode = FREE_MODE;
			//memcpy(&Can_data_21_Info.reg_data[0], &Can_Tx_Evnt_2.reg_data[0], sizeof(Can_Tx_Evnt_2.reg_data));
		//}
	} 	
}
#if 0
void Can_Open_Facing_mode(void)
{
	if(((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01)) || ((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01)))
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x01; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
		//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
		//Hu_Frdr_Vent_Side_sig_backup = 0x01;
		//Hu_Frdr_Vent_Center_sig_backup = 0x01;
		Hu_Fr_Dr_Vent_Mode_Remember = FACING_MODE;
	}
	
	if(((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01)) || ((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01)))
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x01; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
		//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01; 
		//Hu_FrPs_Vent_Side_sig_backup = 0x01;
		//Hu_FrPs_Vent_Center_sig_backup = 0x01;
		Hu_Fr_Ps_Vent_Mode_Remember = FACING_MODE;										
	}
}

void Can_Open_Avoid_mode(void)
{
	if(((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01)) || ((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01)))
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x02; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
		//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
		//Hu_Frdr_Vent_Side_sig_backup = 0x01;
		//Hu_Frdr_Vent_Center_sig_backup = 0x01;											
		Hu_Fr_Dr_Vent_Mode_Remember = AVOID_MODE;
	}
	
	if(((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01)) || ((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01)))
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x02; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
		//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01; 	
		//Hu_FrPs_Vent_Side_sig_backup = 0x01;
		//Hu_FrPs_Vent_Center_sig_backup = 0x01;											
		Hu_Fr_Ps_Vent_Mode_Remember = AVOID_MODE; 									
	}
}

void Can_Open_Swing_mode(void)
{
	if(((Hu_Fr_Dr_Vent_Mode == SWING_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01)) || ((Hu_Fr_Dr_Vent_Mode == SWING_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01)))
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x03; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
		//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
		//Hu_Frdr_Vent_Side_sig_backup = 0x01;
		//Hu_Frdr_Vent_Center_sig_backup = 0x01;											
		Hu_Fr_Dr_Vent_Mode_Remember = SWING_MODE;
		button_LH_cycle_check = 0x03;
	}
	
	if(((Hu_Fr_Ps_Vent_Mode == SWING_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01)) || ((Hu_Fr_Ps_Vent_Mode == SWING_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01)))
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x03; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
		//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01; 	
		//Hu_FrPs_Vent_Side_sig_backup = 0x01;
		//Hu_FrPs_Vent_Center_sig_backup = 0x01;										
		Hu_Fr_Ps_Vent_Mode_Remember = SWING_MODE; 		
		button_RH_cycle_check = 0x03;
	}
}
#endif
/*
* @brief  Can_Open_Manual_mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_Open_Manual_mode(void)
{
	if(((Hu_Fr_Dr_Vent_Mode == FREE_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01)) || ((Hu_Fr_Dr_Vent_Mode == FREE_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01)))
	{
		/* manual mode return*/
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Hu_Fr_Dr_Vent_Mode_Remember = FREE_MODE;
		Touch_mode_Dr_set = HU_DRIVER_TOUCH;
		button_LH_cycle_check = 0x04;
		
		if(Hu_Fr_dr_Vent_Side_signal != 0x02)
		{
			Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_X = Hu_FrDrEVntSidePt_X_reserve;
			//Hu_FrDrEVntSidePt_X_invalid = Hu_FrDrEVntSidePt_X_reserve;
			Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_Y = Hu_FrDrEVntSidePt_Y_reserve;
			//Hu_FrDrEVntSidePt_Y_invalid = Hu_FrDrEVntSidePt_Y_reserve;
		}
		if(Hu_Fr_dr_Vent_Center_signal != 0x02)
		{
			Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_X = Hu_FrDrEVntCtrPt_X_reserve;
			//Hu_FrDrEVntCtrPt_X_invalid = Hu_FrDrEVntCtrPt_X_reserve;
			Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_Y = Hu_FrDrEVntCtrPt_Y_reserve;	
			//Hu_FrDrEVntCtrPt_Y_invalid = Hu_FrDrEVntCtrPt_Y_reserve;
		}
	}

	if(((Hu_Fr_Ps_Vent_Mode == FREE_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01)) || ((Hu_Fr_Ps_Vent_Mode == FREE_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01)))
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x04; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5	
		Hu_Fr_Ps_Vent_Mode_Remember = FREE_MODE;
		Touch_mode_Ps_set = HU_PASSENGER_TOUCH;
		button_RH_cycle_check = 0x04;
		if(Hu_Fr_Ps_Vent_Side_signal != 0x02)
		{
			Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_X = Hu_FrPsEVntSidePt_X_reserve;
			//Hu_FrPsEVntSidePt_X_invalid = Hu_FrPsEVntSidePt_X_reserve;
			Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_Y = Hu_FrPsEVntSidePt_Y_reserve;
			//Hu_FrPsEVntSidePt_Y_invalid = Hu_FrPsEVntSidePt_Y_reserve;
		}
		if(Hu_Fr_Ps_Vent_Center_signal != 0x02)
		{
			Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_X = Hu_FrPsEVntCtrPt_X_reserve;
			//Hu_FrPsEVntCtrPt_X_invalid = Hu_FrPsEVntCtrPt_X_reserve;
			Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_Y = Hu_FrPsEVntCtrPt_Y_reserve;	
			//Hu_FrPsEVntCtrPt_Y_invalid = Hu_FrPsEVntCtrPt_Y_reserve;
		}
	} 
}

/*
* @brief  Can_Full_close function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_Full_close(void)
{
	if((((Hu_Fr_dr_Vent_Center_signal_success == 0) && (Hu_Fr_dr_Vent_Side_signal_success == 0)) && (Hu_Fr_Ps_Vent_Center_signal_success == 0)) && (Hu_Fr_Ps_Vent_Side_signal_success == 0))
	{
		Can_Full_close_mode();	
		
		if((Hu_Fr_dr_Vent_Side_signal == 0x02) || (Hu_Fr_dr_Vent_Center_signal == 0x02) || (Hu_Fr_Ps_Vent_Side_signal == 0x02) || (Hu_Fr_Ps_Vent_Center_signal == 0x02))
		{
			Can_Full_close_value();				
			Evt_queue_add(DEVICE_P_OIP_FULL_CLOSE_CAN_LIN_EVENT);
		}
		Can_Full_close_open();

		#if 0
		if(((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01)) || ((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01)) || ((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01)) || ((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01)))
		{
			Can_Open_Facing_mode();
			Evt_queue_add(DEVICE_P_OIP_FOCUS_CAN_LIN_EVENT);								
		}
	
		if(((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01)) || ((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01)) || ((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01)) || ((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01)))
		{
			Can_Open_Avoid_mode();
			Evt_queue_add(DEVICE_P_OIP_SPREAD_CAN_LIN_EVENT); 							
		} 						
	
		if(((Hu_Fr_Dr_Vent_Mode == SWING_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01)) || ((Hu_Fr_Dr_Vent_Mode == SWING_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01)) || ((Hu_Fr_Ps_Vent_Mode == SWING_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01)) || ((Hu_Fr_Ps_Vent_Mode == SWING_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01)))
		{
			Can_Open_Swing_mode();
			Evt_queue_add(DEVICE_P_OIP_CYCLE_CAN_LIN_EVENT);								
		} 
		#endif
		
		if(((Hu_Fr_Dr_Vent_Mode == FREE_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01)) || ((Hu_Fr_Dr_Vent_Mode == FREE_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01)) || ((Hu_Fr_Ps_Vent_Mode == FREE_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01)) || ((Hu_Fr_Ps_Vent_Mode == FREE_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01)))
		{					
			Can_Open_Manual_mode();
			Evt_queue_add(DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT);
	
			//if(P_Oip_Cts_Select == P_OIP_SELECT)
			///{
				Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x01; //APPLY Duplucate Input Signal Prevention Control Status return
			//}
			//else
			//{
				Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x01; //NOT APPLY
			//}
		} 	
	}

}

/*
* @brief  Can_Vent_mode_set function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_Vent_mode_set(void)
{
	/*start vent mode value check */
	if(Hu_Fr_Dr_Vent_Mode_invalid != INVALID_VALUE)
	{
		//if(Hu_Fr_Dr_Vent_Mode_success != 1)
		//{
			Hu_Fr_Dr_Vent_Mode_success = 1;
			Hu_Fr_Dr_Vent_Mode = (Evnt_mode)Hu_Fr_Dr_Vent_Mode_invalid; 										
		//}
	}
	
	if(Hu_Fr_Ps_Vent_Mode_invalid != INVALID_VALUE)
	{
		//if(Hu_Fr_Ps_Vent_Mode_success != 1)
		//{
			Hu_Fr_Ps_Vent_Mode_success = 1;
			Hu_Fr_Ps_Vent_Mode = (Evnt_mode)Hu_Fr_Ps_Vent_Mode_invalid; 											
		//}
	}
}

/*
* @brief  Can_Facing_mode_set function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_Facing_mode_set(void)
{
	
	if(Hu_Fr_Dr_Vent_Mode_invalid == FACING_MODE)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x01; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
		/* facing mode return*/
		Hu_Fr_dr_Vent_Side_signal = 0x01, Hu_Fr_dr_Vent_Center_signal = 0x01;		
		Hu_Frdr_Vent_Side_sig_backup = 0x01;
		Hu_Frdr_Vent_Center_sig_backup = 0x01;
		Hu_Fr_Dr_Vent_Mode_Remember = FACING_MODE;
	}
	
	if(Hu_Fr_Ps_Vent_Mode_invalid == FACING_MODE)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x01; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01; 
		/* facing mode return*/
		Hu_Fr_Ps_Vent_Side_signal = 0x01, Hu_Fr_Ps_Vent_Center_signal = 0x01;
		Hu_FrPs_Vent_Side_sig_backup = 0x01;
		Hu_FrPs_Vent_Center_sig_backup = 0x01;
		Hu_Fr_Ps_Vent_Mode_Remember = FACING_MODE;										
	}
}

/*
* @brief  Can_Avoid_mode_set function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_Avoid_mode_set(void)
{
	if(Hu_Fr_Dr_Vent_Mode_invalid == AVOID_MODE)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x02; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
		/* Avoid mode return*/
		Hu_Fr_dr_Vent_Side_signal = 0x01, Hu_Fr_dr_Vent_Center_signal = 0x01;
		Hu_Frdr_Vent_Side_sig_backup = 0x01;
		Hu_Frdr_Vent_Center_sig_backup = 0x01;									
		Hu_Fr_Dr_Vent_Mode_Remember = AVOID_MODE;
	}
	
	if(Hu_Fr_Ps_Vent_Mode_invalid == AVOID_MODE)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x02; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
		/* Avoid mode return*/
		Hu_Fr_Ps_Vent_Side_signal = 0x01, Hu_Fr_Ps_Vent_Center_signal = 0x01;
		Hu_FrPs_Vent_Side_sig_backup = 0x01;
		Hu_FrPs_Vent_Center_sig_backup = 0x01;										
		Hu_Fr_Ps_Vent_Mode_Remember = AVOID_MODE; 									
	}
}

/*
* @brief  Can_Cycle_mode_set function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_Cycle_mode_set(void)
{
	if(Hu_Fr_Dr_Vent_Mode_invalid == SWING_MODE)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x03; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
		/* Cycle mode return*/
		Hu_Fr_dr_Vent_Side_signal = 0x01, Hu_Fr_dr_Vent_Center_signal = 0x01;
		Hu_Frdr_Vent_Side_sig_backup = 0x01;
		Hu_Frdr_Vent_Center_sig_backup = 0x01;										
		Hu_Fr_Dr_Vent_Mode_Remember = SWING_MODE;
		button_LH_cycle_check = 0x03;
	}
	
	if(Hu_Fr_Ps_Vent_Mode_invalid == SWING_MODE)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x03; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01; 
		/* Cycle mode return*/
		Hu_Fr_Ps_Vent_Side_signal = 0x01, Hu_Fr_Ps_Vent_Center_signal = 0x01;
		Hu_FrPs_Vent_Side_sig_backup = 0x01;
		Hu_FrPs_Vent_Center_sig_backup = 0x01;										
		Hu_Fr_Ps_Vent_Mode_Remember = SWING_MODE; 		
		button_RH_cycle_check = 0x03;
	}
}

/*
* @brief  Can_Manual_mode_set function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_Manual_mode_set(void)
{
	if(Hu_Fr_Dr_Vent_Mode_invalid == FREE_MODE)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
		Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
		/* Manual mode return*/
		Hu_Fr_dr_Vent_Side_signal = 0x01, Hu_Fr_dr_Vent_Center_signal = 0x01;
		Hu_Frdr_Vent_Side_sig_backup = 0x01;
		Hu_Frdr_Vent_Center_sig_backup = 0x01;		
		Hu_Fr_Dr_Vent_Mode_Remember = FREE_MODE;
		Touch_mode_Dr_set = HU_DRIVER_TOUCH;
		button_LH_cycle_check = 0x04;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_X = Hu_FrDrEVntSidePt_X_reserve;
		Hu_FrDrEVntSidePt_X_invalid = Hu_FrDrEVntSidePt_X_reserve;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_Y = Hu_FrDrEVntSidePt_Y_reserve;
		Hu_FrDrEVntSidePt_Y_invalid = Hu_FrDrEVntSidePt_Y_reserve;

		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_X = Hu_FrDrEVntCtrPt_X_reserve;
		Hu_FrDrEVntCtrPt_X_invalid = Hu_FrDrEVntCtrPt_X_reserve;
		Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_Y = Hu_FrDrEVntCtrPt_Y_reserve;	
		Hu_FrDrEVntCtrPt_Y_invalid = Hu_FrDrEVntCtrPt_Y_reserve;
	}
	
	if(Hu_Fr_Ps_Vent_Mode_invalid == FREE_MODE)
	{
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x04; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
		Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01; 
		/* Manual mode return*/
		Hu_Fr_Ps_Vent_Side_signal = 0x01, Hu_Fr_Ps_Vent_Center_signal = 0x01;
		Hu_FrPs_Vent_Side_sig_backup = 0x01;
		Hu_FrPs_Vent_Center_sig_backup = 0x01;		
		Hu_Fr_Ps_Vent_Mode_Remember = FREE_MODE;
		Touch_mode_Ps_set = HU_PASSENGER_TOUCH;
		button_RH_cycle_check = 0x04;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_X = Hu_FrPsEVntSidePt_X_reserve;
		Hu_FrPsEVntSidePt_X_invalid = Hu_FrPsEVntSidePt_X_reserve;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_Y = Hu_FrPsEVntSidePt_Y_reserve;
		Hu_FrPsEVntSidePt_Y_invalid = Hu_FrPsEVntSidePt_Y_reserve;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_X = Hu_FrPsEVntCtrPt_X_reserve;
		Hu_FrPsEVntCtrPt_X_invalid = Hu_FrPsEVntCtrPt_X_reserve;
		Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_Y = Hu_FrPsEVntCtrPt_Y_reserve;	
		Hu_FrPsEVntCtrPt_Y_invalid = Hu_FrPsEVntCtrPt_Y_reserve;	
	} 
}

/*
* @brief  Can_Vent_mode function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
static void Can_Vent_mode(void)
{
	if((Hu_Fr_Dr_Vent_Mode_success == 0) && (Hu_Fr_Ps_Vent_Mode_success == 0))
	{
		Can_Vent_mode_set();

		if((Hu_Fr_Dr_Vent_Mode_invalid == FACING_MODE) || (Hu_Fr_Ps_Vent_Mode_invalid == FACING_MODE))
		{
			Can_Facing_mode_set();
			Evt_queue_add(DEVICE_P_OIP_FOCUS_CAN_LIN_EVENT);								
		}

		if((Hu_Fr_Dr_Vent_Mode_invalid == AVOID_MODE) || (Hu_Fr_Ps_Vent_Mode_invalid == AVOID_MODE))
		{
			Can_Avoid_mode_set();
			Evt_queue_add(DEVICE_P_OIP_SPREAD_CAN_LIN_EVENT); 							
		} 						

		if((Hu_Fr_Dr_Vent_Mode_invalid == SWING_MODE) || (Hu_Fr_Ps_Vent_Mode_invalid == SWING_MODE))
		{
			Can_Cycle_mode_set();
			Evt_queue_add(DEVICE_P_OIP_CYCLE_CAN_LIN_EVENT);								
		} 

		if((Hu_Fr_Dr_Vent_Mode_invalid == FREE_MODE) || (Hu_Fr_Ps_Vent_Mode_invalid == FREE_MODE))
		{		
			Can_Manual_mode_set();
			Evt_queue_add(DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT);

			//if(P_Oip_Cts_Select == P_OIP_SELECT)
			///{
			Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x01; //APPLY Duplucate Input Signal Prevention Control Status return
			//}
			//else
			//{
			Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x01; //NOT APPLY
			//}
		} 
	}
}

/*
* @brief  flexcan0_Callback function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void flexcan0_Callback(uint8_t instance, flexcan_event_type_t eventType,
					   uint32_t buffIdx, flexcan_state_t *flexcanState)
{
	(void)flexcanState;
	(void)instance;
	(void)buffIdx;
#ifdef AMO_NVM_SETTING	
	//nvmem_data_type nv;
#endif

	//Start flexcan0_Callback value choice
	switch(eventType)
	{
		case FLEXCAN_EVENT_RX_COMPLETE:
		{
			/* rx status return*/
			rxMBdone = 1;
		}
		break;
		case FLEXCAN_EVENT_RXFIFO_COMPLETE:
		break;
		case FLEXCAN_EVENT_DMA_COMPLETE:
		{
			rxFIFOdone = 1;
			Evnt_Network_Release = 1;
			//printf("Flex can recvBuff2 ID = 0x%04x recvBuff2.dataLen = %d \r\n", recvBuff2.msgId, recvBuff2.dataLen);
			Check_rx_start();

			if(recvBuff2.msgId == 0x505)
			{
				if(((recvBuff2.data[0] == 0x79u) && (recvBuff2.data[1] == 0x32u)) && (recvBuff2.data[2] == 0x84u))
				{
					Amo_timer_Stop(timer_7);
					Amo_timer_Stop(timer_0);
					Amo_timer_Stop(timer_1);
					Amo_timer_Stop(timer_2);
					Amo_timer_Stop(timer_3);
					Amo_timer_Stop(timer_4);
					Amo_timer_Stop(timer_5);
					Amo_timer_Stop(timer_6);
					Amo_timer_Stop(timer_8);
					Amo_timer_Stop(timer_9);
					Amo_timer_Stop(timer_10);
					Amo_timer_Stop(timer_11);
					Amo_timer_Stop(timer_12);
					Amo_timer_Stop(timer_13);				
					AIR_VENT_DL_MODE = 1;
				}
			}
			
			if(AIR_VENT_DL_MODE == 0)
			{
				switch(recvBuff2.msgId)
				{
				case 0x22:
				{
					Rx_id_22();
					break;
				}
				case 0x23:		//JG1 ONLY
				{
					memset(Can_data_3_Info.reg_data, 0x00, sizeof(Can_data_3_Info.reg_data));
					memcpy(&Can_data_3_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_3_expire = 0;

					/* ATCU_RheosatLevelStatue status return*/
					ATCU_RheosatLevelStatue = Can_data_3_Info.data.L_ATCU_RhstaLvlSta;

					Rx_info_data[1].start_info = 0x01;
					Rx_info_data[1].rx_off = 0x00;
					Amo_timer_Stop(timer_6);
					Amo_timer_Start(timer_6, 2000, false, Can_rx_3_timer);
					break;
				}				
				case 0x24:
				{			
					Rx_id_24();
					break;
				}
				case 0x25:
				{
					#ifdef AMO_GN7_PE_SETTING_NONE
					memset(Can_data_5_Info.reg_data, 0x00, sizeof(Can_data_5_Info.reg_data));
					memcpy(&Can_data_5_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_5_expire = 0;

					ATCU_RearOperationStatus = Can_data_5_Info.data.L_ATCU_RrOpSta;
					ATCU_Rr_Dr_Mode_display = Can_data_5_Info.data.L_ATCU_RrDrModDis;
					ATCU_Rr_Ps_Mode_display = Can_data_5_Info.data.L_ATCU_RrPsModDis;	

#ifdef AMO_GN7_PE_SETTING_NONE
					if((Evnt_IGN2_Onoff == 1) || (Evnt_IGN3_Onoff == 1))	//IGN2 or IGN3 ON
#else
					if(Evnt_IGN2_Onoff == 1)	//IGN2
#endif
					{
						if((ATCU_Rr_Dr_Mode_display == VENT) || (ATCU_Rr_Dr_Mode_display == B_L) || (ATCU_Rr_Dr_Mode_display == MODE7_DEF_VENT) || (ATCU_Rr_Dr_Mode_display == MODE7_DEF_VENT_FLOOR) || (ATCU_Rr_Dr_Mode_display == DEF_VENT_ARROW) || (ATCU_Rr_Dr_Mode_display == DEF_VENT_FLOOR_ARROW))
						{
							Vent_Mode_RrDrOnOff = 1;
						}
						else
						{
							Vent_Mode_RrDrOnOff = 0;
						}

						if((ATCU_Rr_Ps_Mode_display == VENT) || (ATCU_Rr_Ps_Mode_display == B_L) || (ATCU_Rr_Ps_Mode_display == MODE7_DEF_VENT) || (ATCU_Rr_Ps_Mode_display == MODE7_DEF_VENT_FLOOR) || (ATCU_Rr_Ps_Mode_display == DEF_VENT_ARROW) || (ATCU_Rr_Ps_Mode_display == DEF_VENT_FLOOR_ARROW))
						{
							Vent_Mode_RrPassOnOff = 1;
						}
						else
						{
							Vent_Mode_RrPassOnOff = 0;
						}
					}
					else
					{
						Vent_Mode_RrDrOnOff=0;
						Vent_Mode_RrDrOnOff=0;
					}

					Rx_info_data[3].start_info = 0x01;
					Rx_info_data[3].rx_off = 0x00;
					Amo_timer_Stop(timer_2);
					Amo_timer_Start(timer_2, 2000, false, Can_rx_5_timer);	
					#endif
					break;
				}
				case 0x28:  //utility mode -> indoor light control
				{
					Rx_id_28();
					break;
				}
				case 0x29:		//utility mode -> indoor light control
				{
					Rx_id_29();
					break;
				}
				case 0x111:  //Driver service type
				{
					memset(Can_data_12_Info.reg_data, 0x00, sizeof(Can_data_12_Info.reg_data));
					memcpy(&Can_data_12_Info.reg_data[0], &recvBuff2.data[0], 8);
					/* Driver type setting */
					can_rx_12_expire = 0;
					ATCU_DriverSideType = (Drv_type)Can_data_12_Info.data.L_ATCU_DrvSdTyp;   //LHD = 0, RHD = 0; 
					ATCU_FrBlwLvl = Can_data_12_Info.data.L_ATCU_FrBlwLvl;
					ATCU_PassBlwLvl = Can_data_12_Info.data.L_ATCU_PassBlwLvl;
					Rx_info_data[6].start_info = 0x01;
					Rx_info_data[6].rx_off = 0x00;
					Amo_timer_Stop(timer_5);
					Amo_timer_Start(timer_5, 2000, false, Can_rx_12_timer);				
					break;
				}
				#if 0
				case 0x112:
				{
					Amo_timer_Stop(timer_6);
					Amo_timer_Start(timer_6, 2000, false, Can_rx_13_timer);						
					break;
				}
				case 0x115:
				{
					Amo_timer_Stop(timer_7);
					Amo_timer_Start(timer_7, 2000, false, Can_rx_16_timer);					
					break;
				}
				#endif
				case 0x119: //P-OIP Receive -> Front drive vent user move request value 
				{
					Rx_id_119();
					break;
				}
				case 0x131: //P-OIP Receive -> Front passenger vent user move request value 
				{
					Rx_id_131();
					break;
				}
				case 0x132:  //P-OIP Receive -> Rear drive/ Rear passenger vent mode define
				{
					#ifdef AMO_GN7_PE_SETTING_NONE
					memset(Can_data_22_Info.reg_data, 0x00, sizeof(Can_data_22_Info.reg_data));
					memcpy(&Can_data_22_Info.reg_data[0], &recvBuff2.data[0], 8);
					
					if(P_Oip_Cts_priority == 0)
					{
						P_Oip_Cts_Select = P_OIP_SELECT;
						P_Oip_Cts_priority = 1;
					}

					//P-OIP Receive -> Rear drive/ Rear passenger vent mode define
					Hu_Rr_Dr_Vent_Mode = Can_data_22_Info.data.L_ATCU_HU_RrDrEVntModeSet;
					Hu_Rr_Ps_Vent_Mode = Can_data_22_Info.data.L_ATCU_HU_RrDrEVntModeSet;
					Hu_Rr_Dr_Vent_signal = Can_data_22_Info.data.L_ATCU_HU_RrDrEVntConsOpnCls;
					Hu_Rr_Ps_Vent_signal = Can_data_22_Info.data.L_ATCU_HU_RrPsEVNTConsOpnCls;

					//P-OIP Receive -> Rear drive vent user move request value 
					Hu_RrDrEVntConsPt_X = Can_data_22_Info.data.L_ATCU_HU_RrDrEVntConsPt_X;   
					Hu_RrDrEVntConsPt_Y = Can_data_22_Info.data.L_ATCU_HU_RrDrEVntConsPt_Y;
					
					//P-OIP Receive -> Rear Passenger vent user move request value 
					Hu_RrPsEVntConsPt_X = Can_data_22_Info.data.L_ATCU_HU_RrPsEVntConsPt_X;
					Hu_RrPsEVntConsPt_Y = Can_data_22_Info.data.L_ATCU_HU_RrPsEVntConsPt_Y;

					if((Hu_Rr_Dr_Vent_Mode == FREE_MODE) || (Hu_Rr_Ps_Vent_Mode == FREE_MODE))
					{						
						if(P_Oip_Cts_Select == P_OIP_SELECT)
						{
							Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x01; //APPLY Duplucate Input Signal Prevention Control Status return
						}
						else
						{
							Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x02; //NOT APPLY
						}										
					}						
					
					if((Evnt_IGN2_Onoff == 1) || (Evnt_IGN3_Onoff == 1))	//IGN2 or IGN3 ON
					{
						if((Vent_Mode_RrDrOnOff) || (Vent_Mode_RrPassOnOff))
						{
							if(P_Oip_Cts_Select == P_OIP_SELECT)
							{
								if((Hu_Rr_Dr_Vent_signal == 0x02) || (Hu_Rr_Ps_Vent_signal == 0x02))
								{
									if(Hu_Rr_Dr_Vent_signal == 0x02)
									{
										Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0x00; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x02;
										//Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x02;
									}

									if(Hu_Rr_Ps_Vent_signal == 0x02)
									{
										Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x00; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										//Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x02;
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
										//Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x01;
										Hu_Rr_Dr_Vent_Mode = Hu_Rr_Dr_Vent_Mode_Remember;
										//Hu_Rr_Dr_Vent_signal = 0x00;
									}
								}

								if(Hu_Rr_Ps_Vent_signal == 0x01)
								{
									if(Hu_Rr_Ps_Vent_Mode == 0x00)
									{
										Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = Cts_Rr_Ps_Vent_Mode_remember; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										//Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x01;
										Hu_Rr_Ps_Vent_Mode = Hu_Rr_Ps_Vent_Mode_Remember;
										//Hu_Rr_Dr_Vent_signal = 0x00;
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
										button_LH_cycle_check = 0x04;
									}

									if(Hu_Rr_Ps_Vent_Mode == FREE_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x04;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
										Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
										Hu_Rr_Ps_Vent_Mode_Remember = 0x04;		
										button_RH_cycle_check = 0x04;
									}
									
									if(P_Oip_Cts_Select == P_OIP_SELECT)
									{
										Touch_mode_Rr_set = HU_REAR_TOUCH;
										Evt_queue_add(DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT);
										Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x01; //APPLY Duplucate Input Signal Prevention Control Status return
									}
									else
									{
										Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x02; //NOT APPLY
									}										
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
						}
						else
						{
							P_Oip_Cts_priority = 0;
						}
					}
					else
					{
						P_Oip_Cts_priority = 0;
					}
					
				#endif	
					break;
				}	
				case 0x122:  //P-OIP Receive -> Front drive vent boundary define
				{
					memset(Can_data_23_Info.reg_data, 0x00, sizeof(Can_data_23_Info.reg_data));
					memcpy(&Can_data_23_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_23_expire = 0;
					
					Hu_Fr_Dr_Vent_Side_Bdry_Up_X = (uint16_t)Can_data_23_Info.data.L_ATCU_HU_FrDrEVntSideBdryUp_X*10;
					Hu_Fr_Dr_Vent_Side_Bdry_Lo_X = (uint16_t)Can_data_23_Info.data.L_ATCU_HU_FrDrEVntSideBdryLo_X*10;
					Hu_Fr_Dr_Vent_Side_Bdry_Up_Y = (uint16_t)Can_data_23_Info.data.L_ATCU_HU_FrDrEVntSideBdryUp_Y*10;
					Hu_Fr_Dr_Vent_Side_Bdry_Lo_Y = (uint16_t)Can_data_23_Info.data.L_ATCU_HU_FrDrEVntSideBdryLo_Y*10;
					Hu_Fr_Dr_Vent_Ctr_Bdry_Up_X = (uint16_t)Can_data_23_Info.data.L_ATCU_HU_FrDrEVntCtrBdryUP_X*10;
					Hu_Fr_Dr_Vent_Ctr_Bdry_Lo_X = (uint16_t)Can_data_23_Info.data.L_ATCU_HU_FrDrEVntCtrBdryLo_X*10;
					Hu_Fr_Dr_Vent_Ctr_Bdry_Up_Y = (uint16_t)Can_data_23_Info.data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y*10;
					Hu_Fr_Dr_Vent_Ctr_Bdry_Lo_Y = (uint16_t)Can_data_23_Info.data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y*10;

					Rx_info_data[7].start_info = 0x01;
					Rx_info_data[7].rx_off = 0x00;
					Amo_timer_Stop(timer_8);
					Amo_timer_Start(timer_8, 5000, false, Can_rx_23_timer);
					break;
				}
				case 0x123: //P-OIP Receive ->  Front passenger vent boundary define
				{
					memset(Can_data_24_Info.reg_data, 0x00, sizeof(Can_data_24_Info.reg_data));
					memcpy(&Can_data_24_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_24_expire = 0;
					Hu_Fr_Ps_Vent_Side_Bdry_Up_X = (uint16_t)Can_data_24_Info.data.L_ATCU_HU_FrPsEVntSideBdryUp_X*10;
					Hu_Fr_Ps_Vent_Side_Bdry_Lo_X = (uint16_t)Can_data_24_Info.data.L_ATCU_HU_FrPsEVntSideBdryLo_X*10;
					Hu_Fr_Ps_Vent_Side_Bdry_Up_Y = (uint16_t)Can_data_24_Info.data.L_ATCU_HU_FrPsEVntSideBdryUp_Y*10;
					Hu_Fr_Ps_Vent_Side_Bdry_Lo_Y = (uint16_t)Can_data_24_Info.data.L_ATCU_HU_FrPsEVntSideBdryLo_Y*10;
					Hu_Fr_Ps_Vent_Ctr_Bdry_Up_X = (uint16_t)Can_data_24_Info.data.L_ATCU_HU_FrPsEVntCtrBdryUP_X*10;
					Hu_Fr_Ps_Vent_Ctr_Bdry_Lo_X = (uint16_t)Can_data_24_Info.data.L_ATCU_HU_FrPsEVntCtrBdryLo_X*10;
					Hu_Fr_Ps_Vent_Ctr_Bdry_Up_Y = (uint16_t)Can_data_24_Info.data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y*10;
					Hu_Fr_Ps_Vent_Ctr_Bdry_Lo_Y = (uint16_t)Can_data_24_Info.data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y*10;

					Rx_info_data[8].start_info = 0x01;
					Rx_info_data[8].rx_off = 0x00;
					Amo_timer_Stop(timer_9);
					Amo_timer_Start(timer_9, 5000, false, Can_rx_24_timer);					
					break;
				}
				case 0x124: //P-OIP Receive -> Rear drive/Rear Passenger vent boundary define
				{
					#ifdef AMO_GN7_PE_SETTING_NONE
					memset(Can_data_25_Info.reg_data, 0x00, sizeof(Can_data_25_Info.reg_data));
					memcpy(&Can_data_25_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_25_expire = 0;

					Hu_Rr_Dr_Vent_Cons_Bdry_Up_X = (uint16_t)Can_data_25_Info.data.L_ATCU_HU_RrDrEVntConsBdryUp_X*10;
					Hu_Rr_Dr_Vent_Cons_Bdry_Lo_X = (uint16_t)Can_data_25_Info.data.L_ATCU_HU_RrDrEVntConsBdryLo_X*10;
					Hu_Rr_Dr_Vent_Cons_Bdry_Up_Y = (uint16_t)Can_data_25_Info.data.L_ATCU_HU_RrDrEVntConsBdryUp_Y*10;
					Hu_Rr_Dr_Vent_Cons_Bdry_Lo_Y = (uint16_t)Can_data_25_Info.data.L_ATCU_HU_RrDrEVntConsBdryLo_Y*10;
					Hu_Rr_Ps_Vent_Cons_Bdry_Up_X = (uint16_t)Can_data_25_Info.data.L_ATCU_HU_RrPsEVntConsBdryUP_X*10;
					Hu_Rr_Ps_Vent_Cons_Bdry_Lo_X = (uint16_t)Can_data_25_Info.data.L_ATCU_HU_RrPsEVntConsBdryLo_X*10;
					Hu_Rr_Ps_Vent_Cons_Bdry_Up_Y = (uint16_t)Can_data_25_Info.data.L_ATCU_HU_RrPsEVntConsBdryUP_Y*10;
					Hu_Rr_Ps_Vent_Cons_Bdry_Lo_Y = (uint16_t)Can_data_25_Info.data.L_ATCU_HU_RrPsEVntConsBdryLo_Y*10;

					Amo_timer_Stop(timer_10);
					Amo_timer_Start(timer_10, 5000, false, Can_rx_25_timer);
					#endif
					break;
				}
				case 0x125: //CTS Receive -> Front drive vent user move request value 
				{
					#ifdef AMO_GN7_PE_SETTING_NONE
					memset(Can_data_26_Info.reg_data, 0x00, sizeof(Can_data_26_Info.reg_data));
					memcpy(&Can_data_26_Info.reg_data[0], &recvBuff2.data[0], 8);
					
					Cts_FrDrEVntSidePt_X = Can_data_26_Info.data.L_ATCU_CTS_FrDrEVntSidePt_X;
					Cts_FrDrEVntSidePt_Y = Can_data_26_Info.data.L_ATCU_CTS_FrDrEVntSidePt_Y;
					Cts_FrDrEVntCtrPt_X = Can_data_26_Info.data.L_ATCU_CTS_FrDrEVntCtrPt_X;
					Cts_FrDrEVntCtrPt_Y = Can_data_26_Info.data.L_ATCU_CTS_FrDrEVntCtrPt_Y;

					if((Evnt_IGN2_Onoff == 1) || (Evnt_IGN3_Onoff == 1))	//IGN2 or IGN3 ON
					{
						if(Vent_Mode_FrDrOnOff == 1)
						{
							if(Cts_Fr_Dr_Vent_Mode == FREE_MODE)
							{
								if(P_Oip_Cts_Select == CTS_SELECT)
								{
									Touch_mode_Dr_set = CTS_DRIVER_TOUCH;
									Evt_queue_add(DEVICE_CTS_MANUAL_CAN_LIN_EVENT);
								}
							}
						}
					}		
					#endif
					break;
				}
				case 0x126: //CTS Receive -> Front passenger vent user move request value 
				{
					#ifdef AMO_GN7_PE_SETTING_NONE
					memset(Can_data_27_Info.reg_data, 0x00, sizeof(Can_data_27_Info.reg_data));
					memcpy(&Can_data_27_Info.reg_data[0], &recvBuff2.data[0], 8);
				
					Cts_FrPsEVntSidePt_X = Can_data_27_Info.data.L_ATCU_CTS_FrPsEVntSidePt_X;
					Cts_FrPsEVntSidePt_Y = Can_data_27_Info.data.L_ATCU_CTS_FrPsEVntSidePt_Y;
					Cts_FrPsEVntCtrPt_X = Can_data_27_Info.data.L_ATCU_CTS_FrPsEVntCtrPt_X;
					Cts_FrPsEVntCtrPt_Y = Can_data_27_Info.data.L_ATCU_CTS_FrPsEVntCtrPt_Y;

					if((Evnt_IGN2_Onoff == 1) || (Evnt_IGN3_Onoff == 1))	//IGN2 or IGN3 ON
					{
						if(Vent_Mode_FrPassOnOff == 1)
						{
							if(Cts_Fr_Ps_Vent_Mode == FREE_MODE)
							{
								if(P_Oip_Cts_Select == CTS_SELECT)
								{
									Touch_mode_Ps_set = CTS_PASSENGER_TOUCH;
									Evt_queue_add(DEVICE_CTS_MANUAL_CAN_LIN_EVENT);
								}
							}
						}
					}
					#endif
					break;
				}
				case 0x127: //CTS Receive -> Rear drive/ Rear passenger vent mode define
				{
					#ifdef AMO_GN7_PE_SETTING_NONE
					memset(Can_data_28_Info.reg_data, 0x00, sizeof(Can_data_28_Info.reg_data));
					memcpy(&Can_data_28_Info.reg_data[0], &recvBuff2.data[0], 8);

						if(P_Oip_Cts_priority == 0)
					{
						P_Oip_Cts_Select = CTS_SELECT;
						P_Oip_Cts_priority = 1;
					}

					Cts_Rr_Dr_Vent_Mode = Can_data_28_Info.data.L_ATCU_CTS_RrDrEVntModeSet;
					Cts_Rr_Ps_Vent_Mode = Can_data_28_Info.data.L_ATCU_CTS_RrPsEVntModeSet;
					Cts_Rr_Dr_Vent_siganl = Can_data_28_Info.data.L_ATCU_CTS_RrDrEVntConsOpnCls;
					Cts_Rr_Ps_Vent_siganl = Can_data_28_Info.data.L_ATCU_CTS_RrPsEVntConsOpnCls;

					//CTS Receive -> Rear driver vent user move request value 
					Cts_RrDrEVntConsPt_X = Can_data_28_Info.data.L_ATCU_CTS_RrDrEVntConsPt_X;
					Cts_RrDrEVntConsPt_Y = Can_data_28_Info.data.L_ATCU_CTS_RrDrEVntConsPt_Y;
					
					//CTS Receive -> Rear passenger vent user move request value 
					Cts_RrPsEVntConsPt_X = Can_data_28_Info.data.L_ATCU_CTS_RrPsEVntConsPt_X;
					Cts_RrPsEVntConsPt_Y = Can_data_28_Info.data.L_ATCU_CTS_RrPsEVntConsPt_Y;

					if((Cts_Rr_Dr_Vent_Mode == FREE_MODE) || (Cts_Rr_Ps_Vent_Mode == FREE_MODE))
					{					
						if(P_Oip_Cts_Select == CTS_SELECT)
						{
							Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_CTS = 0x01; //APPLY Duplucate Input Signal Prevention Control Status return
						}
						else
						{
							Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_CTS = 0x02; //NOT APPLY
						} 									
						
					}
					
					if((Evnt_IGN2_Onoff == 1) || (Evnt_IGN3_Onoff == 1))	//IGN2 or IGN3 ON
					{				
						if((Vent_Mode_RrDrOnOff) || (Vent_Mode_RrPassOnOff))
						{							
							if((Cts_Rr_Dr_Vent_siganl == 0x02) || (Cts_Rr_Ps_Vent_siganl == 0x02))
							{
								if(Cts_Rr_Dr_Vent_siganl == 0x02)
								{
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0x00; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x02;
									//Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x02;
								}

								if(Cts_Rr_Ps_Vent_siganl == 0x02)
								{
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x00; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									//Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x02;
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x02;							
								}
								Evt_queue_add(DEVICE_CTS_FULL_CLOSE_CAN_LIN_EVENT);
								
							}
												
							if(Cts_Rr_Dr_Vent_siganl == 0x01)
							{
								if(Cts_Rr_Dr_Vent_Mode == 0x00)
								{
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = Cts_Rr_Dr_Vent_Mode_remember; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x01;
									//Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x01;
									Cts_Rr_Dr_Vent_Mode = Cts_Rr_Dr_Vent_Mode_remember;
									Cts_Rr_Dr_Vent_siganl = 0x00;
								}
							}

							if(Cts_Rr_Ps_Vent_siganl == 0x01)
							{
								if(Cts_Rr_Ps_Vent_Mode == 0x00)
								{
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = Cts_Rr_Ps_Vent_Mode_remember; 													//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									//Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x01;
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x01;
									Cts_Rr_Ps_Vent_Mode = Cts_Rr_Ps_Vent_Mode_remember;
									Cts_Rr_Ps_Vent_siganl = 0x00;
								}
							}	
								
							if((Cts_Rr_Dr_Vent_Mode == FACING_MODE) || (Cts_Rr_Ps_Vent_Mode == FACING_MODE))
							{
								if(Cts_Rr_Dr_Vent_Mode == FACING_MODE)
								{
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0x01;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
									Cts_Rr_Dr_Vent_Mode_remember = 0x01;
								}

								if(Cts_Rr_Ps_Vent_Mode == FACING_MODE)
								{
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x01;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
									Cts_Rr_Ps_Vent_Mode_remember = 0x01;									
								}
								Evt_queue_add(DEVICE_CTS_FOCUS_CAN_LIN_EVENT);
							}

							if((Cts_Rr_Dr_Vent_Mode == AVOID_MODE) || (Cts_Rr_Ps_Vent_Mode == AVOID_MODE))
							{
								if(Cts_Rr_Dr_Vent_Mode == AVOID_MODE)
								{
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0x02;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
									Cts_Rr_Dr_Vent_Mode_remember = 0x02;
								}

								if(Cts_Rr_Ps_Vent_Mode == AVOID_MODE)
								{
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x02;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
									Cts_Rr_Ps_Vent_Mode_remember = 0x02;									
								}
								Evt_queue_add(DEVICE_CTS_SPREAD_CAN_LIN_EVENT);
							}

							if((Cts_Rr_Dr_Vent_Mode == SWING_MODE) || (Cts_Rr_Ps_Vent_Mode == SWING_MODE))
							{
								if(Cts_Rr_Dr_Vent_Mode == SWING_MODE)
								{
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0x03;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
									Cts_Rr_Dr_Vent_Mode_remember = 0x03;
								}

								if(Cts_Rr_Ps_Vent_Mode == SWING_MODE)
								{
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x03;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
									Cts_Rr_Ps_Vent_Mode_remember = 0x03;									
								}
								Evt_queue_add(DEVICE_CTS_CYCLE_CAN_LIN_EVENT);
							}

							if((Cts_Rr_Dr_Vent_Mode == FREE_MODE) || (Cts_Rr_Ps_Vent_Mode == FREE_MODE))
							{
								if(Cts_Rr_Dr_Vent_Mode == FREE_MODE)
								{
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0x04;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
									Cts_Rr_Dr_Vent_Mode_remember = 0x04;
								}

								if(Cts_Rr_Ps_Vent_Mode == FREE_MODE)
								{
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x04;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
									Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
									Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
									Cts_Rr_Ps_Vent_Mode_remember = 0x04;									
								}
								
								if(P_Oip_Cts_Select == CTS_SELECT)
								{
									Touch_mode_Rr_set = CTS_REAR_TOUCH;
									Evt_queue_add(DEVICE_CTS_MANUAL_CAN_LIN_EVENT);
									Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_CTS = 0x01; //APPLY Duplucate Input Signal Prevention Control Status return
								}
								else
								{
									Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_CTS = 0x02; //NOT APPLY
								}										
								
							}

							if(((Cts_Rr_Dr_Vent_Mode_remember == 0x00) && (Cts_Rr_Dr_Vent_siganl != 0x02)) || ((Cts_Rr_Ps_Vent_Mode_remember == 0x00) && (Cts_Rr_Ps_Vent_siganl != 0x02)))
							{
								P_Oip_Cts_priority = 0;
							}			
							
							if(Hu_Fr_Dr_Vent_Mode == MODE_OFF)
							{
								P_Oip_Cts_priority = 0;
								Cts_Rr_Dr_Vent_Mode_remember = MODE_OFF;
								//VENT MODE OFF
							}
							
							if(Hu_Fr_Ps_Vent_Mode == MODE_OFF)
							{
								P_Oip_Cts_priority = 0;
								Cts_Rr_Ps_Vent_Mode_remember = MODE_OFF;
								//VENT MODE OFF
							}
						}
						else
						{
							P_Oip_Cts_priority = 0;
						}
					}
					else
					{
						P_Oip_Cts_priority = 0;
					}
					#endif
					break;
				}
				case 0x128: //CTS Receive -> Front drive vent boundary define
				{
					#ifdef AMO_GN7_PE_SETTING_NONE
					memset(Can_data_29_Info.reg_data, 0x00, sizeof(Can_data_29_Info.reg_data));
					memcpy(&Can_data_29_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_29_expire = 0;

					Cts_Fr_Dr_Vent_Side_Bdry_Up_X = Can_data_29_Info.data.L_ATCU_CTS_FrDrEVntSideBdryUp_X*10;
					Cts_Fr_Dr_Vent_Side_Bdry_Lo_X = Can_data_29_Info.data.L_ATCU_CTS_FrDrEVntSideBdryLo_X*10;
					Cts_Fr_Dr_Vent_Side_Bdry_Up_Y = Can_data_29_Info.data.L_ATCU_CTS_FrDrEVntSideBdryUp_Y*10;
					Cts_Fr_Dr_Vent_Side_Bdry_Lo_Y = Can_data_29_Info.data.L_ATCU_CTS_FrDrEVntSideBdryLo_Y*10;
					Cts_Fr_Dr_Vent_Ctr_Bdry_Up_X = Can_data_29_Info.data.L_ATCU_CTS_FrDrEVntCtrBdryUp_X*10;
					Cts_Fr_Dr_Vent_Ctr_Bdry_Lo_X = Can_data_29_Info.data.L_ATCU_CTS_FrDrEVntCtrBdryLo_X*10;
					Cts_Fr_Dr_Vent_Ctr_Bdry_Up_Y = Can_data_29_Info.data.L_ATCU_CTS_FrDrEVntCtrBdryUp_Y*10;
					Cts_Fr_Dr_Vent_Ctr_Bdry_Lo_Y = Can_data_29_Info.data.L_ATCU_CTS_FrDrEVntCtrBdryLo_Y*10;

					Amo_timer_Stop(timer_11);
					Amo_timer_Start(timer_11, 5000, false, Can_rx_29_timer);	
					#endif
					break;
				}
				case 0x129: //CTS Receive -> Front Passenger vent boundary define
				{
					#ifdef AMO_GN7_PE_SETTING_NONE
					memset(Can_data_30_Info.reg_data, 0x00, sizeof(Can_data_30_Info.reg_data));
					memcpy(&Can_data_30_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_30_expire = 0;

					Cts_Fr_Ps_Vent_Side_Bdry_Up_X = Can_data_30_Info.data.L_ATCU_CTS_FrPsEVntSideBdryUp_X*10;
					Cts_Fr_Ps_Vent_Side_Bdry_Lo_X = Can_data_30_Info.data.L_ATCU_CTS_FrPsEVntSideBdryLo_X*10;
					Cts_Fr_Ps_Vent_Side_Bdry_Up_Y = Can_data_30_Info.data.L_ATCU_CTS_FrPsEVntSideBdryUp_Y*10;
					Cts_Fr_Ps_Vent_Side_Bdry_Lo_Y = Can_data_30_Info.data.L_ATCU_CTS_FrPsEVntSideBdryLo_Y*10;
					Cts_Fr_Ps_Vent_Ctr_Bdry_Up_X = Can_data_30_Info.data.L_ATCU_CTS_FrPsEVntCtrBdryUp_X*10;
					Cts_Fr_Ps_Vent_Ctr_Bdry_Lo_X = Can_data_30_Info.data.L_ATCU_CTS_FrPsEVntCtrBdryLo_X*10;
					Cts_Fr_Ps_Vent_Ctr_Bdry_Up_Y = Can_data_30_Info.data.L_ATCU_CTS_FrPsEVntCtrBdryUp_Y*10;
					Cts_Fr_Ps_Vent_Ctr_Bdry_Lo_Y = Can_data_30_Info.data.L_ATCU_CTS_FrPsEVntCtrBdryLo_Y*10;

					Amo_timer_Stop(timer_12);
					Amo_timer_Start(timer_12, 5000, false, Can_rx_30_timer);			
					#endif
					break;
				}
				case 0x130: //CTS Receive -> Rear driver/Rear Passenger vent boundary define
				{
					#ifdef AMO_GN7_PE_SETTING_NONE
					memset(Can_data_31_Info.reg_data, 0x00, sizeof(Can_data_31_Info.reg_data));
					memcpy(&Can_data_31_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_31_expire = 0;
					
					Cts_Rr_Dr_Vent_Cons_Bdry_Up_X = Can_data_31_Info.data.L_ATCU_CTS_RrDrEVntConsBdryUp_X*10;
					Cts_Rr_Dr_Vent_Cons_Bdry_Lo_X = Can_data_31_Info.data.L_ATCU_CTS_RrDrEVntConsBdryLo_X*10;
					Cts_Rr_Dr_Vent_Cons_Bdry_Up_Y = Can_data_31_Info.data.L_ATCU_CTS_RrDrEVntConsBdryUp_Y*10;
					Cts_Rr_Dr_Vent_Cons_Bdry_Lo_Y = Can_data_31_Info.data.L_ATCU_CTS_RrDrEVntConsBdryLo_Y*10;
					Cts_Rr_Ps_Vent_Cons_Bdry_Up_X = Can_data_31_Info.data.L_ATCU_CTS_RrPsEVntConsBdryUp_X*10;
					Cts_Rr_Ps_Vent_Cons_Bdry_Lo_X = Can_data_31_Info.data.L_ATCU_CTS_RrPsEVntConsBdryLo_X*10;
					Cts_Rr_Ps_Vent_Cons_Bdry_Up_Y = Can_data_31_Info.data.L_ATCU_CTS_RrPsEVntConsBdryUp_Y*10;
					Cts_Rr_Ps_Vent_Cons_Bdry_Lo_Y = Can_data_31_Info.data.L_ATCU_CTS_RrPsEVntConsBdryLo_Y*10;

					Amo_timer_Stop(timer_13);
					Amo_timer_Start(timer_13, 5000, false, Can_rx_31_timer);		
					#endif
					break;
				}				
				case 0x161:		//P-OIP Receive -> Front drive/ Front passenger vent mode define
				{
					/*start 0x161 value */
					Rx_id_161();
					break;
				}
				case 0x162: //CTS Receive -> Front drive/ Front passenger vent mode define
				{
					#ifdef AMO_GN7_PE_SETTING_NONE
					memset(Can_data_33_Info.reg_data, 0x00, sizeof(Can_data_33_Info.reg_data));
					memcpy(&Can_data_33_Info.reg_data[0], &recvBuff2.data[0], 8);

						if(P_Oip_Cts_priority == 0)
					{
						P_Oip_Cts_Select = CTS_SELECT;
						P_Oip_Cts_priority = 1;
					}

					Cts_Fr_Dr_Vent_Mode = Can_data_33_Info.data.L_ATCU_CTS_FrDrEVntModeSet;
					Cts_Fr_Ps_Vent_Mode = Can_data_33_Info.data.L_ATCU_CTS_FrPsEVntModeSet;
					Cts_Fr_dr_Vent_Center_signal = Can_data_33_Info.data.L_ATCU_CTS_FrDrEVntCtrOpnCls;
					Cts_Fr_dr_Vent_Side_siganl = Can_data_33_Info.data.L_ATCU_CTS_FrDrEVntSideOpenCls;
					Cts_Fr_Ps_Vent_Center_signal = Can_data_33_Info.data.L_ATCU_CTS_FrPsEVntCtrOpnCls;
					Cts_Fr_Ps_Vent_Side_signal = Can_data_33_Info.data.L_ATCU_CTS_FrPsEVntSideOpnCls;

					if((Cts_Fr_Dr_Vent_Mode == FREE_MODE) || (Cts_Fr_Ps_Vent_Mode == FREE_MODE))
					{						
						if(P_Oip_Cts_Select == CTS_SELECT)
						{
							Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_CTS = 0x01; //APPLY Duplucate Input Signal Prevention Control Status return
						}
						else
						{
							Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_CTS = 0x02; //NOT APPLY
						} 							
					}
					/*Evnt_IGN2_Onoff on/off check */
					if((Evnt_IGN2_Onoff == 1) || (Evnt_IGN3_Onoff == 1))	//IGN2 or IGN3 ON
					{
						if((Vent_Mode_FrDrOnOff == 1) || (Vent_Mode_FrPassOnOff == 1))
						{
							if(P_Oip_Cts_Select == CTS_SELECT)
							{
								if((Cts_Fr_dr_Vent_Side_siganl == 0x02) || (Cts_Fr_dr_Vent_Center_signal == 0x02) || (Cts_Fr_Ps_Vent_Side_signal == 0x02) || (Cts_Fr_Ps_Vent_Center_signal == 0x02))
								{
									if(Cts_Fr_dr_Vent_Side_siganl == 0x02)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x02;
										//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x02;
									}

									if(Cts_Fr_dr_Vent_Center_signal == 0x02)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x02;
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x02;
									}

									if(Cts_Fr_Ps_Vent_Side_signal == 0x02)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x02;
										//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x02;								
									}

									if(Cts_Fr_Ps_Vent_Center_signal == 0x02)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x02;
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x02;								
									}				
									
									Evt_queue_add(DEVICE_CTS_FULL_CLOSE_CAN_LIN_EVENT);
								}
								
								if((Cts_Fr_dr_Vent_Side_siganl == 0x01))
								{
									if(Cts_Fr_Dr_Vent_Mode == 0x00)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Cts_Fr_Dr_Vent_Mode_remember;		//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
										//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Cts_Fr_Dr_Vent_Mode = Cts_Fr_Dr_Vent_Mode_remember;
									}
								}
							
								if(Cts_Fr_dr_Vent_Center_signal == 0x01)
								{
									if(Cts_Fr_Dr_Vent_Mode == 0x00)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Cts_Fr_Dr_Vent_Mode_remember;		//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Cts_Fr_Dr_Vent_Mode = Cts_Fr_Dr_Vent_Mode_remember;
									}
								}
							
								if(Cts_Fr_Ps_Vent_Side_signal == 0x01)
								{
									if(Cts_Fr_Ps_Vent_Mode == 0x00)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Cts_Fr_Ps_Vent_Mode_remember;		//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
										//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
										Cts_Fr_Ps_Vent_Mode = Cts_Fr_Ps_Vent_Mode_remember;
									}
								} 			
							
								if(Cts_Fr_Ps_Vent_Center_signal == 0x01)
								{
									if(Cts_Fr_Ps_Vent_Mode == 0x00)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Cts_Fr_Ps_Vent_Mode_remember;		//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
										Cts_Fr_Ps_Vent_Mode = Cts_Fr_Ps_Vent_Mode_remember;
									}
								} 	

								if((Cts_Fr_Dr_Vent_Mode == FACING_MODE) || (Cts_Fr_Ps_Vent_Mode == FACING_MODE))
								{
									if(Cts_Fr_Dr_Vent_Mode == FACING_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x01;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Cts_Fr_Dr_Vent_Mode_remember = FACING_MODE; 
									}

									if(Cts_Fr_Ps_Vent_Mode == FACING_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x01;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
										Cts_Fr_Ps_Vent_Mode_remember = FACING_MODE; 
									}								
									Evt_queue_add(DEVICE_CTS_FOCUS_CAN_LIN_EVENT);								
								}

								if((Cts_Fr_Dr_Vent_Mode == AVOID_MODE) || (Cts_Fr_Ps_Vent_Mode == AVOID_MODE))
								{
									if(Cts_Fr_Dr_Vent_Mode == AVOID_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x02;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Cts_Fr_Dr_Vent_Mode_remember = AVOID_MODE; 
									}

									if(Cts_Fr_Ps_Vent_Mode == AVOID_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x02;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
										Cts_Fr_Ps_Vent_Mode_remember = AVOID_MODE; 
									}								
									Evt_queue_add(DEVICE_CTS_SPREAD_CAN_LIN_EVENT);								
								}

								if((Cts_Fr_Dr_Vent_Mode == SWING_MODE) || (Cts_Fr_Ps_Vent_Mode == SWING_MODE))
								{
									if(Cts_Fr_Dr_Vent_Mode == SWING_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x03;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Cts_Fr_Dr_Vent_Mode_remember = SWING_MODE; 
									}

									if(Cts_Fr_Ps_Vent_Mode == SWING_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x03;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
										Cts_Fr_Ps_Vent_Mode_remember = SWING_MODE; 
									}								
									Evt_queue_add(DEVICE_CTS_CYCLE_CAN_LIN_EVENT);								
								}

								if((Cts_Fr_Dr_Vent_Mode == FREE_MODE) || (Cts_Fr_Ps_Vent_Mode == FREE_MODE))
								{
									if(Cts_Fr_Dr_Vent_Mode == FREE_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Cts_Fr_Dr_Vent_Mode_remember = FREE_MODE; 
									}

									if(Cts_Fr_Ps_Vent_Mode == FREE_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x04;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
										Cts_Fr_Ps_Vent_Mode_remember = FREE_MODE; 
									}
									
									if(P_Oip_Cts_Select == CTS_SELECT)
									{
										Evt_queue_add(DEVICE_CTS_MANUAL_CAN_LIN_EVENT);
										Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_CTS = 0x01; //APPLY Duplucate Input Signal Prevention Control Status return
									}
									else
									{
										Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_CTS = 0x02; //NOT APPLY
									}								
								}			
								
								if(((Cts_Fr_Dr_Vent_Mode_remember == 0x00) && (Cts_Fr_dr_Vent_Side_siganl != 0x02)) || ((Cts_Fr_Dr_Vent_Mode_remember == 0x00) && (Cts_Fr_dr_Vent_Center_signal != 0x02)))
								{
									P_Oip_Cts_priority = 0;
								}

								if(((Cts_Fr_Ps_Vent_Mode_remember == 0x00) && (Cts_Fr_Ps_Vent_Side_signal != 0x02)) || ((Cts_Fr_Ps_Vent_Mode_remember == 0x00) && (Cts_Fr_Ps_Vent_Center_signal != 0x02)))
								{
									P_Oip_Cts_priority = 0;
								}

								if(Cts_Fr_Dr_Vent_Mode == MODE_OFF)
								{
									//VENT MODE OFF
									Cts_Fr_Dr_Vent_Mode_remember = MODE_OFF; 
									P_Oip_Cts_priority = 0;
								}					
								if(Cts_Fr_Ps_Vent_Mode == MODE_OFF)
								{
									//VENT MODE OFF
									Cts_Fr_Ps_Vent_Mode_remember = MODE_OFF;
									P_Oip_Cts_priority = 0;
								}
							}
						}
						else
						{
							P_Oip_Cts_priority = 0;
						}
					}
					else
					{
						P_Oip_Cts_priority = 0;
					}

					#endif
					break;

					}				
					default:
					/* default code */
					break;
				}
			}
			else
			{		
					switch(recvBuff2.msgId)
					{
						case 0x505:		
						{
							Rx_id_505();
							break;
					}
					default:
					/* default code */
					break;
				}
			}
		}
		break;
		case FLEXCAN_EVENT_TX_COMPLETE:
		break;
		default:
		/* default code */
		break;
	}
}

/*
* @brief  Last_Setting_Save function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/					 
void Last_Setting_Save(Profile_mode last_mode)
{
	nvmem_data_type nv;
	//nvmem_read(&nv);
	read_nv_func(&nv);

	/*start Last_Setting_Save value */
	if(last_mode != DEFAULT_PROFILE)
	{
		nv.nv_item.profile_item[last_mode].items.fr_dr_mode = (uint8_t)Hu_Fr_Dr_Vent_Mode;
		nv.nv_item.profile_item[last_mode].items.fr_ps_mode = (uint8_t)Hu_Fr_Ps_Vent_Mode; 
#ifdef AMO_GN7_PE_SETTING_NONE
		nv.nv_item.profile_item[last_mode].items.rear_dr_mode = (uint8_t)Hu_Rr_Dr_Vent_Mode;
		nv.nv_item.profile_item[last_mode].rear_ps_mode = (uint8_t)Hu_Rr_Ps_Vent_Mode;
#endif
		nv.nv_item.profile_item[last_mode].items.fr_dr_side_opncls = Hu_Frdr_Vent_Side_sig_backup;
		nv.nv_item.profile_item[last_mode].items.fr_dr_ctr_opncls = Hu_Frdr_Vent_Center_sig_backup;
		nv.nv_item.profile_item[last_mode].items.fr_ps_side_opncls = Hu_FrPs_Vent_Side_sig_backup;
		nv.nv_item.profile_item[last_mode].items.fr_ps_ctr_opncls = Hu_FrPs_Vent_Center_sig_backup;
#ifdef AMO_GN7_PE_SETTING_NONE
		nv.nv_item.profile_item[last_mode].items.rr_dr_side_opncls = Hu_Rr_Dr_Vent_signal;	
		nv.nv_item.profile_item[last_mode].items.rr_ps_ctr_opncls = Hu_Rr_Ps_Vent_signal; 
#endif			
		nv.nv_item.profile_item[last_mode].items.fr_dr_side_leftright_target = Fr_Dr_side_leftright_target_backup;
		nv.nv_item.profile_item[last_mode].items.fr_dr_side_updown_target = Fr_Dr_side_updown_target_backup;
		nv.nv_item.profile_item[last_mode].items.fr_dr_ctr_leftright_target = Fr_Dr_ctr_leftright_target_backup;
		nv.nv_item.profile_item[last_mode].items.fr_dr_ctr_updown_target = Fr_Dr_ctr_updown_target_backup;
		nv.nv_item.profile_item[last_mode].items.fr_ps_side_leftright_target = Fr_Ps_side_leftright_target_backup;
		nv.nv_item.profile_item[last_mode].items.fr_ps_side_updown_target = Fr_Ps_side_updown_target_backup;
		nv.nv_item.profile_item[last_mode].items.fr_ps_ctr_leftright_target = Fr_Ps_ctr_leftright_target_backup;
		nv.nv_item.profile_item[last_mode].items.fr_ps_ctr_updown_target = Fr_Ps_ctr_updown_target_backup;
#ifdef AMO_GN7_PE_SETTING_NONE
		nv.nv_item.profile_item[last_mode].items.rr_dr_side_leftright_target = Rr_Dr_side_leftright_target_backup;
		nv.nv_item.profile_item[last_mode].items.rr_dr_side_updown_target = Rr_Dr_side_updown_targe_backup;
		nv.nv_item.profile_item[last_mode].items.rr_ps_ctr_leftright_target = Rr_Ps_ctr_leftright_target_backup;
		nv.nv_item.profile_item[last_mode].items.rr_ps_ctr_updown_target = Rr_Ps_ctr_updown_target_backup;
#endif
		if((Hu_Fr_Dr_Vent_Mode == FACING_MODE) || (Hu_Fr_Ps_Vent_Mode == FACING_MODE) || (Hu_Fr_Dr_Vent_Mode == AVOID_MODE) || (Hu_Fr_Ps_Vent_Mode == AVOID_MODE) || (Hu_Fr_Dr_Vent_Mode == SWING_MODE) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE))
		{
		nv.nv_item.profile_item[last_mode].items.hu_fr_dr_sidept_x = Hu_FrDrEVntSidePt_X_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_dr_sidept_y = Hu_FrDrEVntSidePt_Y_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_dr_ctrpt_x = Hu_FrDrEVntCtrPt_X_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_dr_ctrpt_y = Hu_FrDrEVntCtrPt_Y_backup;

		nv.nv_item.profile_item[last_mode].items.hu_fr_ps_sidept_x = Hu_FrPsEVntSidePt_X_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_ps_sidept_y = Hu_FrPsEVntSidePt_Y_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_ps_ctrpt_x = Hu_FrPsEVntCtrPt_X_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_ps_ctrpt_y = Hu_FrPsEVntCtrPt_Y_backup;
		}
		else if((Hu_Fr_Dr_Vent_Mode == FREE_MODE) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE))
		{
		nv.nv_item.profile_item[last_mode].items.hu_fr_dr_sidept_x = Hu_FrDrEVntSidePt_X_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_dr_sidept_y = Hu_FrDrEVntSidePt_Y_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_dr_ctrpt_x = Hu_FrDrEVntCtrPt_X_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_dr_ctrpt_y = Hu_FrDrEVntCtrPt_Y_backup;

		nv.nv_item.profile_item[last_mode].items.hu_fr_ps_sidept_x = Hu_FrPsEVntSidePt_X_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_ps_sidept_y = Hu_FrPsEVntSidePt_Y_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_ps_ctrpt_x = Hu_FrPsEVntCtrPt_X_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_ps_ctrpt_y = Hu_FrPsEVntCtrPt_Y_backup;			
		}
		else
		{
		nv.nv_item.profile_item[last_mode].items.hu_fr_dr_sidept_x = Hu_FrDrEVntSidePt_X_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_dr_sidept_y = Hu_FrDrEVntSidePt_Y_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_dr_ctrpt_x = Hu_FrDrEVntCtrPt_X_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_dr_ctrpt_y = Hu_FrDrEVntCtrPt_Y_backup;

		nv.nv_item.profile_item[last_mode].items.hu_fr_ps_sidept_x = Hu_FrPsEVntSidePt_X_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_ps_sidept_y = Hu_FrPsEVntSidePt_Y_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_ps_ctrpt_x = Hu_FrPsEVntCtrPt_X_backup;
		nv.nv_item.profile_item[last_mode].items.hu_fr_ps_ctrpt_y = Hu_FrPsEVntCtrPt_Y_backup;				
		}
#ifdef AMO_GN7_PE_SETTING_NONE
		nv.nv_item.profile_item[last_mode].items.hu_rr_dr_constpt_x = Hu_RrDrEVntConsPt_X;
		nv.nv_item.profile_item[last_mode].items.hu_rr_dr_constpt_y = Hu_RrDrEVntConsPt_Y;
		nv.nv_item.profile_item[last_mode].items.hu_rr_ps_constpt_x = Hu_RrPsEVntConsPt_X;
		nv.nv_item.profile_item[last_mode].items.hu_rr_ps_constpt_x = Hu_RrPsEVntConsPt_Y;			
#endif
		nv.nv_item.write_cnt = nv.nv_item.write_cnt + 1;
		nv_max_cnt = nv.nv_item.write_cnt;
		nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 441);
		nvmem_write(nv.nv_data, PAGE_SIZE);	
	}
}

/*
* @brief  Current_Setting_Setup function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Current_Setting_Setup(void)
{
	#ifdef DEBUG_MODE
	printf("Current_Setting_Setup \r\n");
	#endif

	backup_Flag = TRUE;
	
	/*start Current_Setting_Setup value */
	if((Hu_Frdr_Vent_Side_sig_backup == 0x02) || (Hu_Frdr_Vent_Center_sig_backup == 0x02) || (Hu_FrPs_Vent_Side_sig_backup == 0x02) || (Hu_FrPs_Vent_Center_sig_backup == 0x02))
	{
		Last_Mode_dr_fr_set_func();
		Last_Mode_dr_fr_set_fullclose_func();
	}
	else
	{
		Last_Mode_dr_fr_set_func();
	}
	
	backup_Flag = FALSE;
}

/*
* @brief  Last_Setting_Setup function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Last_Setting_Setup(Profile_mode mode_set)
{
	nvmem_data_type nv;
	//nvmem_read(&nv);
	read_nv_func(&nv);
#ifdef DEBUG_MODE
	printf("Last_Setting_Setup = %d \r\n", mode_set);
#endif

	/*Initialize Last_Setting_Setup value */
	Hu_Fr_Dr_Vent_Mode = (Evnt_mode)nv.nv_item.profile_item[mode_set].items.fr_dr_mode;
	Hu_Fr_Dr_Vent_Mode_Remember = Hu_Fr_Dr_Vent_Mode;
	Hu_Fr_Ps_Vent_Mode = (Evnt_mode)nv.nv_item.profile_item[mode_set].items.fr_ps_mode;
	Hu_Fr_Ps_Vent_Mode_Remember = Hu_Fr_Ps_Vent_Mode;
#ifdef AMO_GN7_PE_SETTING_NONE			
	Hu_Rr_Dr_Vent_Mode = nv.nv_item.profile_item[mode_set].items.rear_dr_mode;
	Hu_Rr_Ps_Vent_Mode = nv.nv_item.profile_item[mode_set].items.rear_ps_mode;
#endif		
	Hu_Frdr_Vent_Side_sig_backup = nv.nv_item.profile_item[mode_set].items.fr_dr_side_opncls;
	Hu_Frdr_Vent_Center_sig_backup = nv.nv_item.profile_item[mode_set].items.fr_dr_ctr_opncls;
	Hu_FrPs_Vent_Side_sig_backup = nv.nv_item.profile_item[mode_set].items.fr_ps_side_opncls;
	Hu_FrPs_Vent_Center_sig_backup = nv.nv_item.profile_item[mode_set].items.fr_ps_ctr_opncls;
#ifdef AMO_GN7_PE_SETTING_NONE
	Hu_Rr_Dr_Vent_signal = nv.nv_item.profile_item[mode_set].items.rr_dr_side_opncls; 
	Hu_Rr_Ps_Vent_signal = nv.nv_item.profile_item[mode_set].items.rr_ps_ctr_opncls; 
#endif			
	Fr_Dr_side_leftright_target_backup = nv.nv_item.profile_item[mode_set].items.fr_dr_side_leftright_target;
	Fr_Dr_side_updown_target_backup = nv.nv_item.profile_item[mode_set].items.fr_dr_side_updown_target;
	Fr_Dr_ctr_leftright_target_backup = nv.nv_item.profile_item[mode_set].items.fr_dr_ctr_leftright_target;
	Fr_Dr_ctr_updown_target_backup = nv.nv_item.profile_item[mode_set].items.fr_dr_ctr_updown_target;
	Fr_Ps_side_leftright_target_backup = nv.nv_item.profile_item[mode_set].items.fr_ps_side_leftright_target;
	Fr_Ps_side_updown_target_backup = nv.nv_item.profile_item[mode_set].items.fr_ps_side_updown_target;
	Fr_Ps_ctr_leftright_target_backup = nv.nv_item.profile_item[mode_set].items.fr_ps_ctr_leftright_target;
	Fr_Ps_ctr_updown_target_backup = nv.nv_item.profile_item[mode_set].items.fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE			
	Rr_Dr_side_leftright_target_backup = nv.nv_item.profile_item[mode_set].items.rr_dr_side_leftright_target;
	Rr_Dr_side_updown_targe_backup = nv.nv_item.profile_item[mode_set].items.rr_dr_side_updown_target;
	Rr_Ps_ctr_leftright_target_backup = nv.nv_item.profile_item[mode_set].items.rr_ps_ctr_leftright_target;
	Rr_Ps_ctr_updown_target_backup = nv.nv_item.profile_item[mode_set].items.rr_ps_ctr_updown_target;
#endif			
	/*positon data backup function */
	Hu_FrDrEVntSidePt_X_backup = nv.nv_item.profile_item[mode_set].items.hu_fr_dr_sidept_x;
	Hu_FrDrEVntSidePt_Y_backup = nv.nv_item.profile_item[mode_set].items.hu_fr_dr_sidept_y;
	Hu_FrDrEVntCtrPt_X_backup = nv.nv_item.profile_item[mode_set].items.hu_fr_dr_ctrpt_x;
	Hu_FrDrEVntCtrPt_Y_backup = nv.nv_item.profile_item[mode_set].items.hu_fr_dr_ctrpt_y;
	Hu_FrPsEVntSidePt_X_backup = nv.nv_item.profile_item[mode_set].items.hu_fr_ps_sidept_x;
	Hu_FrPsEVntSidePt_Y_backup = nv.nv_item.profile_item[mode_set].items.hu_fr_ps_sidept_y;
	Hu_FrPsEVntCtrPt_X_backup = nv.nv_item.profile_item[mode_set].items.hu_fr_ps_ctrpt_x;
	Hu_FrPsEVntCtrPt_Y_backup = nv.nv_item.profile_item[mode_set].items.hu_fr_ps_ctrpt_y;
	if(((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) || (Hu_Fr_Dr_Vent_Mode == FACING_MODE) || (Hu_Fr_Dr_Vent_Mode == SWING_MODE) ||(Hu_Fr_Dr_Vent_Mode == FREE_MODE)) && (((((Hu_FrDrEVntSidePt_X_backup != 0x00) && (Hu_FrDrEVntSidePt_Y_backup != 0x00)) && (Hu_FrDrEVntCtrPt_X_backup != 0x00)) && (Hu_FrDrEVntCtrPt_Y_backup != 0x00))))
	{
		Hu_FrDrEVntSidePt_X_reserve = Hu_FrDrEVntSidePt_X_backup;
		Hu_FrDrEVntSidePt_Y_reserve = Hu_FrDrEVntSidePt_Y_backup;
		Hu_FrDrEVntCtrPt_X_reserve = Hu_FrDrEVntCtrPt_X_backup;
		Hu_FrDrEVntCtrPt_Y_reserve = Hu_FrDrEVntCtrPt_Y_backup;	
	}
	if(((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) || (Hu_Fr_Ps_Vent_Mode == FACING_MODE) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE)) && (((((Hu_FrPsEVntSidePt_X_backup != 0x00) && (Hu_FrPsEVntSidePt_Y_backup != 0x00)) && (Hu_FrPsEVntCtrPt_X_backup != 0x00)) && (Hu_FrPsEVntCtrPt_Y_backup != 0x00))))
	{
		Hu_FrPsEVntSidePt_X_reserve = Hu_FrPsEVntSidePt_X_backup;
		Hu_FrPsEVntSidePt_Y_reserve = Hu_FrPsEVntSidePt_Y_backup;
		Hu_FrPsEVntCtrPt_X_reserve = Hu_FrPsEVntCtrPt_X_backup;
		Hu_FrPsEVntCtrPt_Y_reserve = Hu_FrPsEVntCtrPt_Y_backup;	
	}
#ifdef AMO_GN7_PE_SETTING_NONE			
	Hu_RrDrEVntConsPt_X_backup = nv.nv_item.profile_item[mode_set].items.hu_rr_dr_constpt_x;
	Hu_RrDrEVntConsPt_Y_backup = nv.nv_item.profile_item[mode_set].items.hu_rr_dr_constpt_y;
	Hu_RrPsEVntConsPt_X_backup = nv.nv_item.profile_item[mode_set].items.hu_rr_ps_constpt_x;
	Hu_RrPsEVntConsPt_Y_backup = nv.nv_item.profile_item[mode_set].items.hu_rr_ps_constpt_x;
#endif			
	Current_Setting_Setup();
}

/*
* @brief  Default_Setting_Setup function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Default_Setting_Setup(Profile_mode default_mode)
{
	nvmem_data_type nv;
	//nvmem_read(&nv);
	read_nv_func(&nv);
#ifdef DEBUG_MODE
	printf("Default_Setting_Setup = %d \r\n", default_mode);
#endif

	/*Initialize Default_Setting_Setup value */
	Hu_Fr_Dr_Vent_Mode = (Evnt_mode)nv.nv_item.profile_item[default_mode].items.fr_dr_mode;
	Hu_Fr_Ps_Vent_Mode = (Evnt_mode)nv.nv_item.profile_item[default_mode].items.fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE			
	Hu_Rr_Dr_Vent_Mode =(Evnt_mode)nv.nv_item.profile_item[default_mode].items.rear_dr_mode;
	Hu_Rr_Ps_Vent_Mode =(Evnt_mode)nv.nv_item.profile_item[default_mode].items.rear_ps_mode;
#endif			
	Hu_Fr_dr_Vent_Side_signal = nv.nv_item.profile_item[default_mode].items.fr_dr_side_opncls;
	Hu_Fr_dr_Vent_Center_signal = nv.nv_item.profile_item[default_mode].items.fr_dr_ctr_opncls;
	Hu_Fr_Ps_Vent_Side_signal = nv.nv_item.profile_item[default_mode].items.fr_ps_side_opncls;
	Hu_Fr_Ps_Vent_Center_signal = nv.nv_item.profile_item[default_mode].items.fr_ps_ctr_opncls;
	Hu_Frdr_Vent_Side_sig_backup = nv.nv_item.profile_item[default_mode].items.fr_dr_side_opncls;
	Hu_Frdr_Vent_Center_sig_backup = nv.nv_item.profile_item[default_mode].items.fr_dr_ctr_opncls;
	Hu_FrPs_Vent_Side_sig_backup = nv.nv_item.profile_item[default_mode].items.fr_ps_side_opncls;
	Hu_FrPs_Vent_Center_sig_backup = nv.nv_item.profile_item[default_mode].items.fr_ps_ctr_opncls;	
#ifdef AMO_GN7_PE_SETTING_NONE
	Hu_Rr_Dr_Vent_signal = nv.nv_item.profile_item[default_mode].items.rr_dr_side_opncls; 
	Hu_Rr_Ps_Vent_signal = nv.nv_item.profile_item[default_mode].items.rr_ps_ctr_opncls; 
#endif			
	Fr_Dr_side_leftright_target_backup = nv.nv_item.profile_item[default_mode].items.fr_dr_side_leftright_target;
	Fr_Dr_side_updown_target_backup = nv.nv_item.profile_item[default_mode].items.fr_dr_side_updown_target;
	Fr_Dr_ctr_leftright_target_backup = nv.nv_item.profile_item[default_mode].items.fr_dr_ctr_leftright_target;
	Fr_Dr_ctr_updown_target_backup = nv.nv_item.profile_item[default_mode].items.fr_dr_ctr_updown_target;
	Fr_Ps_side_leftright_target_backup = nv.nv_item.profile_item[default_mode].items.fr_ps_side_leftright_target;
	Fr_Ps_side_updown_target_backup = nv.nv_item.profile_item[default_mode].items.fr_ps_side_updown_target;
	Fr_Ps_ctr_leftright_target_backup = nv.nv_item.profile_item[default_mode].items.fr_ps_ctr_leftright_target;
	Fr_Ps_ctr_updown_target_backup = nv.nv_item.profile_item[default_mode].items.fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE			
	Rr_Dr_side_leftright_target_backup =nv.nv_item.profile_item[default_mode].items.rr_dr_side_leftright_target;
	Rr_Dr_side_updown_targe_backup = nv.nv_item.profile_item[default_mode].items.rr_dr_side_updown_target;
	Rr_Ps_ctr_leftright_target_backup = nv.nv_item.profile_item[default_mode].items.rr_ps_ctr_leftright_target;
	Rr_Ps_ctr_updown_target_backup = nv.nv_item.profile_item[default_mode].items.rr_ps_ctr_updown_target;
#endif			
	/*positon data backup function */
	Hu_FrDrEVntSidePt_X_backup = nv.nv_item.profile_item[default_mode].items.hu_fr_dr_sidept_x;
	Hu_FrDrEVntSidePt_Y_backup = nv.nv_item.profile_item[default_mode].items.hu_fr_dr_sidept_y;
	Hu_FrDrEVntCtrPt_X_backup = nv.nv_item.profile_item[default_mode].items.hu_fr_dr_ctrpt_x;
	Hu_FrDrEVntCtrPt_Y_backup = nv.nv_item.profile_item[default_mode].items.hu_fr_dr_ctrpt_y;
	Hu_FrPsEVntSidePt_X_backup = nv.nv_item.profile_item[default_mode].items.hu_fr_ps_sidept_x;
	Hu_FrPsEVntSidePt_Y_backup = nv.nv_item.profile_item[default_mode].items.hu_fr_ps_sidept_y;
	Hu_FrPsEVntCtrPt_X_backup = nv.nv_item.profile_item[default_mode].items.hu_fr_ps_ctrpt_x;
	Hu_FrPsEVntCtrPt_Y_backup = nv.nv_item.profile_item[default_mode].items.hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE			
	Hu_RrDrEVntConsPt_X_backup = nv.nv_item.profile_item[default_mode].items.hu_rr_dr_constpt_x;
	Hu_RrDrEVntConsPt_Y_backup = nv.nv_item.profile_item[default_mode].items.hu_rr_dr_constpt_y;
	Hu_RrPsEVntConsPt_X_backup = nv.nv_item.profile_item[default_mode].items.hu_rr_ps_constpt_x;
	Hu_RrPsEVntConsPt_Y_backup = nv.nv_item.profile_item[default_mode].items.hu_rr_ps_constpt_x;
#endif
	Last_Mode_dr_fr_set_func();
}

/*
* @brief  handleCanError function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
///////////////////////////////////////////////
static void handleCanError(volatile CAN_Type *base, uint32_t error_flag)
{
	/*Initialize handleCanError function */
	base->ESR1 = error_flag;
	base->MCR = (base->MCR & ~CAN_MCR_SOFTRST_MASK) | CAN_MCR_SOFTRST(1U); 
	//start handle errir
	while (((base->MCR & CAN_MCR_SOFTRST_MASK) >> CAN_MCR_SOFTRST_SHIFT) != 0U)
	{
	}
	Amo_Can_Init();
}

/*
* @brief  flexcan0_ErrorCallback function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void flexcan0_ErrorCallback(uint8_t instance, flexcan_event_type_t eventType, flexcan_state_t *flexcanState)
{
	volatile uint32_t error_ret;

	(void)flexcanState;
	(void)instance;

	CAN_Type * const g_flexcanBase1[] = { CAN0 };	//CAN_BASE_PTRS;
	
	/*Initialize flexcan0_ErrorCallback driver */
	switch(eventType)
	{
		case FLEXCAN_EVENT_ERROR:
		{
			error_ret = FLEXCAN_DRV_GetErrorStatus(INST_CANCOM1);

			CAN_Type * base = g_flexcanBase1[INST_CANCOM1];	
			can_error_data = error_ret;

			const uint32_t error_masks[] =
			{
				CAN_ESR1_BOFFINT_MASK,	// Bus off error
				CAN_ESR1_TXWRN_MASK,		// Transmit error
				CAN_ESR1_RXWRN_MASK,		// Receive error
				CAN_ESR1_STFERR_MASK, 	// Stuff error
				CAN_ESR1_CRCERR_MASK, 	// CRC error
				CAN_ESR1_ACKERR_MASK, 	// Acknowledge error
				CAN_ESR1_FRMERR_MASK, 	// Frame error
				CAN_ESR1_BIT0ERR_MASK,	// Bit error
				CAN_ESR1_BIT1ERR_MASK 	// Bit error
			};

			const uint32_t num_errors = sizeof(error_masks) / sizeof(error_masks[0]);

			/* start can error handle setting */
			for(uint32_t i = 0; i < num_errors; i++)
			{
				if(((error_ret & error_masks[i]) !=0))
				{
					handleCanError(base, error_masks[i]);
				}
			}			

			/* CAN_ESR1_FLTCONF & CAN_ESR1_FLTCONF check */
			if (((error_ret & CAN_ESR1_FLTCONF(2)) !=0) || ((error_ret & CAN_ESR1_FLTCONF(3)) !=0))
			{
					handleCanError(base, CAN_ESR1_BOFFINT_MASK);
			}
			
		break;
		}
		default:
		/* default code */
		break;
	}
}

/*
* @brief  Amo_Can_Init function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Amo_Can_Init(void)
{
	uint16_t id_counter = 0;

	/*Initialize FlexCAN driver */
	FLEXCAN_DRV_Init(INST_CANCOM1, &canCom1_State, &canCom1_InitConfig0);
	
	/* Install callback function */
	FLEXCAN_DRV_InstallEventCallback(INST_CANCOM1, flexcan0_Callback, NULL);
	FLEXCAN_DRV_InstallErrorCallback(INST_CANCOM1, (flexcan_error_callback_t)flexcan0_ErrorCallback, (void*)NULL);

	/* Set information about the data to be received */
	flexcan_data_info_t dataInfo =
	{
		.data_length = 8U,
		.msg_id_type = FLEXCAN_MSG_ID_STD,
		.enable_brs  = false,
		.fd_enable	 = false,
		.fd_padding  = 0U
	};

	/* Configure RX message buffer with index RX_MSG_ID and RX_MAILBOX */
	for(uint8_t i=0; i<CAN_RX_NUM; i++)
	{
		FLEXCAN_DRV_ConfigRxMb(INST_CANCOM1, i, &dataInfo, IDlist[i]);
	}

	// Fill id filter table,
	for(id_counter=0;id_counter<24;id_counter++)
	{
		filterTable[id_counter].isRemoteFrame = false;
		filterTable[id_counter].isExtendedFrame = false;
		filterTable[id_counter].id = IDlist[id_counter];
	}
	/* Configure RX FIFO ID filter table elements based on filter table defined above*/
	FLEXCAN_DRV_ConfigRxFifo(INST_CANCOM1, FLEXCAN_RX_FIFO_ID_FORMAT_A, filterTable);
	/* set individual masking type */
	//FLEXCAN_DRV_SetRxMaskType(INST_CANCOM1, FLEXCAN_RX_MASK_INDIVIDUAL);

	/* Start receiving data in RX_MAILBOX. */
	//FLEXCAN_DRV_Receive(INST_CANCOM1, RX_MAILBOX, &recvBuff1);
	/* Start receiving data in RX_RXFIFO. */
	FLEXCAN_DRV_RxFifo(INST_CANCOM1,&recvBuff2);

	Init_CanTx_Parameter();
}


#endif /* AMO_CAN_C */

