#ifndef __AMO_CAN_C
#define __AMO_CAN_C

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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stdlib.h"

#include "Amo_CAN_Parsing.h"
#include "ewl_misra_types.h"
#include "Amo_STATE_Event.h"
#include "Amo_main.h"
#include "Amo_timer.h"
#include "Amo_nvm.h"
#include "Amo_Lin.h"
/******************************************************************************
 * Groval variable prototypes
 ******************************************************************************/
unsigned long long vent_evt[256] = {0x00,};
int q_front = 0, q_rear = 0;

extern uint8_t rxMBdone;
extern uint8_t rxFIFOdone;
uint8_t Can_rx_success = 0;

/* Define user receive buffer */
extern flexcan_msgbuff_t recvBuff1, recvBuff2;
extern flexio_uart_state_t   uartStateTX;
uint8_t Evnt_Network_Release = 1;
/******************************************************************************
 * Local variable Define
 ******************************************************************************/

/* ID Filter table */
flexcan_id_table_t filterTable[24];

uint16_t IDlist[24] = {0x22,0x23,0x24,0x25,0x28,0x29,0x111,/*0x112,0x115,*/0x119,0x131,0x132,0x122,0x123,0x124,0x125,0x126,0x127,0x128,0x129,0x130,0x161,0x162,0,0,0};
uint16_t IDmask[10] = {0x7E0,0x7F0,0x7FC,0x7F0,0x7FF,0x7FC,0x7E0,0x7FC,0x7F8,0x7F8};
uint8_t Tx_candata[8] = {0,};
static uint8_t callback_test = 0;
unsigned int can_error_data = 0;
/******************************************************************************
 * CAN DATA Groval variable prototypes
 ******************************************************************************/
uint8_t Vent_Mode_FrDrOnOff = 0;
uint8_t Vent_Mode_FrPassOnOff = 0;
uint8_t Vent_Mode_RrDrOnOff = 0;
uint8_t Vent_Mode_RrPassOnOff = 0;

uint8_t can_rx_2_expire = 0, can_rx_3_expire = 0, can_rx_4_expire = 0, can_rx_5_expire = 0, can_rx_8_expire = 0, can_rx_9_expire = 0, can_rx_12_expire = 0, can_rx_13_expire = 0, can_rx_16_expire = 0;
uint8_t can_rx_23_expire = 0, can_rx_24_expire = 0, can_rx_25_expire = 0, can_rx_29_expire = 0, can_rx_30_expire = 0, can_rx_31_expire = 0;

extern uint8_t Evnt_IGN2_Onoff;
extern uint8_t Evnt_IGN3_Onoff;

extern uint8_t key_full_left_close;
extern uint8_t key_full_right_close;

uint8_t P_Oip_Cts_priority = 0;

TaskProiority P_Oip_Cts_Select = P_OIP_SELECT;	//NONE_SELECT;			//P_OIP, CTS Priority
Touch_mode_Dr Touch_mode_Dr_set = 0;
Touch_mode_Ps Touch_mode_Ps_set = 0;
Touch_mode_Rr Touch_mode_Rr_set = 0;


int16_t Fr_Dr_side_leftright_target_backup = 0;
int16_t Fr_Dr_side_updown_target_backup = 0;
int16_t Fr_Dr_ctr_leftright_target_backup = 0;
int16_t Fr_Dr_ctr_updown_target_backup = 0;
int16_t Fr_Ps_side_leftright_target_backup = 0;
int16_t Fr_Ps_side_updown_target_backup = 0;
int16_t Fr_Ps_ctr_leftright_target_backup = 0;
int16_t Fr_Ps_ctr_updown_target_backup = 0;
int16_t Rr_Dr_side_leftright_target_backup = 0;
int16_t Rr_Dr_side_updown_targe_backup = 0;
int16_t Rr_Ps_ctr_leftright_target_backup = 0;
int16_t Rr_Ps_ctr_updown_target_backup = 0;

extern Profile_mode last_mode_save;
Profile_mode lvnt_current_mode = DEFAULT_PROFILE;

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
Atcu_5_data Can_data_5_Info;

uint8_t ATCU_RearOperationStatus = 0;
Drivermode ATCU_Rr_Dr_Mode_display = OFF;
Drivermode ATCU_Rr_Ps_Mode_display = OFF;

///////////////////////////////////////////////////////////////////////////////
Atcu_8_data Can_data_8_Info;

uint8_t ATCU_CamModeStatus = 0;
uint8_t ATCU_UtilityModeUSMStatus = 0;

///////////////////////////////////////////////////////////////////////////////
Atcu_9_data Can_data_9_Info;

uint8_t ATCU_DetentOutStatus = 0;			//Detent off = 0, Detent On = 1
uint8_t ATCU_AutoBrightStatus = 0;
uint8_t ATCU_NotMinimumModeBrightStatus = 0;
uint8_t L_ATCU_IAUProfileValue = 0;
uint8_t L_ATCU_AVNProfileValue = 0;

///////////////////////////////////////////////////////////////////////////////
Atcu_12_data Can_data_12_Info;

uint8_t ATCU_DriverSideType = 0;			//0x00 : LHD, 0x01 : RHD
uint8_t ATCU_FrBlwLvl = 0;
uint8_t ATCU_PassBlwLvl = 0;

///////////////////////////////////////////////////////////////////////////////
Atcu_20_data Can_data_20_Info;	//P-OIP RECEIVED Front driver

uint16_t Hu_FrDrEVntSidePt_X = 0;
uint16_t Hu_FrDrEVntSidePt_Y = 0;
uint16_t Hu_FrDrEVntCtrPt_X = 0;
uint16_t Hu_FrDrEVntCtrPt_Y = 0;

uint16_t Hu_FrDrEVntSideCurrentPt_X = 0;
uint16_t Hu_FrDrEVntSideCurrentPt_Y = 0;
uint16_t Hu_FrDrEVntCtrCurrentPt_X = 0;
uint16_t Hu_FrDrEVntCtrCurrentPt_Y = 0;
///////////////////////////////////////////////////////////////////////////////
Atcu_21_data Can_data_21_Info; //P-OIP RECEIVED Front passenger

uint16_t Hu_FrPsEVntSidePt_X = 0;
uint16_t Hu_FrPsEVntSidePt_Y = 0;
uint16_t Hu_FrPsEVntCtrPt_X = 0;
uint16_t Hu_FrPsEVntCtrPt_Y = 0;

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
Atcu_22_data Can_data_22_Info; //P-OIP RECEIVED Rear driver/passenger

Evnt_mode Hu_Rr_Dr_Vent_Mode = 0x05;
Evnt_mode Hu_Rr_Ps_Vent_Mode = 0x05;
Evnt_mode Hu_Rr_Dr_Vent_Mode_Remember = 0;
Evnt_mode Hu_Rr_Ps_Vent_Mode_Remember = 0;

uint8_t Hu_Rr_Dr_Vent_signal = 0;
uint8_t Hu_Rr_Ps_Vent_signal = 0;

uint16_t Hu_RrDrEVntConsPt_X = 0;
uint16_t Hu_RrDrEVntConsPt_Y = 0;
uint16_t Hu_RrPsEVntConsPt_X = 0;
uint16_t Hu_RrPsEVntConsPt_Y = 0;

uint16_t Hu_RrDrEVntConsCurrentPt_X = 0;
uint16_t Hu_RrDrEVntConsCurrentPt_Y = 0;
uint16_t Hu_RrPsEVntConsCurrentPt_X = 0;
uint16_t Hu_RrPsEVntConsCurrentPt_Y = 0;

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
Atcu_25_data Can_data_25_Info;

uint16_t Hu_Rr_Dr_Vent_Cons_Bdry_Up_X = 0;
uint16_t Hu_Rr_Dr_Vent_Cons_Bdry_Lo_X = 0;
uint16_t Hu_Rr_Dr_Vent_Cons_Bdry_Up_Y = 0;
uint16_t Hu_Rr_Dr_Vent_Cons_Bdry_Lo_Y = 0;
uint16_t Hu_Rr_Ps_Vent_Cons_Bdry_Up_X = 0;
uint16_t Hu_Rr_Ps_Vent_Cons_Bdry_Lo_X = 0;
uint16_t Hu_Rr_Ps_Vent_Cons_Bdry_Up_Y = 0;
uint16_t Hu_Rr_Ps_Vent_Cons_Bdry_Lo_Y = 0;

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
Atcu_28_data Can_data_28_Info;  //CTS RECEIVED 

Evnt_mode Cts_Rr_Dr_Vent_Mode = 0;
Evnt_mode Cts_Rr_Ps_Vent_Mode = 0;
Evnt_mode Cts_Rr_Dr_Vent_Mode_remember = 0;
Evnt_mode Cts_Rr_Ps_Vent_Mode_remember = 0;

uint8_t Cts_Rr_Dr_Vent_siganl = 0;
uint8_t Cts_Rr_Ps_Vent_siganl = 0;

uint16_t Cts_RrDrEVntConsPt_X = 0;
uint16_t Cts_RrDrEVntConsPt_Y = 0;
uint16_t Cts_RrPsEVntConsPt_X = 0;
uint16_t Cts_RrPsEVntConsPt_Y = 0;

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
Atcu_31_data Can_data_31_Info;

uint16_t Cts_Rr_Dr_Vent_Cons_Bdry_Up_X = 0;
uint16_t Cts_Rr_Dr_Vent_Cons_Bdry_Lo_X = 0;
uint16_t Cts_Rr_Dr_Vent_Cons_Bdry_Up_Y = 0;
uint16_t Cts_Rr_Dr_Vent_Cons_Bdry_Lo_Y = 0;
uint16_t Cts_Rr_Ps_Vent_Cons_Bdry_Up_X = 0;
uint16_t Cts_Rr_Ps_Vent_Cons_Bdry_Lo_X = 0;
uint16_t Cts_Rr_Ps_Vent_Cons_Bdry_Up_Y = 0;
uint16_t Cts_Rr_Ps_Vent_Cons_Bdry_Lo_Y = 0;

///////////////////////////////////////////////////////////////////////////////
Atcu_32_data Can_data_32_Info;

Evnt_mode Hu_Fr_Dr_Vent_Mode = 0x05;
Evnt_mode Hu_Fr_Ps_Vent_Mode = 0x05;
Evnt_mode Hu_Fr_Dr_Vent_Mode_Remember = 0;
Evnt_mode Hu_Fr_Ps_Vent_Mode_Remember = 0;

uint8_t Hu_Fr_dr_Vent_Side_signal = 0;
uint8_t Hu_Fr_dr_Vent_Center_signal = 0;
uint8_t Hu_Fr_Ps_Vent_Side_signal = 0;
uint8_t Hu_Fr_Ps_Vent_Center_signal = 0;

///////////////////////////////////////////////////////////////////////////////
Atcu_33_data Can_data_33_Info;

Evnt_mode Cts_Fr_Dr_Vent_Mode = 0x05;
Evnt_mode Cts_Fr_Ps_Vent_Mode = 0x05;
Evnt_mode Cts_Fr_Dr_Vent_Mode_remember = 0;
Evnt_mode Cts_Fr_Ps_Vent_Mode_remember = 0;

uint8_t Cts_Fr_dr_Vent_Side_siganl = 0;
uint8_t Cts_Fr_dr_Vent_Center_signal = 0;
uint8_t Cts_Fr_Ps_Vent_Side_signal = 0;
uint8_t Cts_Fr_Ps_Vent_Center_signal = 0;

///////////////////////////////////////////////////////////////////////////////

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
void SendCANData(uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len);
void flexcan0_Callback(uint8_t instance, flexcan_event_type_t eventType, uint32_t buffIdx, flexcan_state_t *flexcanState); 
void flexcan0_ErrorCallback(uint8_t instance, flexcan_event_type_t eventType, flexcan_state_t *flexcanState);

/*
* @brief: Send data via CAN to the specified mailbox with the specified message id
* @param mailbox 	: Destination mailbox number
* @param messageId : Message ID
* @param data			: Pointer to the TX data
* @param len 			: Length of the TX data
* @return					: None
*/
void SendCANData(uint32_t mailbox, uint32_t messageId, uint8_t * data, uint32_t len)
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
 FLEXCAN_DRV_ConfigTxMb(INST_CANCOM1, mailbox, &dataInfo, messageId);

 /* Execute send non-blocking */
 FLEXCAN_DRV_Send(INST_CANCOM1, mailbox, &dataInfo, messageId, data);
 //FLEXCAN_DRV_SendBlocking(INST_CANCOM1, mailbox, &dataInfo, messageId, data, 100); 
 while(FLEXCAN_DRV_GetTransferStatus(INST_CANCOM1,TX_MAILBOX)==STATUS_BUSY);

}

void Evt_queue_init()
{
	q_front = 0;
	q_rear = 0;

	memset(vent_evt, 0x00, sizeof(vent_evt));
}

char Evt_queue_full()
{
	//return mq->front == (mq->rear+1) % SIZE;
	return ((q_rear + 1) % EVT_QUEUE == q_front);
}

char Evt_queue_empty()
{
	return q_front == q_rear;
}

char Evt_queue_add(unsigned long long m_data)
{
	char ret = 0;

	if(Evt_queue_full())
	{
	 return 1;
	}

	q_rear = (q_rear + 1) % EVT_QUEUE;
	vent_evt[q_rear] = m_data;

	return ret;
}

unsigned long long Evt_queue_value()
{
	char ret = 0;

	if(Evt_queue_empty())
	{
	 return ret;
	}

	q_front = (q_front + 1) % EVT_QUEUE;

	return vent_evt[q_front];
}

///////////////////////////////////////////////////////////////////////////////
void CAN_Recovery_TX()
{
	Can_Tx_Evnt_9.data.L_EVNT_AstCntrHorExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_AstCntrVertExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsHorExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsVertExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsHorExtFlt = 0x01;
	Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsVertExtFlt = 0x01;		
}

//CAN RX RECOVERY CHECK
void Can_rx_2000_timer_recovery()
{
	if(((((((can_rx_2_expire == 0) && (can_rx_3_expire == 0)) && (can_rx_4_expire == 0)) && (can_rx_5_expire == 0)) && (can_rx_8_expire == 0)) && (can_rx_9_expire == 0)) && (can_rx_12_expire == 0))
	{
		Amo_timer_Stop(timer_14);	
		Evt_queue_add(DEVICE_CAN_RXDATA_RECOVERY_EVENT);
	}
	else
	{
		//CAN RX ERROR
		Can_Tx_Evnt_9.data.L_EVNT_AstCntrHorExtFlt = 0x02;
		Can_Tx_Evnt_9.data.L_EVNT_AstCntrVertExtFlt = 0x02;
		Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsHorExtFlt = 0x02;
		Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsVertExtFlt = 0x02;
		Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsHorExtFlt = 0x02;
		Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsVertExtFlt = 0x02;

	}
}

void Can_rx_5000_timer_recovery()
{
	if((((((can_rx_23_expire == 0) && (can_rx_24_expire == 0)) && (can_rx_25_expire == 0)) && (can_rx_29_expire == 0)) && (can_rx_30_expire == 0)) && (can_rx_31_expire == 0))
	{
		Amo_timer_Stop(timer_15);	
		Evt_queue_add(DEVICE_CAN_RXDATA_RECOVERY_EVENT);
	}
	else
	{
		//CAN RX ERROR
		Can_Tx_Evnt_9.data.L_EVNT_AstCntrHorExtFlt = 0x02;
		Can_Tx_Evnt_9.data.L_EVNT_AstCntrVertExtFlt = 0x02;
		Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsHorExtFlt = 0x02;
		Can_Tx_Evnt_9.data.L_EVNT_RrAstCnsVertExtFlt = 0x02;
		Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsHorExtFlt = 0x02;
		Can_Tx_Evnt_9.data.L_EVNT_RrDrvCnsVertExtFlt = 0x02;		
		Evnt_Network_Release = 0x00;
	}
}

///////////////////////////////////////////////////////////////////////////////
//CAN RX TIME OUT CHECK
void Can_rx_2_timer()
{
	//Actuator stop !!!
	can_rx_2_expire = 1;
	Amo_timer_Stop(timer_14);
	Amo_timer_Start(timer_14, 2000, true, Can_rx_2000_timer_recovery);	
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Can_rx_3_timer()
{
	//Actuator stop !!!
	can_rx_3_expire = 1;
	Amo_timer_Stop(timer_14);
	Amo_timer_Start(timer_14, 2000, true, Can_rx_2000_timer_recovery);	
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Can_rx_4_timer()
{
	//Actuator stop !!!
	can_rx_4_expire = 1;
	Amo_timer_Stop(timer_14);
	Amo_timer_Start(timer_14, 2000, true, Can_rx_2000_timer_recovery);	
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Can_rx_5_timer()
{
	//Actuator stop !!!
	can_rx_5_expire = 1;
	Amo_timer_Stop(timer_14);
	Amo_timer_Start(timer_14, 2000, true, Can_rx_2000_timer_recovery);	
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Can_rx_8_timer()
{
	//Actuator stop !!!
	can_rx_8_expire = 1;
	Amo_timer_Stop(timer_14);
	Amo_timer_Start(timer_14, 2000, true, Can_rx_2000_timer_recovery);		
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Can_rx_9_timer()
{
	//Actuator stop !!!
	can_rx_9_expire = 1;
	Amo_timer_Stop(timer_14);
	Amo_timer_Start(timer_14, 2000, true, Can_rx_2000_timer_recovery);		
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Can_rx_12_timer()
{
	//Actuator stop !!!
	can_rx_12_expire = 1;
	Amo_timer_Stop(timer_14);
	Amo_timer_Start(timer_14, 2000, true, Can_rx_2000_timer_recovery);		
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

#if 0
void Can_rx_13_timer()
{
	//Actuator stop !!!
	can_rx_13_expire = 1;
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Can_rx_16_timer()
{
	//Actuator stop !!!
	can_rx_16_expire = 1;
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}
#endif
void Can_rx_23_timer()
{
	//Actuator stop !!!
	can_rx_23_expire = 1;
	Amo_timer_Stop(timer_15);
	Amo_timer_Start(timer_15, 5000, true, Can_rx_5000_timer_recovery);		
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Can_rx_24_timer()
{
	//Actuator stop !!!
	can_rx_24_expire = 1;
	Amo_timer_Stop(timer_15);
	Amo_timer_Start(timer_15, 5000, true, Can_rx_5000_timer_recovery);	
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Can_rx_25_timer()
{
	//Actuator stop !!!
	can_rx_25_expire = 1;
	Amo_timer_Stop(timer_15);
	Amo_timer_Start(timer_15, 5000, true, Can_rx_5000_timer_recovery);
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Can_rx_29_timer()
{
	//Actuator stop !!!
	can_rx_29_expire = 1;
	Amo_timer_Stop(timer_15);
	Amo_timer_Start(timer_15, 5000, true, Can_rx_5000_timer_recovery);
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Can_rx_30_timer()
{
	//Actuator stop !!!
	can_rx_30_expire = 1;
	Amo_timer_Stop(timer_15);
	Amo_timer_Start(timer_15, 5000, true, Can_rx_5000_timer_recovery);
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Can_rx_31_timer()
{
	//Actuator stop !!!
	can_rx_31_expire = 1;
	Amo_timer_Stop(timer_15);
	Amo_timer_Start(timer_15, 5000, true, Can_rx_5000_timer_recovery);	
	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
}

void Init_CanTx_Parameter(void)
{
	memset(&Can_Tx_Evnt_1, 0x00, sizeof(Can_Tx_Evnt_1));
	memset(&Can_Tx_Evnt_2, 0x00, sizeof(Can_Tx_Evnt_2));
	memset(&Can_Tx_Evnt_3, 0x00, sizeof(Can_Tx_Evnt_3));
	memset(&Can_Tx_Evnt_4, 0x00, sizeof(Can_Tx_Evnt_4));
	memset(&Can_Tx_Evnt_5, 0x00, sizeof(Can_Tx_Evnt_5));
	memset(&Can_Tx_Evnt_6, 0x00, sizeof(Can_Tx_Evnt_6));
	memset(&Can_Tx_Evnt_7, 0x00, sizeof(Can_Tx_Evnt_7));
	memset(&Can_Tx_Evnt_8, 0x00, sizeof(Can_Tx_Evnt_8));
	memset(&Can_Tx_Evnt_9, 0x00, sizeof(Can_Tx_Evnt_9));

////////////////////////////////////////////////////////////////////
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_X = 0;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntSidePtDisp_Y = 0;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_X = 0;
	Can_Tx_Evnt_1.data.L_EVNT_HU_FrDrEvntCtrPtDisp_Y = 0;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_X = 0;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntSidePtDisp_Y = 0;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_X = 0;
	Can_Tx_Evnt_2.data.L_EVNT_HU_FrPsEvntCtrPtDisp_Y = 0;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_3.data.L_EVNT_HU_RrDrEvntConsPtDisp_X = 0;
	Can_Tx_Evnt_3.data.L_EVNT_HU_RrDrEvntConsPtDisp_Y = 0;
	Can_Tx_Evnt_3.data.L_EVNT_HU_RrPsEvntConsPtDisp_X = 0;
	Can_Tx_Evnt_3.data.L_EVNT_HU_RrPsEvntConsPtDisp_Y = 0;

	Can_Tx_Evnt_3.data.L_EVNT_FrDrEvntSideFlt = 0x01;			//No error init 0x01 : No error 0x02: Error
	Can_Tx_Evnt_3.data.L_EVNT_FrDrEvntCtrFlt = 0x01;			//Recovery Mode/Actuator(Lin Bus Off/Interior Error Flag/External Error Flag) Error Noti
	Can_Tx_Evnt_3.data.L_EVNT_FrPsEvntSideFlt = 0x01;
	Can_Tx_Evnt_3.data.L_EVNT_FrPsEvntCtrFlt = 0x01;
	Can_Tx_Evnt_3.data.L_EVNT_RrDrEvntConsFlt = 0x01;
	Can_Tx_Evnt_3.data.L_EVNT_RrPsEvntConsFlt = 0x01;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0;	//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0;
	Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntModeSet = 0;
	Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0;
	Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0;
	Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0;
	Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0;
	Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0;
	Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0;
	Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_CTS = 0;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntSidePtDisp_X = 0;
	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntSidePtDisp_Y = 0;
	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntCtrPtDisp_X = 0;
	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntCtrPtDisp_Y = 0;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntSidePtDisp_X = 0;
	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntSidePtDisp_Y = 0;
	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntCtrPtDisp_X = 0;
	Can_Tx_Evnt_5.data.L_EVNT_CTS_FrDrEvntCtrPtDisp_Y = 0;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_6.data.L_EVNT_CTS_FrPsEvntSidePtDisp_X = 0;
	Can_Tx_Evnt_6.data.L_EVNT_CTS_FrPsEvntSidePtDisp_Y = 0;
	Can_Tx_Evnt_6.data.L_EVNT_CTS_FrPsEvntCtrPtDisp_X = 0;
	Can_Tx_Evnt_6.data.L_EVNT_CTS_FrPsEvntCtrPtDisp_Y = 0;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_7.data.L_EVNT_CTS_RrDrEvntConsPtDisp_X = 0;
	Can_Tx_Evnt_7.data.L_EVNT_CTS_RrDrEvntConsPtDisp_Y = 0;
	Can_Tx_Evnt_7.data.L_EVNT_CTS_RrPsEvntConsPtDisp_X = 0;
	Can_Tx_Evnt_7.data.L_EVNT_CTS_RrPsEvntConsPtDisp_Y = 0;

////////////////////////////////////////////////////////////////////

	Can_Tx_Evnt_8.data.L_EVNT_ATCUComFlt = 0x01;	//default : 0, No error : 0x01, Error : 0x02, Error Indicator : 0x03
	Can_Tx_Evnt_8.data.L_EVNT_EVNTFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvSdVertComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvSdHorComFlt = 0x01;
	Can_Tx_Evnt_8.data.L_EVNT_DrvCntrVentComFlt = 0x01;
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

void Can_Rx_TimeOut(void)
{
	printf("CAN Rx timeout \r\n");	
}

void Evnt_Atcu_Diagmsg(void)
{
	Evt_queue_add(DEVICE_CAN_EVNT_ATCU_SEND_EVENT);
}

void Evnt_Atcu_tx_data(void)
{
	Can_tx_id tx_id;
	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_1.reg_data[0], sizeof(Can_Tx_Evnt_1.reg_data));
	tx_id = L_EVNT_01;
	SendCANData(24, tx_id, Tx_candata, sizeof(Tx_candata));
	
	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_2.reg_data[0], sizeof(Can_Tx_Evnt_2.reg_data));
	tx_id = L_EVNT_02;
	SendCANData(24, tx_id, Tx_candata, sizeof(Tx_candata));
	
	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_3.reg_data[0], sizeof(Can_Tx_Evnt_3.reg_data));
	tx_id = L_EVNT_03;
	SendCANData(24, tx_id, Tx_candata, sizeof(Tx_candata)); 

	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_4.reg_data[0], sizeof(Can_Tx_Evnt_4.reg_data));
	tx_id = L_EVNT_04;
	SendCANData(24, tx_id, Tx_candata, sizeof(Tx_candata));

	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_5.reg_data[0], sizeof(Can_Tx_Evnt_5.reg_data));
	tx_id = L_EVNT_05;
	SendCANData(24, tx_id, Tx_candata, sizeof(Tx_candata));

	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_6.reg_data[0], sizeof(Can_Tx_Evnt_6.reg_data));
	tx_id = L_EVNT_06;
	SendCANData(24, tx_id, Tx_candata, sizeof(Tx_candata));

	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_7.reg_data[0], sizeof(Can_Tx_Evnt_7.reg_data));
	tx_id = L_EVNT_07;
	SendCANData(24, tx_id, Tx_candata, sizeof(Tx_candata));

	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_8.reg_data[0], sizeof(Can_Tx_Evnt_8.reg_data));
	tx_id = L_EVNT_08;
	SendCANData(24, tx_id, Tx_candata, sizeof(Tx_candata));

	memset(Tx_candata, 0x00, sizeof(Tx_candata));
	memcpy(&Tx_candata[0], &Can_Tx_Evnt_9.reg_data[0], sizeof(Can_Tx_Evnt_9.reg_data));
	tx_id = L_EVNT_09;
	SendCANData(24, tx_id, Tx_candata, sizeof(Tx_candata));

}

void flexcan0_Callback(uint8_t instance, flexcan_event_type_t eventType,
					   uint32_t buffIdx, flexcan_state_t *flexcanState)
{
	(void)flexcanState;
	(void)instance;
	(void)buffIdx;
#ifdef AMO_NVM_SETTING	
	nvmem_data_type nv;
#endif
	
	switch(eventType)
	{
	case FLEXCAN_EVENT_RX_COMPLETE:
		{
			rxMBdone = 1;
		}
		break;
	case FLEXCAN_EVENT_RXFIFO_COMPLETE:
		break;
	case FLEXCAN_EVENT_DMA_COMPLETE:
		{
			rxFIFOdone = 1;
			
			//printf("Flex can recvBuff2 ID = 0x%04x recvBuff2.dataLen = %d \r\n", recvBuff2.msgId, recvBuff2.dataLen);

			switch(recvBuff2.msgId)
			{
				case 0x22:
				{
					memset(Can_data_2_Info.reg_data, 0x00, sizeof(Can_data_2_Info.reg_data));
					memcpy(&Can_data_2_Info.reg_data[0], &recvBuff2.data[0], 8);

					can_rx_2_expire = 0;
					ATCU_Fr_Dr_Mode_display = Can_data_2_Info.data.L_ATCU_FrDrModDis;
					ATCU_Fr_Ps_Mode_display = Can_data_2_Info.data.L_ATCU_FrPassModDis;

					if((Evnt_IGN2_Onoff) || (Evnt_IGN3_Onoff))	//IGN2 or IGN3 ON
					{

						if(ATCU_Fr_Dr_Mode_display == VENT || ATCU_Fr_Dr_Mode_display == B_L || ATCU_Fr_Dr_Mode_display == MODE7_DEF_VENT || ATCU_Fr_Dr_Mode_display == MODE7_DEF_VENT_FLOOR || ATCU_Fr_Dr_Mode_display == DEF_VENT_ARROW || ATCU_Fr_Dr_Mode_display == DEF_VENT_FLOOR_ARROW)
						{
							Vent_Mode_FrDrOnOff = 1;
						}
						else
						{
							Vent_Mode_FrDrOnOff = 0;
						}

						if(ATCU_Fr_Ps_Mode_display == VENT || ATCU_Fr_Ps_Mode_display == B_L || ATCU_Fr_Ps_Mode_display == MODE7_DEF_VENT || ATCU_Fr_Ps_Mode_display == MODE7_DEF_VENT_FLOOR || ATCU_Fr_Ps_Mode_display == DEF_VENT_ARROW || ATCU_Fr_Ps_Mode_display == DEF_VENT_FLOOR_ARROW)
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
					Amo_timer_Stop(timer_0);
					Amo_timer_Start(timer_0, 2000, false, Can_rx_2_timer);
					break;
				}
				case 0x23:
				{
					memset(Can_data_3_Info.reg_data, 0x00, sizeof(Can_data_3_Info.reg_data));
					memcpy(&Can_data_3_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_3_expire = 0;
					
					ATCU_RheosatLevelStatue = Can_data_3_Info.data.L_ATCU_RhstaLvlSta;

					Amo_timer_Stop(timer_6);
					Amo_timer_Start(timer_6, 2000, false, Can_rx_3_timer);
					break;
				}				
				case 0x24:
				{
					memset(Can_data_4_Info.reg_data, 0x00, sizeof(Can_data_4_Info.reg_data));
					memcpy(&Can_data_4_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_4_expire = 0;
					
					ATCU_OperationStatus = Can_data_4_Info.data.L_ATCU_OpSta;
					
					Amo_timer_Stop(timer_1);
					Amo_timer_Start(timer_1, 2000, false, Can_rx_4_timer);					
					break;
				}
				case 0x25:
				{
					memset(Can_data_5_Info.reg_data, 0x00, sizeof(Can_data_5_Info.reg_data));
					memcpy(&Can_data_5_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_5_expire = 0;

					ATCU_RearOperationStatus = Can_data_5_Info.data.L_ATCU_RrOpSta;
					ATCU_Rr_Dr_Mode_display = Can_data_5_Info.data.L_ATCU_RrDrModDis;
					ATCU_Rr_Ps_Mode_display = Can_data_5_Info.data.L_ATCU_RrPsModDis;	

					if((Evnt_IGN2_Onoff) || (Evnt_IGN3_Onoff))	//IGN2 or IGN3 ON
					{

						if(ATCU_Rr_Dr_Mode_display == VENT || ATCU_Rr_Dr_Mode_display == B_L || ATCU_Rr_Dr_Mode_display == MODE7_DEF_VENT || ATCU_Rr_Dr_Mode_display == MODE7_DEF_VENT_FLOOR || ATCU_Rr_Dr_Mode_display == DEF_VENT_ARROW || ATCU_Rr_Dr_Mode_display == DEF_VENT_FLOOR_ARROW)
						{
							Vent_Mode_RrDrOnOff = 1;
						}
						else
						{
							Vent_Mode_RrDrOnOff = 0;
						}

						if(ATCU_Rr_Ps_Mode_display == VENT || ATCU_Rr_Ps_Mode_display == B_L || ATCU_Rr_Ps_Mode_display == MODE7_DEF_VENT || ATCU_Rr_Ps_Mode_display == MODE7_DEF_VENT_FLOOR || ATCU_Rr_Ps_Mode_display == DEF_VENT_ARROW || ATCU_Rr_Ps_Mode_display == DEF_VENT_FLOOR_ARROW)
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

					Amo_timer_Stop(timer_2);
					Amo_timer_Start(timer_2, 2000, false, Can_rx_5_timer);				
					break;
				}
				case 0x28:
				{
					memset(Can_data_8_Info.reg_data, 0x00, sizeof(Can_data_8_Info.reg_data));
					memcpy(&Can_data_8_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_8_expire = 0;

					ATCU_CamModeStatus = Can_data_8_Info.data.L_ATCU_CamModeSta;
					ATCU_UtilityModeUSMStatus = Can_data_8_Info.data.L_ATCU_UtilModeUsmSta;

					if((ATCU_CamModeStatus == 0x02) && (ATCU_UtilityModeUSMStatus == 0x02))
					{
						Evt_queue_add(DEVICE_CAN_EVNT_UTILITY_MODE_EVENT);
					}
					
					Amo_timer_Stop(timer_3);
					Amo_timer_Start(timer_3, 2000, false, Can_rx_8_timer);				
					break;
				}
				case 0x29:
				{
					memset(Can_data_9_Info.reg_data, 0x00, sizeof(Can_data_9_Info.reg_data));
					memcpy(&Can_data_9_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_9_expire = 0;

					ATCU_DetentOutStatus = Can_data_9_Info.data.L_ATCU_DtntOutSta;
					ATCU_AutoBrightStatus = Can_data_9_Info.data.L_ATCU_AutoBrightSta;
					ATCU_NotMinimumModeBrightStatus = Can_data_9_Info.data.L_ATCU_NotMiniModeBrightSta;
					L_ATCU_IAUProfileValue = Can_data_9_Info.data.L_ATCU_IAUPrfleVal;		//priority low
					L_ATCU_AVNProfileValue = Can_data_9_Info.data.L_ATCU_AVNPrfleVal;   //priority high
#ifdef AMO_NVM_SETTING
					if(L_ATCU_AVNProfileValue != DEFAULT_PROFILE)
					{
						last_mode_save = L_ATCU_AVNProfileValue;  

						if(lvnt_current_mode != last_mode_save)
						{
							lvnt_current_mode = last_mode_save;

							///////////////////////////////////////////////////////////////////////////	
							nv.profile = last_mode_save;																	//20241217_temp apply delete code
							nvmem_write(NVMEM_PROFILE_ID_IDX,&nv,1);
							////////////////////////////////////////////////////////////////////////////	
							Evt_queue_add(DEVICE_CAN_EVNT_LAST_MODE_EVENT);
						}
					}
#endif
					if(Can_rx_success == 0)
					{
						Can_rx_success = 1;
						Amo_timer_Start(timer_7, 1000, true, Evnt_Atcu_Diagmsg);
					}

					Amo_timer_Stop(timer_4);
					Amo_timer_Start(timer_4, 2000, false, Can_rx_9_timer);
					break;
				}
				case 0x111:
				{
					memset(Can_data_12_Info.reg_data, 0x00, sizeof(Can_data_12_Info.reg_data));
					memcpy(&Can_data_12_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_12_expire = 0;
					ATCU_DriverSideType = Can_data_12_Info.data.L_ATCU_DrvSdTyp;
					ATCU_FrBlwLvl = Can_data_12_Info.data.L_ATCU_FrBlwLvl;
					ATCU_PassBlwLvl = Can_data_12_Info.data.L_ATCU_PassBlwLvl;

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
					memset(Can_data_20_Info.reg_data, 0x00, sizeof(Can_data_20_Info.reg_data));
					memcpy(&Can_data_20_Info.reg_data[0], &recvBuff2.data[0], 8);

					Hu_FrDrEVntSidePt_X = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_X;
					Hu_FrDrEVntSidePt_Y = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_Y;
					Hu_FrDrEVntCtrPt_X = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_X;
					Hu_FrDrEVntCtrPt_Y = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_Y;
/*
					Hu_FrDrEVntSidePt_X_current = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_X;
					Hu_FrDrEVntSidePt_Y_current = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_Y;
					Hu_FrDrEVntCtrPt_X_current = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_X;
					Hu_FrDrEVntCtrPt_Y_current = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_Y;
*/
					CAN_OLD_NEW_PT.FrDrPt.Hu_FrDrEVntSidePt_X_newVal = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_X;
					CAN_OLD_NEW_PT.FrDrPt.Hu_FrDrEVntSidePt_Y_newVal = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntSidePt_Y;
					CAN_OLD_NEW_PT.FrDrPt.Hu_FrDrEVntCtrPt_X_newVal = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_X;
					CAN_OLD_NEW_PT.FrDrPt.Hu_FrDrEVntCtrPt_Y_newVal = Can_data_20_Info.data.L_ATCU_HU_FrDrEVntCtrPt_Y;
					

					if((Evnt_IGN2_Onoff) || (Evnt_IGN3_Onoff))	//IGN2 or IGN3 ON
					{
						if(Vent_Mode_FrDrOnOff)
						{
							if(Hu_Fr_Dr_Vent_Mode == FREE_MODE)
							{
								if(P_Oip_Cts_Select == P_OIP_SELECT)
								{							
									Touch_mode_Dr_set = HU_DRIVER_TOUCH;
									Evt_queue_add(DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT);
								}
							}
						}
					}
					break;
				}
				case 0x131: //P-OIP Receive -> Front passenger vent user move request value 
				{
					memset(Can_data_21_Info.reg_data, 0x00, sizeof(Can_data_21_Info.reg_data));
					memcpy(&Can_data_21_Info.reg_data[0], &recvBuff2.data[0], 8);

					Hu_FrPsEVntSidePt_X = Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_X;
					Hu_FrPsEVntSidePt_Y = Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_Y;
					Hu_FrPsEVntCtrPt_X = Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_X;
					Hu_FrPsEVntCtrPt_Y = Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_Y;	

					CAN_OLD_NEW_PT.FrPsPt.Hu_FrPsEVntSidePt_X_newVal = Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_X;
					CAN_OLD_NEW_PT.FrPsPt.Hu_FrPsEVntSidePt_Y_newVal = Can_data_21_Info.data.L_ATCU_HU_FrPsEVntSidePt_Y;
					CAN_OLD_NEW_PT.FrPsPt.Hu_FrPsEVntCtrPt_X_newVal = Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_X;
					CAN_OLD_NEW_PT.FrPsPt.Hu_FrPsEVntCtrPt_Y_newVal = Can_data_21_Info.data.L_ATCU_HU_FrPsEVntCtrPt_Y;

					if((Evnt_IGN2_Onoff) || (Evnt_IGN3_Onoff))	//IGN2 or IGN3 ON
					{
						if(Vent_Mode_FrPassOnOff)
						{
							if(Hu_Fr_Ps_Vent_Mode == FREE_MODE)
							{
								if(P_Oip_Cts_Select == P_OIP_SELECT)
								{
									Touch_mode_Ps_set = HU_PASSENGER_TOUCH;
									Evt_queue_add(DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT);
								}
							}
						}
					}
					break;
				}
				case 0x132:  //P-OIP Receive -> Rear drive/ Rear passenger vent mode define
				{
					memset(Can_data_22_Info.reg_data, 0x00, sizeof(Can_data_22_Info.reg_data));
					memcpy(&Can_data_22_Info.reg_data[0], &recvBuff2.data[0], 8);
					
					if(!P_Oip_Cts_priority)
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
					
					if((Evnt_IGN2_Onoff) || (Evnt_IGN3_Onoff))	//IGN2 or IGN3 ON
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
					break;
				}	
				case 0x122:  //P-OIP Receive -> Front drive vent boundary define
				{
					memset(Can_data_23_Info.reg_data, 0x00, sizeof(Can_data_23_Info.reg_data));
					memcpy(&Can_data_23_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_23_expire = 0;
					
					Hu_Fr_Dr_Vent_Side_Bdry_Up_X = Can_data_23_Info.data.L_ATCU_HU_FrDrEVntSideBdryUp_X*10;
					Hu_Fr_Dr_Vent_Side_Bdry_Lo_X = Can_data_23_Info.data.L_ATCU_HU_FrDrEVntSideBdryLo_X*10;
					Hu_Fr_Dr_Vent_Side_Bdry_Up_Y = Can_data_23_Info.data.L_ATCU_HU_FrDrEVntSideBdryUp_Y*10;
					Hu_Fr_Dr_Vent_Side_Bdry_Lo_Y = Can_data_23_Info.data.L_ATCU_HU_FrDrEVntSideBdryLo_Y*10;
					Hu_Fr_Dr_Vent_Ctr_Bdry_Up_X = Can_data_23_Info.data.L_ATCU_HU_FrDrEVntCtrBdryUP_X*10;
					Hu_Fr_Dr_Vent_Ctr_Bdry_Lo_X = Can_data_23_Info.data.L_ATCU_HU_FrDrEVntCtrBdryLo_X*10;
					Hu_Fr_Dr_Vent_Ctr_Bdry_Up_Y = Can_data_23_Info.data.L_ATCU_HU_FrDrEVntCtrBdryUP_Y*10;
					Hu_Fr_Dr_Vent_Ctr_Bdry_Lo_Y = Can_data_23_Info.data.L_ATCU_HU_FrDrEVntCtrBdryLo_Y*10;

					Amo_timer_Stop(timer_8);
					Amo_timer_Start(timer_8, 5000, false, Can_rx_23_timer);
					break;
				}
				case 0x123: //P-OIP Receive ->  Front passenger vent boundary define
				{
					memset(Can_data_24_Info.reg_data, 0x00, sizeof(Can_data_24_Info.reg_data));
					memcpy(&Can_data_24_Info.reg_data[0], &recvBuff2.data[0], 8);

					Hu_Fr_Ps_Vent_Side_Bdry_Up_X = Can_data_24_Info.data.L_ATCU_HU_FrPsEVntSideBdryUp_X*10;
					Hu_Fr_Ps_Vent_Side_Bdry_Lo_X = Can_data_24_Info.data.L_ATCU_HU_FrPsEVntSideBdryLo_X*10;
					Hu_Fr_Ps_Vent_Side_Bdry_Up_Y = Can_data_24_Info.data.L_ATCU_HU_FrPsEVntSideBdryUp_Y*10;
					Hu_Fr_Ps_Vent_Side_Bdry_Lo_Y = Can_data_24_Info.data.L_ATCU_HU_FrPsEVntSideBdryLo_Y*10;
					Hu_Fr_Ps_Vent_Ctr_Bdry_Up_X = Can_data_24_Info.data.L_ATCU_HU_FrPsEVntCtrBdryUP_X*10;
					Hu_Fr_Ps_Vent_Ctr_Bdry_Lo_X = Can_data_24_Info.data.L_ATCU_HU_FrPsEVntCtrBdryLo_X*10;
					Hu_Fr_Ps_Vent_Ctr_Bdry_Up_Y = Can_data_24_Info.data.L_ATCU_HU_FrPsEVntCtrBdryUP_Y*10;
					Hu_Fr_Ps_Vent_Ctr_Bdry_Lo_Y = Can_data_24_Info.data.L_ATCU_HU_FrPsEVntCtrBdryLo_Y*10;
					
					Amo_timer_Stop(timer_9);
					Amo_timer_Start(timer_9, 5000, false, Can_rx_24_timer);					
					break;
				}
				case 0x124: //P-OIP Receive -> Rear drive/Rear Passenger vent boundary define
				{
					memset(Can_data_25_Info.reg_data, 0x00, sizeof(Can_data_25_Info.reg_data));
					memcpy(&Can_data_25_Info.reg_data[0], &recvBuff2.data[0], 8);
					can_rx_24_expire = 0;

					Hu_Rr_Dr_Vent_Cons_Bdry_Up_X = Can_data_25_Info.data.L_ATCU_HU_RrDrEVntConsBdryUp_X*10;
					Hu_Rr_Dr_Vent_Cons_Bdry_Lo_X = Can_data_25_Info.data.L_ATCU_HU_RrDrEVntConsBdryLo_X*10;
					Hu_Rr_Dr_Vent_Cons_Bdry_Up_Y = Can_data_25_Info.data.L_ATCU_HU_RrDrEVntConsBdryUp_Y*10;
					Hu_Rr_Dr_Vent_Cons_Bdry_Lo_Y = Can_data_25_Info.data.L_ATCU_HU_RrDrEVntConsBdryLo_Y*10;
					Hu_Rr_Ps_Vent_Cons_Bdry_Up_X = Can_data_25_Info.data.L_ATCU_HU_RrPsEVntConsBdryUP_X*10;
					Hu_Rr_Ps_Vent_Cons_Bdry_Lo_X = Can_data_25_Info.data.L_ATCU_HU_RrPsEVntConsBdryLo_X*10;
					Hu_Rr_Ps_Vent_Cons_Bdry_Up_Y = Can_data_25_Info.data.L_ATCU_HU_RrPsEVntConsBdryUP_Y*10;
					Hu_Rr_Ps_Vent_Cons_Bdry_Lo_Y = Can_data_25_Info.data.L_ATCU_HU_RrPsEVntConsBdryLo_Y*10;

					Amo_timer_Stop(timer_10);
					Amo_timer_Start(timer_10, 5000, false, Can_rx_25_timer);						
					break;
				}
				case 0x125: //CTS Receive -> Front drive vent user move request value 
				{
					memset(Can_data_26_Info.reg_data, 0x00, sizeof(Can_data_26_Info.reg_data));
					memcpy(&Can_data_26_Info.reg_data[0], &recvBuff2.data[0], 8);
					
					Cts_FrDrEVntSidePt_X = Can_data_26_Info.data.L_ATCU_CTS_FrDrEVntSidePt_X;
					Cts_FrDrEVntSidePt_Y = Can_data_26_Info.data.L_ATCU_CTS_FrDrEVntSidePt_Y;
					Cts_FrDrEVntCtrPt_X = Can_data_26_Info.data.L_ATCU_CTS_FrDrEVntCtrPt_X;
					Cts_FrDrEVntCtrPt_Y = Can_data_26_Info.data.L_ATCU_CTS_FrDrEVntCtrPt_Y;

					if((Evnt_IGN2_Onoff) || (Evnt_IGN3_Onoff))	//IGN2 or IGN3 ON
					{
						if(Vent_Mode_FrDrOnOff)
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
					break;
				}
				case 0x126: //CTS Receive -> Front passenger vent user move request value 
				{
					memset(Can_data_27_Info.reg_data, 0x00, sizeof(Can_data_27_Info.reg_data));
					memcpy(&Can_data_27_Info.reg_data[0], &recvBuff2.data[0], 8);
				
					Cts_FrPsEVntSidePt_X = Can_data_27_Info.data.L_ATCU_CTS_FrPsEVntSidePt_X;
					Cts_FrPsEVntSidePt_Y = Can_data_27_Info.data.L_ATCU_CTS_FrPsEVntSidePt_Y;
					Cts_FrPsEVntCtrPt_X = Can_data_27_Info.data.L_ATCU_CTS_FrPsEVntCtrPt_X;
					Cts_FrPsEVntCtrPt_Y = Can_data_27_Info.data.L_ATCU_CTS_FrPsEVntCtrPt_Y;

					if((Evnt_IGN2_Onoff) || (Evnt_IGN3_Onoff))	//IGN2 or IGN3 ON
					{
						if(Vent_Mode_FrPassOnOff)
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
					
					break;
				}
				case 0x127: //CTS Receive -> Rear drive/ Rear passenger vent mode define
				{
					memset(Can_data_28_Info.reg_data, 0x00, sizeof(Can_data_28_Info.reg_data));
					memcpy(&Can_data_28_Info.reg_data[0], &recvBuff2.data[0], 8);

					if(!P_Oip_Cts_priority)
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
					
					if((Evnt_IGN2_Onoff) || (Evnt_IGN3_Onoff))	//IGN2 or IGN3 ON
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
					break;
				}
				case 0x128: //CTS Receive -> Front drive vent boundary define
				{
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
					break;
				}
				case 0x129: //CTS Receive -> Front Passenger vent boundary define
				{
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
					break;
				}
				case 0x130: //CTS Receive -> Rear driver/Rear Passenger vent boundary define
				{
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
					break;
				}				
				case 0x161:		//P-OIP Receive -> Front drive/ Front passenger vent mode define
				{
					memset(Can_data_32_Info.reg_data, 0x00, sizeof(Can_data_32_Info.reg_data));
					memcpy(&Can_data_32_Info.reg_data[0], &recvBuff2.data[0], 8);
					
					if(!P_Oip_Cts_priority)
					{
						P_Oip_Cts_Select = P_OIP_SELECT;
						P_Oip_Cts_priority = 1;
					}

					Hu_Fr_Dr_Vent_Mode = Can_data_32_Info.data.L_ATCU_HU_FrDrEVntModeSet;
					Hu_Fr_Ps_Vent_Mode = Can_data_32_Info.data.L_ATCU_HU_FrPsEVntModeSet;
					Hu_Fr_dr_Vent_Center_signal = Can_data_32_Info.data.L_ATCU_HU_FrDrEVntCtrOpnCls;
					Hu_Fr_dr_Vent_Side_signal = Can_data_32_Info.data.L_ATCU_HU_FrDrEVntSlideOpnCls;
					Hu_Fr_Ps_Vent_Center_signal = Can_data_32_Info.data.L_ATCU_HU_FrPsEVntCtrOpnCls;
					Hu_Fr_Ps_Vent_Side_signal = Can_data_32_Info.data.L_ATCU_HU_FrPsEVntSideOpnCls;

					if((Hu_Fr_Dr_Vent_Mode == FREE_MODE) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE))
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
					
					if((Evnt_IGN2_Onoff) || (Evnt_IGN3_Onoff))	//IGN2 or IGN3 ON
					{
						if((Vent_Mode_FrDrOnOff) || (Vent_Mode_FrPassOnOff))
						{
							if(P_Oip_Cts_Select == P_OIP_SELECT)
							{
								if(Hu_Fr_dr_Vent_Side_signal == 0x02 || Hu_Fr_dr_Vent_Center_signal == 0x02 || Hu_Fr_Ps_Vent_Side_signal == 0x02 || Hu_Fr_Ps_Vent_Center_signal == 0x02)
								{
									if(Hu_Fr_dr_Vent_Side_signal == 0x02)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x02;


										//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x02;
									}

									if(Hu_Fr_dr_Vent_Center_signal == 0x02)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x02;
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x02;
	
									}

									if(Hu_Fr_Ps_Vent_Side_signal == 0x02)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x02;
										//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x02;		

									}

									if(Hu_Fr_Ps_Vent_Center_signal == 0x02)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x02;
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x02;			

									}								
									Evt_queue_add(DEVICE_P_OIP_FULL_CLOSE_CAN_LIN_EVENT);
								}
								

								if(Hu_Fr_dr_Vent_Side_signal == 0x01)
								{
									if(Hu_Fr_Dr_Vent_Mode == 0x00)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Hu_Fr_Dr_Vent_Mode_Remember;		//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
										//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Hu_Fr_Dr_Vent_Mode = Hu_Fr_Dr_Vent_Mode_Remember;
									}
								}

								if(Hu_Fr_dr_Vent_Center_signal == 0x01)
								{
									if(Hu_Fr_Dr_Vent_Mode == 0x00)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Hu_Fr_Dr_Vent_Mode_Remember;		//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Hu_Fr_Dr_Vent_Mode = Hu_Fr_Dr_Vent_Mode_Remember;
									}
								}

								if(Hu_Fr_Ps_Vent_Side_signal == 0x01)
								{
									if(Hu_Fr_Ps_Vent_Mode == 0x00)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Hu_Fr_Ps_Vent_Mode_Remember;		//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
										//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
										Hu_Fr_Ps_Vent_Mode = Hu_Fr_Ps_Vent_Mode_Remember;
									}
								}				

								if(Hu_Fr_Ps_Vent_Center_signal == 0x01)
								{
									if(Hu_Fr_Ps_Vent_Mode == 0x00)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Hu_Fr_Ps_Vent_Mode_Remember;		//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
										Hu_Fr_Ps_Vent_Mode = Hu_Fr_Ps_Vent_Mode_Remember;
									}
								}		

								if((Hu_Fr_Dr_Vent_Mode == FACING_MODE) || (Hu_Fr_Ps_Vent_Mode == FACING_MODE))
								{
									if(Hu_Fr_Dr_Vent_Mode == FACING_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x01;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Hu_Fr_Dr_Vent_Mode_Remember = FACING_MODE;
									}

									if(Hu_Fr_Ps_Vent_Mode == FACING_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x01;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;		
										Hu_Fr_Ps_Vent_Mode_Remember = FACING_MODE;										
									}
									Evt_queue_add(DEVICE_P_OIP_FOCUS_CAN_LIN_EVENT);								
								}

								if((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) || (Hu_Fr_Ps_Vent_Mode == AVOID_MODE))
								{
									if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x02;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Hu_Fr_Dr_Vent_Mode_Remember = AVOID_MODE;
									}

									if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x02;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;		
										Hu_Fr_Ps_Vent_Mode_Remember = AVOID_MODE;										
									}
									Evt_queue_add(DEVICE_P_OIP_SPREAD_CAN_LIN_EVENT);								
								}							

								if((Hu_Fr_Dr_Vent_Mode == SWING_MODE) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE))
								{
									if(Hu_Fr_Dr_Vent_Mode == SWING_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x03;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Hu_Fr_Dr_Vent_Mode_Remember = SWING_MODE;
										button_LH_cycle_check = 0x03;
									}

									if(Hu_Fr_Ps_Vent_Mode == SWING_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x03;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;		
										Hu_Fr_Ps_Vent_Mode_Remember = SWING_MODE;			
										button_RH_cycle_check = 0x03;
									}
									Evt_queue_add(DEVICE_P_OIP_CYCLE_CAN_LIN_EVENT);								
								}	

								if((Hu_Fr_Dr_Vent_Mode == FREE_MODE) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE))
								{
									if(Hu_Fr_Dr_Vent_Mode == FREE_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
//										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
//										Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
										Hu_Fr_Dr_Vent_Mode_Remember = FREE_MODE;
										Touch_mode_Dr_set = HU_DRIVER_TOUCH;
										button_LH_cycle_check = 0x04;
									}

									if(Hu_Fr_Ps_Vent_Mode == FREE_MODE)
									{
										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x04;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
//										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
//										Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;		
										Hu_Fr_Ps_Vent_Mode_Remember = FREE_MODE;
										Touch_mode_Ps_set = HU_PASSENGER_TOUCH;
										button_RH_cycle_check = 0x04;
									}				

									Evt_queue_add(DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT);

									if(P_Oip_Cts_Select == P_OIP_SELECT)
									{
										Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x01; //APPLY Duplucate Input Signal Prevention Control Status return
									}
									else
									{
										Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x02; //NOT APPLY
									}
								}

								if(((Hu_Fr_Dr_Vent_Mode_Remember == 0x00) && (Hu_Fr_dr_Vent_Side_signal != 0x02)) || ((Hu_Fr_Dr_Vent_Mode_Remember == 0x00) && (Hu_Fr_dr_Vent_Center_signal != 0x02)))
								{
									P_Oip_Cts_priority = 0;
								}

								if(((Hu_Fr_Ps_Vent_Mode_Remember == 0x00) && (Hu_Fr_Ps_Vent_Side_signal != 0x02)) || ((Hu_Fr_Ps_Vent_Mode_Remember == 0x00) && (Hu_Fr_Ps_Vent_Center_signal != 0x02)))
								{
									P_Oip_Cts_priority = 0;
								}				
								
								if(Hu_Fr_Dr_Vent_Mode == MODE_OFF)
								{
									P_Oip_Cts_priority = 0;
									Hu_Fr_Dr_Vent_Mode_Remember = MODE_OFF;
									//VENT MODE OFF
								}					
								if(Hu_Fr_Ps_Vent_Mode == MODE_OFF)
								{
									P_Oip_Cts_priority = 0;
									Hu_Fr_Ps_Vent_Mode_Remember = MODE_OFF;
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
					break;
				}
				case 0x162: //CTS Receive -> Front drive/ Front passenger vent mode define
				{
					memset(Can_data_33_Info.reg_data, 0x00, sizeof(Can_data_33_Info.reg_data));
					memcpy(&Can_data_33_Info.reg_data[0], &recvBuff2.data[0], 8);

					if(!P_Oip_Cts_priority)
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

					if((Evnt_IGN2_Onoff) || (Evnt_IGN3_Onoff))	//IGN2 or IGN3 ON
					{
						if((Vent_Mode_FrDrOnOff) || (Vent_Mode_FrPassOnOff))
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
								
								if(Cts_Fr_dr_Vent_Side_siganl == 0x01)
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
					break;

				}
				default:
				break;
			}

		}
		break;
	case FLEXCAN_EVENT_TX_COMPLETE:
		break;
	default:
		break;
	}
}
///////////////////////////////////////////////
//Test
void Hu_Front_mode_FullClose_setting()
{
if(Hu_Fr_dr_Vent_Side_signal == 0x02 || Hu_Fr_dr_Vent_Center_signal == 0x02 || Hu_Fr_Ps_Vent_Side_signal == 0x02 || Hu_Fr_Ps_Vent_Center_signal == 0x02) //Full close mode setting
{
if(Hu_Fr_dr_Vent_Side_signal == 0x02)
{
PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
Hu_Fr_Dr_Vent_Mode = DEFAULT_MODE;
//if(Hu_Fr_Dr_Vent_Mode == 0x00)
//{
lin_FrDrVentMode_task(&LIN_LH_EVNT_MASTER_CMD);
//}


}
 
if(Hu_Fr_dr_Vent_Center_signal == 0x02)
{
PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
Hu_Fr_Dr_Vent_Mode = DEFAULT_MODE;
//if(Hu_Fr_Dr_Vent_Mode == 0x00)
//{
lin_FrDrVentMode_task(&LIN_LH_EVNT_MASTER_CMD);
//}

}
 
if(Hu_Fr_Ps_Vent_Side_signal == 0x02)
{
PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
Hu_Fr_Ps_Vent_Mode = DEFAULT_MODE;
//if(Hu_Fr_Ps_Vent_Mode == 0x00)
//{
lin_FrPsVentMode_task(&LIN_RH_EVNT_MASTER_CMD);
//

}
 
if(Hu_Fr_Ps_Vent_Center_signal == 0x02)
{
PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);  
Hu_Fr_Ps_Vent_Mode = DEFAULT_MODE;
//if(Hu_Fr_Ps_Vent_Mode == 0x00)
//{
lin_FrPsVentMode_task(&LIN_RH_EVNT_MASTER_CMD);
//}

}  
}

}


void Hu_Front_mode_setting()
{
	 if((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01))
	 {
	 lin_FrDrVentMode_task(&LIN_LH_EVNT_MASTER_CMD);
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
	 }
	 
	 if((Hu_Fr_Dr_Vent_Mode == FACING_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01))
	 {
	 	lin_FrDrVentMode_task(&LIN_LH_EVNT_MASTER_CMD);
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
	 }
	 
	 if((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01))
	 {
	 	 lin_FrPsVentMode_task(&LIN_RH_EVNT_MASTER_CMD);
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
	 }
	 
	 if((Hu_Fr_Ps_Vent_Mode == FACING_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01))
	 {
	 lin_FrPsVentMode_task(&LIN_RH_EVNT_MASTER_CMD);	
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
	 }
	 
	 if((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_dr_Vent_Side_signal == 0x01))
	 {
	 lin_FrDrVentMode_task(&LIN_LH_EVNT_MASTER_CMD);
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
	 }
	 
	 if((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) && (Hu_Fr_dr_Vent_Center_signal == 0x01))
	 {
	 lin_FrDrVentMode_task(&LIN_LH_EVNT_MASTER_CMD);
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
	 }
	 
	 if((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Side_signal == 0x01)) 
	 {
	 lin_FrPsVentMode_task(&LIN_RH_EVNT_MASTER_CMD);
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
	 }
	 
	 if((Hu_Fr_Ps_Vent_Mode == AVOID_MODE) && (Hu_Fr_Ps_Vent_Center_signal == 0x01)) 
	 {
	 lin_FrPsVentMode_task(&LIN_RH_EVNT_MASTER_CMD);
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
	 }

	 if(((Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01)) && ((Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01)))
	 {
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
		 lin_FrDrPsCycleMode_task();
		 PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
		 PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		 PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
		 PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
	 }
	 else if((Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == SWING_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01))  
	 {
	 if(key_full_left_close == 1)
{
	key_full_left_close = 0;
}
	 	button_LH_cycle_check = 0x03;
		 lin_FrDrCycleMode_task(&LIN_LH_EVNT_MASTER_CMD);
		 PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
		 PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
	 } 
	 
	 else if((Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01)) 	 
	 {
	 	button_RH_cycle_check = 0x03;
		 lin_FrPsCycleMode_task(&LIN_RH_EVNT_MASTER_CMD);
		 if(key_full_right_close == 1)
{
	key_full_right_close = 0;
}
		 
		 PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
		 PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
	 }
	 else
	 {

	 }

	 #if 0
	 if((Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01)) 
	 {

	 } 
	 
	 if((Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01))  
	 {

	 }
	 #endif
}


void Hu_Front_mode_manual_key_setting()
{
//	lin_FrDrCycleMode_TimerStop();
if(button_LH_cycle_check == 0x03 && button_RH_cycle_check == 0x03)
{
	if((Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01))  
	{

			
//		FrPsLinData_Parsing_Key(&LIN_RH_EVNT_MASTER_CMD);
		
		LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = 0;
		LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = 0;		
		LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = 0;
		LIN_RH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = 0;	

		lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);
		PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
	}
		if((Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01)) 
	{
	
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 0;
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 0;
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;

//		FrDrLinData_Parsing_Key(&LIN_LH_EVNT_MASTER_CMD);

		lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);
		PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
	} 
}

else if(button_LH_cycle_check == 0x03) //|| button_RH_cycle_check == 0x03) //|| button_RH_cycle_check == 0x03)
{
	if((Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01)) 
	{
	
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = 0;
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = 0;
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = 0;
		LIN_LH_EVNT_MASTER_CMD.LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = 0;

//		FrDrLinData_Parsing_Key(&LIN_LH_EVNT_MASTER_CMD);

		lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);
		PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
	} 
}
else if(button_RH_cycle_check == 0x03)
{
	if((Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01))  
	{

			
//		FrPsLinData_Parsing_Key(&LIN_RH_EVNT_MASTER_CMD);
		
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
		if((Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01)) 
		{
			FrDrLinData_Parsing(&LIN_LH_EVNT_MASTER_CMD);
			lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);
			PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
		}
	
		if((Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01))	
		{
			FrPsLinData_Parsing(&LIN_RH_EVNT_MASTER_CMD);
			lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);
			PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
			PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
		}
}

else if(button_LH_cycle_check == 0x04)
{
	if((Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_dr_Vent_Side_signal == 0x01) || (Hu_Fr_Dr_Vent_Mode == FREE_MODE && Hu_Fr_dr_Vent_Center_signal == 0x01)) 
	{
		FrDrLinData_Parsing(&LIN_LH_EVNT_MASTER_CMD);
		lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);
		PINS_DRV_ClearPins(PTE,LH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTB,LH_AVOID_INDI_PWM_MASK);
	}
}

else if(button_RH_cycle_check == 0x04)
{

	if((Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Side_signal == 0x01) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE && Hu_Fr_Ps_Vent_Center_signal == 0x01))  
	{
		FrPsLinData_Parsing(&LIN_RH_EVNT_MASTER_CMD);
		lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);
		PINS_DRV_ClearPins(PTB,RH_FACING_INDI_PWM_MASK);
		PINS_DRV_ClearPins(PTC,RH_AVOID_INDI_PWM_MASK);
	}
}

}



void HU_Rear_mode_setting()
{
	if((Hu_Rr_Dr_Vent_signal == 0x02) || (Hu_Rr_Dr_Vent_signal == 0x02))   //Full close mode setting
	{
		if(Hu_Rr_Dr_Vent_signal == 0x02)
		{
			//if(Hu_Rr_Dr_Vent_Mode == 0x00)
			//{
					lin_RrCycleMode_task(&LIN_RC_EVNT_MASTER_CMD);
			//}
		}
	
		if(Hu_Rr_Dr_Vent_signal == 0x02)
		{
			//if(Hu_Rr_Ps_Vent_Mode == 0x00)
			//{
				lin_RrVentMode_task(&LIN_RC_EVNT_MASTER_CMD);
			//}
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

		#if 0	
		if((Hu_Rr_Dr_Vent_Mode == FREE_MODE) || (Hu_Rr_Ps_Vent_Mode == FREE_MODE))
		{
			if(Hu_Rr_Dr_Vent_Mode == FREE_MODE)
			{

			}
	
			if(Hu_Rr_Ps_Vent_Mode == FREE_MODE)
			{
							
			}
		}
		#endif

}

//////////////////////////////////////////////////////////////////
//CTS
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

		#if 0
		if((Cts_Fr_Dr_Vent_Mode == FREE_MODE) || (Cts_Fr_Ps_Vent_Mode == FREE_MODE))
		{
			if(Cts_Fr_Dr_Vent_Mode == FREE_MODE)
			{

			}

			if(Cts_Fr_Ps_Vent_Mode == FREE_MODE)
			{

			}
		}
		#endif
}

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
	#if 0
	if((Cts_Rr_Dr_Vent_Mode == FREE_MODE) || (Cts_Rr_Ps_Vent_Mode == FREE_MODE))
	{
		if(Cts_Rr_Dr_Vent_Mode == FREE_MODE)
		{

		}
	
		if(Cts_Rr_Ps_Vent_Mode == FREE_MODE)
		{
								
		}							
	}
	#endif
}

#ifdef AMO_NVM_SETTING
void Last_Mode_dr_fr_set_func(void)
{
	if(Hu_Fr_dr_Vent_Side_signal == 0x02 || Hu_Fr_dr_Vent_Center_signal == 0x02 || Hu_Fr_Ps_Vent_Side_signal == 0x02 || Hu_Fr_Ps_Vent_Center_signal == 0x02)
	{
		if(Hu_Fr_dr_Vent_Side_signal == 0x02)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x02;
			//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x02;
		}

		if(Hu_Fr_dr_Vent_Center_signal == 0x02)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x02;
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x02;
		}

		if(Hu_Fr_Ps_Vent_Side_signal == 0x02)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x02;
			//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x02;								
		}

		if(Hu_Fr_Ps_Vent_Center_signal == 0x02)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x00;															//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x02;
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x02;								
		}								
		//Evt_queue_add(DEVICE_P_OIP_FULL_CLOSE_CAN_LIN_EVENT);
		Hu_Front_mode_FullClose_setting();
		HU_Rear_mode_setting();		
	}


	if(Hu_Fr_dr_Vent_Side_signal == 0x01)
	{
		if(Hu_Fr_Dr_Vent_Mode == 0x00)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Hu_Fr_Dr_Vent_Mode_Remember;		//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
			//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
			Hu_Fr_Dr_Vent_Mode = Hu_Fr_Dr_Vent_Mode_Remember;
		}
	}

	if(Hu_Fr_dr_Vent_Center_signal == 0x01)
	{
		if(Hu_Fr_Dr_Vent_Mode == 0x00)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = Hu_Fr_Dr_Vent_Mode_Remember;		//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			//Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
			Hu_Fr_Dr_Vent_Mode = Hu_Fr_Dr_Vent_Mode_Remember;
		}
	}

	if(Hu_Fr_Ps_Vent_Side_signal == 0x01)
	{
		if(Hu_Fr_Ps_Vent_Mode == 0x00)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Hu_Fr_Ps_Vent_Mode_Remember;		//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
			//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
			Hu_Fr_Ps_Vent_Mode = Hu_Fr_Ps_Vent_Mode_Remember;
		}
	}				

	if(Hu_Fr_Ps_Vent_Center_signal == 0x01)
	{
		if(Hu_Fr_Ps_Vent_Mode == 0x00)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = Hu_Fr_Ps_Vent_Mode_Remember;		//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			//Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;
			Hu_Fr_Ps_Vent_Mode = Hu_Fr_Ps_Vent_Mode_Remember;
		}
	}		

	if((Hu_Fr_Dr_Vent_Mode == FACING_MODE) || (Hu_Fr_Ps_Vent_Mode == FACING_MODE))
	{
		if(Hu_Fr_Dr_Vent_Mode == FACING_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x01;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
			Hu_Fr_Dr_Vent_Mode_Remember = FACING_MODE;
		}

		if(Hu_Fr_Ps_Vent_Mode == FACING_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x01;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;		
			Hu_Fr_Ps_Vent_Mode_Remember = FACING_MODE;										
		}
		//Evt_queue_add(DEVICE_P_OIP_FOCUS_CAN_LIN_EVENT);	
		Hu_Front_mode_setting();
		HU_Rear_mode_setting();		
	}

	if((Hu_Fr_Dr_Vent_Mode == AVOID_MODE) || (Hu_Fr_Ps_Vent_Mode == AVOID_MODE))
	{
		if(Hu_Fr_Dr_Vent_Mode == AVOID_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x02;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
			Hu_Fr_Dr_Vent_Mode_Remember = AVOID_MODE;
		}

		if(Hu_Fr_Ps_Vent_Mode == AVOID_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x02;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;		
			Hu_Fr_Ps_Vent_Mode_Remember = AVOID_MODE;										
		}
		//Evt_queue_add(DEVICE_P_OIP_SPREAD_CAN_LIN_EVENT);
		Hu_Front_mode_setting();
		HU_Rear_mode_setting();

	}							

	if((Hu_Fr_Dr_Vent_Mode == SWING_MODE) || (Hu_Fr_Ps_Vent_Mode == SWING_MODE))
	{
		if(Hu_Fr_Dr_Vent_Mode == SWING_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x03;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
			Hu_Fr_Dr_Vent_Mode_Remember = SWING_MODE;
		}

		if(Hu_Fr_Ps_Vent_Mode == SWING_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x03;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;		
			Hu_Fr_Ps_Vent_Mode_Remember = SWING_MODE;										
		}
		//Evt_queue_add(DEVICE_P_OIP_CYCLE_CAN_LIN_EVENT);		
		Hu_Front_mode_setting();
		HU_Rear_mode_setting();		
	}	

	if((Hu_Fr_Dr_Vent_Mode == FREE_MODE) || (Hu_Fr_Ps_Vent_Mode == FREE_MODE))
	{
		if(Hu_Fr_Dr_Vent_Mode == FREE_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntModeSet = 0x04;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrDrEvntCtrOpnCls = 0x01;
			Hu_Fr_Dr_Vent_Mode_Remember = FREE_MODE;
			Touch_mode_Dr_set = HU_DRIVER_TOUCH;
			
			FrDrLinData_Parsing_Backup(&LIN_LH_EVNT_MASTER_CMD);
			lin_FrDrmanualMode_task(&LIN_LH_EVNT_MASTER_CMD);
						
		}

		if(Hu_Fr_Ps_Vent_Mode == FREE_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntModeSet = 0x04;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntSideOpnCls = 0x01;
			Can_Tx_Evnt_4.data.L_EVNT_FrPsEvntCtrOpnCls = 0x01;		
			Hu_Fr_Ps_Vent_Mode_Remember = FREE_MODE;
			Touch_mode_Ps_set = HU_PASSENGER_TOUCH;

			FrPsLinData_Parsing_Backup(&LIN_RH_EVNT_MASTER_CMD);
			lin_FrPsmanualMode_task(&LIN_RH_EVNT_MASTER_CMD);		
		}				

		//Evt_queue_add(DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT);

		#if 0
		if(P_Oip_Cts_Select == P_OIP_SELECT)
		{
			Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x01; //APPLY Duplucate Input Signal Prevention Control Status return
		}
		else
		{
			Can_Tx_Evnt_4.data.L_EVNT_EvntTouchInform_HU = 0x02; //NOT APPLY
		}
		#endif
	}

	if(((Hu_Fr_Dr_Vent_Mode_Remember == 0x00) && (Hu_Fr_dr_Vent_Side_signal != 0x02)) || ((Hu_Fr_Dr_Vent_Mode_Remember == 0x00) && (Hu_Fr_dr_Vent_Center_signal != 0x02)))
	{
		P_Oip_Cts_priority = 0;
	}

	if(((Hu_Fr_Ps_Vent_Mode_Remember == 0x00) && (Hu_Fr_Ps_Vent_Side_signal != 0x02)) || ((Hu_Fr_Ps_Vent_Mode_Remember == 0x00) && (Hu_Fr_Ps_Vent_Center_signal != 0x02)))
	{
		P_Oip_Cts_priority = 0;
	}				

	if(Hu_Fr_Dr_Vent_Mode == MODE_OFF)
	{
		P_Oip_Cts_priority = 0;
		Hu_Fr_Dr_Vent_Mode_Remember = MODE_OFF;
		//VENT MODE OFF
	}					
	if(Hu_Fr_Ps_Vent_Mode == MODE_OFF)
	{
		P_Oip_Cts_priority = 0;
		Hu_Fr_Ps_Vent_Mode_Remember = MODE_OFF;
		//VENT MODE OFF
	}
}

void Last_Mode_rear_dr_ps_set_func(void)
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
		}

		if(Hu_Rr_Ps_Vent_Mode == FREE_MODE)
		{
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntModeSet = 0x04;														//default=0, facing = 1, avoid =2, swing = 3, free = 4, off = 5
			Can_Tx_Evnt_4.data.L_EVNT_RrDrEvntConsOpnCls = 0x00;
			Can_Tx_Evnt_4.data.L_EVNT_RrPsEvntConsOpnCls = 0x00;
			Hu_Rr_Ps_Vent_Mode_Remember = 0x04;									
		}

		Evt_queue_add(DEVICE_P_OIP_MANUAL_CAN_LIN_EVENT);	
		#if 0
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
		#endif
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


void Last_Setting_Save(Profile_mode Profile_mode)
{
	nvmem_data_type nv;
	
	switch(Profile_mode)
	{
		#if 0
		case DEFAULT:
		{
			nv.default_nv.items.fr_dr_mode = Hu_Fr_Dr_Vent_Mode;
			nv.default_nv.items.fr_ps_mode = Hu_Fr_Ps_Vent_Mode;
			nv.default_nv.items.rear_dr_mode = Hu_Rr_Dr_Vent_Mode;
			nv.default_nv.items.rear_ps_mode = Hu_Rr_Ps_Vent_Mode;
			nv.default_nv.items.fr_dr_side_opncls = Hu_Fr_dr_Vent_Side_signal;
			nv.default_nv.items.fr_dr_ctr_opncls = Hu_Fr_dr_Vent_Center_signal;
			nv.default_nv.items.fr_ps_side_opncls = Hu_Fr_Ps_Vent_Side_signal;
			nv.default_nv.items.fr_ps_ctr_opncls = Hu_Fr_Ps_Vent_Center_signal;
			nv.default_nv.items.fr_dr_side_leftright_target = Fr_Dr_side_leftright_target_backup;
			nv.default_nv.items.fr_dr_side_updown_target = Fr_Dr_side_updown_target_backup;
			nv.default_nv.items.fr_dr_ctr_leftright_target = Fr_Dr_ctr_leftright_target_backup;
			nv.default_nv.items.fr_dr_ctr_updown_target = Fr_Dr_ctr_updown_target_backup;
			nv.default_nv.items.fr_ps_side_leftright_target = Fr_Ps_side_leftright_target_backup;
			nv.default_nv.items.fr_ps_side_updown_target = Fr_Ps_side_updown_target_backup;
			nv.default_nv.items.fr_ps_ctr_leftright_target = Fr_Ps_ctr_leftright_target_backup;
			nv.default_nv.items.fr_ps_ctr_updown_target = Fr_Ps_ctr_updown_target_backup;
			nv.default_nv.items.rr_dr_side_leftright_target = Rr_Dr_side_leftright_target_backup;
			nv.default_nv.items.rr_dr_side_updown_target = Rr_Dr_side_updown_targe_backup;
			nv.default_nv.items.rr_ps_ctr_leftright_target = Rr_Ps_ctr_leftright_target_backup;
			nv.default_nv.items.rr_ps_ctr_updown_target = Rr_Ps_ctr_updown_target_backup;
			nv.default_nv.items.hu_fr_dr_sidept_x = Hu_FrDrEVntSidePt_X;
			nv.default_nv.items.hu_fr_dr_sidept_y = Hu_FrDrEVntSidePt_Y;
			nv.default_nv.items.hu_fr_dr_ctrpt_x = Hu_FrDrEVntCtrPt_X;
			nv.default_nv.items.hu_fr_dr_ctrpt_y = Hu_FrDrEVntCtrPt_Y;
			nv.default_nv.items.hu_fr_ps_sidept_x = Hu_FrPsEVntSidePt_X;
			nv.default_nv.items.hu_fr_ps_sidept_y = Hu_FrPsEVntSidePt_Y;
			nv.default_nv.items.hu_fr_ps_ctrpt_x = Hu_FrPsEVntCtrPt_X;
			nv.default_nv.items.hu_fr_ps_ctrpt_y = Hu_FrPsEVntCtrPt_Y;
			nv.default_nv.items.hu_rr_dr_constpt_x = Hu_RrDrEVntConsPt_X;
			nv.default_nv.items.hu_rr_dr_constpt_y = Hu_RrDrEVntConsPt_Y;
			nv.default_nv.items.hu_rr_ps_constpt_x = Hu_RrPsEVntConsPt_X;
			nv.default_nv.items.hu_rr_ps_constpt_x = Hu_RrPsEVntConsPt_Y;			
			nvmem_write(NVMEM_DEFAULT_NV_ID_IDX,&nv,58);	
			break;
		}
		#endif
		case GUEST:
		{
			nv.guest_nv.items.fr_dr_mode = Hu_Fr_Dr_Vent_Mode;
			nv.guest_nv.items.fr_ps_mode = Hu_Fr_Ps_Vent_Mode;
			nv.guest_nv.items.rear_dr_mode = Hu_Rr_Dr_Vent_Mode;
			nv.guest_nv.items.rear_ps_mode = Hu_Rr_Ps_Vent_Mode;
			nv.guest_nv.items.fr_dr_side_opncls = Hu_Fr_dr_Vent_Side_signal;
			nv.guest_nv.items.fr_dr_ctr_opncls = Hu_Fr_dr_Vent_Center_signal;
			nv.guest_nv.items.fr_ps_side_opncls = Hu_Fr_Ps_Vent_Side_signal;
			nv.guest_nv.items.fr_ps_ctr_opncls = Hu_Fr_Ps_Vent_Center_signal;
			nv.guest_nv.items.fr_dr_side_leftright_target = Fr_Dr_side_leftright_target_backup;
			nv.guest_nv.items.fr_dr_side_updown_target = Fr_Dr_side_updown_target_backup;
			nv.guest_nv.items.fr_dr_ctr_leftright_target = Fr_Dr_ctr_leftright_target_backup;
			nv.guest_nv.items.fr_dr_ctr_updown_target = Fr_Dr_ctr_updown_target_backup;
			nv.guest_nv.items.fr_ps_side_leftright_target = Fr_Ps_side_leftright_target_backup;
			nv.guest_nv.items.fr_ps_side_updown_target = Fr_Ps_side_updown_target_backup;
			nv.guest_nv.items.fr_ps_ctr_leftright_target = Fr_Ps_ctr_leftright_target_backup;
			nv.guest_nv.items.fr_ps_ctr_updown_target = Fr_Ps_ctr_updown_target_backup;
			nv.guest_nv.items.rr_dr_side_leftright_target = Rr_Dr_side_leftright_target_backup;
			nv.guest_nv.items.rr_dr_side_updown_target = Rr_Dr_side_updown_targe_backup;
			nv.guest_nv.items.rr_ps_ctr_leftright_target = Rr_Ps_ctr_leftright_target_backup;
			nv.guest_nv.items.rr_ps_ctr_updown_target = Rr_Ps_ctr_updown_target_backup;
			nv.guest_nv.items.hu_fr_dr_sidept_x = Hu_FrDrEVntSidePt_X;
			nv.guest_nv.items.hu_fr_dr_sidept_y = Hu_FrDrEVntSidePt_Y;
			nv.guest_nv.items.hu_fr_dr_ctrpt_x = Hu_FrDrEVntCtrPt_X;
			nv.guest_nv.items.hu_fr_dr_ctrpt_y = Hu_FrDrEVntCtrPt_Y;
			nv.guest_nv.items.hu_fr_ps_sidept_x = Hu_FrPsEVntSidePt_X;
			nv.guest_nv.items.hu_fr_ps_sidept_y = Hu_FrPsEVntSidePt_Y;
			nv.guest_nv.items.hu_fr_ps_ctrpt_x = Hu_FrPsEVntCtrPt_X;
			nv.guest_nv.items.hu_fr_ps_ctrpt_y = Hu_FrPsEVntCtrPt_Y;
			nv.guest_nv.items.hu_rr_dr_constpt_x = Hu_RrDrEVntConsPt_X;
			nv.guest_nv.items.hu_rr_dr_constpt_y = Hu_RrDrEVntConsPt_Y;
			nv.guest_nv.items.hu_rr_ps_constpt_x = Hu_RrPsEVntConsPt_X;
			nv.guest_nv.items.hu_rr_ps_constpt_x = Hu_RrPsEVntConsPt_Y;			
			nvmem_write(NVMEM_GUEST_NV_ID_IDX,&nv,58);		
			break;
		}
		case PROFILE_1:
		{
			nv.profile_1_nv.items.fr_dr_mode = Hu_Fr_Dr_Vent_Mode;
			nv.profile_1_nv.items.fr_ps_mode = Hu_Fr_Ps_Vent_Mode;
			nv.profile_1_nv.items.rear_dr_mode = Hu_Rr_Dr_Vent_Mode;
			nv.profile_1_nv.items.rear_ps_mode = Hu_Rr_Ps_Vent_Mode;
			nv.profile_1_nv.items.fr_dr_side_opncls = Hu_Fr_dr_Vent_Side_signal;
			nv.profile_1_nv.items.fr_dr_ctr_opncls = Hu_Fr_dr_Vent_Center_signal;
			nv.profile_1_nv.items.fr_ps_side_opncls = Hu_Fr_Ps_Vent_Side_signal;
			nv.profile_1_nv.items.fr_ps_ctr_opncls = Hu_Fr_Ps_Vent_Center_signal;
			nv.profile_1_nv.items.fr_dr_side_leftright_target = Fr_Dr_side_leftright_target_backup;
			nv.profile_1_nv.items.fr_dr_side_updown_target = Fr_Dr_side_updown_target_backup;
			nv.profile_1_nv.items.fr_dr_ctr_leftright_target = Fr_Dr_ctr_leftright_target_backup;
			nv.profile_1_nv.items.fr_dr_ctr_updown_target = Fr_Dr_ctr_updown_target_backup;
			nv.profile_1_nv.items.fr_ps_side_leftright_target = Fr_Ps_side_leftright_target_backup;
			nv.profile_1_nv.items.fr_ps_side_updown_target = Fr_Ps_side_updown_target_backup;
			nv.profile_1_nv.items.fr_ps_ctr_leftright_target = Fr_Ps_ctr_leftright_target_backup;
			nv.profile_1_nv.items.fr_ps_ctr_updown_target = Fr_Ps_ctr_updown_target_backup;
			nv.profile_1_nv.items.rr_dr_side_leftright_target = Rr_Dr_side_leftright_target_backup;
			nv.profile_1_nv.items.rr_dr_side_updown_target = Rr_Dr_side_updown_targe_backup;
			nv.profile_1_nv.items.rr_ps_ctr_leftright_target = Rr_Ps_ctr_leftright_target_backup;
			nv.profile_1_nv.items.rr_ps_ctr_updown_target = Rr_Ps_ctr_updown_target_backup;
			nv.profile_1_nv.items.hu_fr_dr_sidept_x = Hu_FrDrEVntSidePt_X;
			nv.profile_1_nv.items.hu_fr_dr_sidept_y = Hu_FrDrEVntSidePt_Y;
			nv.profile_1_nv.items.hu_fr_dr_ctrpt_x = Hu_FrDrEVntCtrPt_X;
			nv.profile_1_nv.items.hu_fr_dr_ctrpt_y = Hu_FrDrEVntCtrPt_Y;
			nv.profile_1_nv.items.hu_fr_ps_sidept_x = Hu_FrPsEVntSidePt_X;
			nv.profile_1_nv.items.hu_fr_ps_sidept_y = Hu_FrPsEVntSidePt_Y;
			nv.profile_1_nv.items.hu_fr_ps_ctrpt_x = Hu_FrPsEVntCtrPt_X;
			nv.profile_1_nv.items.hu_fr_ps_ctrpt_y = Hu_FrPsEVntCtrPt_Y;
			nv.profile_1_nv.items.hu_rr_dr_constpt_x = Hu_RrDrEVntConsPt_X;
			nv.profile_1_nv.items.hu_rr_dr_constpt_y = Hu_RrDrEVntConsPt_Y;
			nv.profile_1_nv.items.hu_rr_ps_constpt_x = Hu_RrPsEVntConsPt_X;
			nv.profile_1_nv.items.hu_rr_ps_constpt_x = Hu_RrPsEVntConsPt_Y;			
			nvmem_write(NVMEM_PROFILE_1_NV_ID_IDX,&nv,58);		
			break;
		}		
		case PROFILE_2:
		{
			nv.profile_2_nv.items.fr_dr_mode = Hu_Fr_Dr_Vent_Mode;
			nv.profile_2_nv.items.fr_ps_mode = Hu_Fr_Ps_Vent_Mode;
			nv.profile_2_nv.items.rear_dr_mode = Hu_Rr_Dr_Vent_Mode;
			nv.profile_2_nv.items.rear_ps_mode = Hu_Rr_Ps_Vent_Mode;
			nv.profile_2_nv.items.fr_dr_side_opncls = Hu_Fr_dr_Vent_Side_signal;
			nv.profile_2_nv.items.fr_dr_ctr_opncls = Hu_Fr_dr_Vent_Center_signal;
			nv.profile_2_nv.items.fr_ps_side_opncls = Hu_Fr_Ps_Vent_Side_signal;
			nv.profile_2_nv.items.fr_ps_ctr_opncls = Hu_Fr_Ps_Vent_Center_signal;
			nv.profile_2_nv.items.fr_dr_side_leftright_target = Fr_Dr_side_leftright_target_backup;
			nv.profile_2_nv.items.fr_dr_side_updown_target = Fr_Dr_side_updown_target_backup;
			nv.profile_2_nv.items.fr_dr_ctr_leftright_target = Fr_Dr_ctr_leftright_target_backup;
			nv.profile_2_nv.items.fr_dr_ctr_updown_target = Fr_Dr_ctr_updown_target_backup;
			nv.profile_2_nv.items.fr_ps_side_leftright_target = Fr_Ps_side_leftright_target_backup;
			nv.profile_2_nv.items.fr_ps_side_updown_target = Fr_Ps_side_updown_target_backup;
			nv.profile_2_nv.items.fr_ps_ctr_leftright_target = Fr_Ps_ctr_leftright_target_backup;
			nv.profile_2_nv.items.fr_ps_ctr_updown_target = Fr_Ps_ctr_updown_target_backup;
			nv.profile_2_nv.items.rr_dr_side_leftright_target = Rr_Dr_side_leftright_target_backup;
			nv.profile_2_nv.items.rr_dr_side_updown_target = Rr_Dr_side_updown_targe_backup;
			nv.profile_2_nv.items.rr_ps_ctr_leftright_target = Rr_Ps_ctr_leftright_target_backup;
			nv.profile_2_nv.items.rr_ps_ctr_updown_target = Rr_Ps_ctr_updown_target_backup;
			nv.profile_2_nv.items.hu_fr_dr_sidept_x = Hu_FrDrEVntSidePt_X;
			nv.profile_2_nv.items.hu_fr_dr_sidept_y = Hu_FrDrEVntSidePt_Y;
			nv.profile_2_nv.items.hu_fr_dr_ctrpt_x = Hu_FrDrEVntCtrPt_X;
			nv.profile_2_nv.items.hu_fr_dr_ctrpt_y = Hu_FrDrEVntCtrPt_Y;
			nv.profile_2_nv.items.hu_fr_ps_sidept_x = Hu_FrPsEVntSidePt_X;
			nv.profile_2_nv.items.hu_fr_ps_sidept_y = Hu_FrPsEVntSidePt_Y;
			nv.profile_2_nv.items.hu_fr_ps_ctrpt_x = Hu_FrPsEVntCtrPt_X;
			nv.profile_2_nv.items.hu_fr_ps_ctrpt_y = Hu_FrPsEVntCtrPt_Y;
			nv.profile_2_nv.items.hu_rr_dr_constpt_x = Hu_RrDrEVntConsPt_X;
			nv.profile_2_nv.items.hu_rr_dr_constpt_y = Hu_RrDrEVntConsPt_Y;
			nv.profile_2_nv.items.hu_rr_ps_constpt_x = Hu_RrPsEVntConsPt_X;
			nv.profile_2_nv.items.hu_rr_ps_constpt_x = Hu_RrPsEVntConsPt_Y;			
			nvmem_write(NVMEM_PROFILE_2_NV_ID_IDX,&nv,58);		
			break;
		}		
		default:
		break;
	}
}

void Last_Setting_Setup(Profile_mode Profile_mode)
{
	nvmem_data_type nv;

	printf("Last_Setting_Setup = %d \r\n", Profile_mode);
	switch(Profile_mode)
	{
		#if 0
		case DEFAULT:
		{
			nvmem_read(NVMEM_DEFAULT_NV_ID_IDX, &nv);
			
			Hu_Fr_Dr_Vent_Mode = nv.default_nv.items.fr_dr_mode;
			Hu_Fr_Ps_Vent_Mode = nv.default_nv.items.fr_ps_mode;
			Hu_Rr_Dr_Vent_Mode = nv.default_nv.items.rear_dr_mode;
			Hu_Rr_Ps_Vent_Mode = nv.default_nv.items.rear_ps_mode;
			Hu_Fr_dr_Vent_Side_signal = nv.default_nv.items.fr_dr_side_opncls;
			Hu_Fr_dr_Vent_Center_signal = nv.default_nv.items.fr_dr_ctr_opncls;
			Hu_Fr_Ps_Vent_Side_signal = nv.default_nv.items.fr_ps_side_opncls;
			Hu_Fr_Ps_Vent_Center_signal = nv.default_nv.items.fr_ps_ctr_opncls;
			Fr_Dr_side_leftright_target_backup = nv.default_nv.items.fr_dr_side_leftright_target;
			Fr_Dr_side_updown_target_backup = nv.default_nv.items.fr_dr_side_updown_target;
			Fr_Dr_ctr_leftright_target_backup = nv.default_nv.items.fr_dr_ctr_leftright_target;
			Fr_Dr_ctr_updown_target_backup = nv.default_nv.items.fr_dr_ctr_updown_target;
			Fr_Ps_side_leftright_target_backup = nv.default_nv.items.fr_ps_side_leftright_target;
			Fr_Ps_side_updown_target_backup = nv.default_nv.items.fr_ps_side_updown_target;
			Fr_Ps_ctr_leftright_target_backup = nv.default_nv.items.fr_ps_ctr_leftright_target;
			Fr_Ps_ctr_updown_target_backup = nv.default_nv.items.fr_ps_ctr_updown_target;
			Rr_Dr_side_leftright_target_backup = nv.default_nv.items.rr_dr_side_leftright_target;
			Rr_Dr_side_updown_targe_backup = nv.default_nv.items.rr_dr_side_updown_target;
			Rr_Ps_ctr_leftright_target_backup = nv.default_nv.items.rr_ps_ctr_leftright_target;
			Rr_Ps_ctr_updown_target_backup = nv.default_nv.items.rr_ps_ctr_updown_target;
			Hu_FrDrEVntSidePt_X = nv.default_nv.items.hu_fr_dr_sidept_x;
			Hu_FrDrEVntSidePt_Y = nv.default_nv.items.hu_fr_dr_sidept_y;
			Hu_FrDrEVntCtrPt_X = nv.default_nv.items.hu_fr_dr_ctrpt_x;
			Hu_FrDrEVntCtrPt_Y = nv.default_nv.items.hu_fr_dr_ctrpt_y;
			Hu_FrPsEVntSidePt_X = nv.default_nv.items.hu_fr_ps_sidept_x;
			Hu_FrPsEVntSidePt_Y = nv.default_nv.items.hu_fr_ps_sidept_y;
			Hu_FrPsEVntCtrPt_X = nv.default_nv.items.hu_fr_ps_ctrpt_x;
			Hu_FrPsEVntCtrPt_Y = nv.default_nv.items.hu_fr_ps_ctrpt_y;
			Hu_RrDrEVntConsPt_X = nv.default_nv.items.hu_rr_dr_constpt_x;
			Hu_RrDrEVntConsPt_Y = nv.default_nv.items.hu_rr_dr_constpt_y;
			Hu_RrPsEVntConsPt_X = nv.default_nv.items.hu_rr_ps_constpt_x;
			Hu_RrPsEVntConsPt_Y = nv.default_nv.items.hu_rr_ps_constpt_x;
			
			printf("%d %d \r\n", Hu_Fr_Dr_Vent_Mode, Hu_Fr_Ps_Vent_Mode);
			Last_Mode_dr_fr_set_func();
			break;
		}
		#endif
		case GUEST:
		{
			nvmem_read(NVMEM_GUEST_NV_ID_IDX, &nv);
			
			Hu_Fr_Dr_Vent_Mode = nv.guest_nv.items.fr_dr_mode;
			Hu_Fr_Ps_Vent_Mode = nv.guest_nv.items.fr_ps_mode;
			Hu_Rr_Dr_Vent_Mode = nv.guest_nv.items.rear_dr_mode;
			Hu_Rr_Ps_Vent_Mode = nv.guest_nv.items.rear_ps_mode;
			Hu_Fr_dr_Vent_Side_signal = nv.guest_nv.items.fr_dr_side_opncls;
			Hu_Fr_dr_Vent_Center_signal = nv.guest_nv.items.fr_dr_ctr_opncls;
			Hu_Fr_Ps_Vent_Side_signal = nv.guest_nv.items.fr_ps_side_opncls;
			Hu_Fr_Ps_Vent_Center_signal = nv.guest_nv.items.fr_ps_ctr_opncls;
			Fr_Dr_side_leftright_target_backup = nv.guest_nv.items.fr_dr_side_leftright_target;
			Fr_Dr_side_updown_target_backup = nv.guest_nv.items.fr_dr_side_updown_target;
			Fr_Dr_ctr_leftright_target_backup = nv.guest_nv.items.fr_dr_ctr_leftright_target;
			Fr_Dr_ctr_updown_target_backup = nv.guest_nv.items.fr_dr_ctr_updown_target;
			Fr_Ps_side_leftright_target_backup = nv.guest_nv.items.fr_ps_side_leftright_target;
			Fr_Ps_side_updown_target_backup = nv.guest_nv.items.fr_ps_side_updown_target;
			Fr_Ps_ctr_leftright_target_backup = nv.guest_nv.items.fr_ps_ctr_leftright_target;
			Fr_Ps_ctr_updown_target_backup = nv.guest_nv.items.fr_ps_ctr_updown_target;
			Rr_Dr_side_leftright_target_backup = nv.guest_nv.items.rr_dr_side_leftright_target;
			Rr_Dr_side_updown_targe_backup = nv.guest_nv.items.rr_dr_side_updown_target;
			Rr_Ps_ctr_leftright_target_backup = nv.guest_nv.items.rr_ps_ctr_leftright_target;
			Rr_Ps_ctr_updown_target_backup = nv.guest_nv.items.rr_ps_ctr_updown_target;
			Hu_FrDrEVntSidePt_X = nv.guest_nv.items.hu_fr_dr_sidept_x;
			Hu_FrDrEVntSidePt_Y = nv.guest_nv.items.hu_fr_dr_sidept_y;
			Hu_FrDrEVntCtrPt_X = nv.guest_nv.items.hu_fr_dr_ctrpt_x;
			Hu_FrDrEVntCtrPt_Y = nv.guest_nv.items.hu_fr_dr_ctrpt_y;
			Hu_FrPsEVntSidePt_X = nv.guest_nv.items.hu_fr_ps_sidept_x;
			Hu_FrPsEVntSidePt_Y = nv.guest_nv.items.hu_fr_ps_sidept_y;
			Hu_FrPsEVntCtrPt_X = nv.guest_nv.items.hu_fr_ps_ctrpt_x;
			Hu_FrPsEVntCtrPt_Y = nv.guest_nv.items.hu_fr_ps_ctrpt_y;
			Hu_RrDrEVntConsPt_X = nv.guest_nv.items.hu_rr_dr_constpt_x;
			Hu_RrDrEVntConsPt_Y = nv.guest_nv.items.hu_rr_dr_constpt_y;
			Hu_RrPsEVntConsPt_X = nv.guest_nv.items.hu_rr_ps_constpt_x;
			Hu_RrPsEVntConsPt_Y = nv.guest_nv.items.hu_rr_ps_constpt_x;
			Last_Mode_dr_fr_set_func();
			break;
		}
		case PROFILE_1:
		{
			nvmem_read(NVMEM_PROFILE_1_NV_ID_IDX, &nv);
			
			Hu_Fr_Dr_Vent_Mode = nv.profile_1_nv.items.fr_dr_mode;
			Hu_Fr_Ps_Vent_Mode = nv.profile_1_nv.items.fr_ps_mode;
			Hu_Rr_Dr_Vent_Mode = nv.profile_1_nv.items.rear_dr_mode;
			Hu_Rr_Ps_Vent_Mode = nv.profile_1_nv.items.rear_ps_mode;
			Hu_Fr_dr_Vent_Side_signal = nv.profile_1_nv.items.fr_dr_side_opncls;
			Hu_Fr_dr_Vent_Center_signal = nv.profile_1_nv.items.fr_dr_ctr_opncls;
			Hu_Fr_Ps_Vent_Side_signal = nv.profile_1_nv.items.fr_ps_side_opncls;
			Hu_Fr_Ps_Vent_Center_signal = nv.profile_1_nv.items.fr_ps_ctr_opncls;
			Fr_Dr_side_leftright_target_backup = nv.profile_1_nv.items.fr_dr_side_leftright_target;
			Fr_Dr_side_updown_target_backup = nv.profile_1_nv.items.fr_dr_side_updown_target;
			Fr_Dr_ctr_leftright_target_backup = nv.profile_1_nv.items.fr_dr_ctr_leftright_target;
			Fr_Dr_ctr_updown_target_backup = nv.profile_1_nv.items.fr_dr_ctr_updown_target;
			Fr_Ps_side_leftright_target_backup = nv.profile_1_nv.items.fr_ps_side_leftright_target;
			Fr_Ps_side_updown_target_backup = nv.profile_1_nv.items.fr_ps_side_updown_target;
			Fr_Ps_ctr_leftright_target_backup = nv.profile_1_nv.items.fr_ps_ctr_leftright_target;
			Fr_Ps_ctr_updown_target_backup = nv.profile_1_nv.items.fr_ps_ctr_updown_target;
			Rr_Dr_side_leftright_target_backup = nv.profile_1_nv.items.rr_dr_side_leftright_target;
			Rr_Dr_side_updown_targe_backup = nv.profile_1_nv.items.rr_dr_side_updown_target;
			Rr_Ps_ctr_leftright_target_backup = nv.profile_1_nv.items.rr_ps_ctr_leftright_target;
			Rr_Ps_ctr_updown_target_backup = nv.profile_1_nv.items.rr_ps_ctr_updown_target;
			Hu_FrDrEVntSidePt_X = nv.profile_1_nv.items.hu_fr_dr_sidept_x;
			Hu_FrDrEVntSidePt_Y = nv.profile_1_nv.items.hu_fr_dr_sidept_y;
			Hu_FrDrEVntCtrPt_X = nv.profile_1_nv.items.hu_fr_dr_ctrpt_x;
			Hu_FrDrEVntCtrPt_Y = nv.profile_1_nv.items.hu_fr_dr_ctrpt_y;
			Hu_FrPsEVntSidePt_X = nv.profile_1_nv.items.hu_fr_ps_sidept_x;
			Hu_FrPsEVntSidePt_Y = nv.profile_1_nv.items.hu_fr_ps_sidept_y;
			Hu_FrPsEVntCtrPt_X = nv.profile_1_nv.items.hu_fr_ps_ctrpt_x;
			Hu_FrPsEVntCtrPt_Y = nv.profile_1_nv.items.hu_fr_ps_ctrpt_y;
			Hu_RrDrEVntConsPt_X = nv.profile_1_nv.items.hu_rr_dr_constpt_x;
			Hu_RrDrEVntConsPt_Y = nv.profile_1_nv.items.hu_rr_dr_constpt_y;
			Hu_RrPsEVntConsPt_X = nv.profile_1_nv.items.hu_rr_ps_constpt_x;
			Hu_RrPsEVntConsPt_Y = nv.profile_1_nv.items.hu_rr_ps_constpt_x;				
			Last_Mode_dr_fr_set_func();
			break;
		}		

		case PROFILE_2:
		{
			nvmem_read(NVMEM_PROFILE_2_NV_ID_IDX, &nv);
			
			Hu_Fr_Dr_Vent_Mode = nv.profile_2_nv.items.fr_dr_mode;
			Hu_Fr_Ps_Vent_Mode = nv.profile_2_nv.items.fr_ps_mode;
			Hu_Rr_Dr_Vent_Mode = nv.profile_2_nv.items.rear_dr_mode;
			Hu_Rr_Ps_Vent_Mode = nv.profile_2_nv.items.rear_ps_mode;
			Hu_Fr_dr_Vent_Side_signal = nv.profile_2_nv.items.fr_dr_side_opncls;
			Hu_Fr_dr_Vent_Center_signal = nv.profile_2_nv.items.fr_dr_ctr_opncls;
			Hu_Fr_Ps_Vent_Side_signal = nv.profile_2_nv.items.fr_ps_side_opncls;
			Hu_Fr_Ps_Vent_Center_signal = nv.profile_2_nv.items.fr_ps_ctr_opncls;
			Fr_Dr_side_leftright_target_backup = nv.profile_2_nv.items.fr_dr_side_leftright_target;
			Fr_Dr_side_updown_target_backup = nv.profile_2_nv.items.fr_dr_side_updown_target;
			Fr_Dr_ctr_leftright_target_backup = nv.profile_2_nv.items.fr_dr_ctr_leftright_target;
			Fr_Dr_ctr_updown_target_backup = nv.profile_2_nv.items.fr_dr_ctr_updown_target;
			Fr_Ps_side_leftright_target_backup = nv.profile_2_nv.items.fr_ps_side_leftright_target;
			Fr_Ps_side_updown_target_backup = nv.profile_2_nv.items.fr_ps_side_updown_target;
			Fr_Ps_ctr_leftright_target_backup = nv.profile_2_nv.items.fr_ps_ctr_leftright_target;
			Fr_Ps_ctr_updown_target_backup = nv.profile_2_nv.items.fr_ps_ctr_updown_target;
			Rr_Dr_side_leftright_target_backup = nv.profile_2_nv.items.rr_dr_side_leftright_target;
			Rr_Dr_side_updown_targe_backup = nv.profile_2_nv.items.rr_dr_side_updown_target;
			Rr_Ps_ctr_leftright_target_backup = nv.profile_2_nv.items.rr_ps_ctr_leftright_target;
			Rr_Ps_ctr_updown_target_backup = nv.profile_2_nv.items.rr_ps_ctr_updown_target;
			Hu_FrDrEVntSidePt_X = nv.profile_2_nv.items.hu_fr_dr_sidept_x;
			Hu_FrDrEVntSidePt_Y = nv.profile_2_nv.items.hu_fr_dr_sidept_y;
			Hu_FrDrEVntCtrPt_X = nv.profile_2_nv.items.hu_fr_dr_ctrpt_x;
			Hu_FrDrEVntCtrPt_Y = nv.profile_2_nv.items.hu_fr_dr_ctrpt_y;
			Hu_FrPsEVntSidePt_X = nv.profile_2_nv.items.hu_fr_ps_sidept_x;
			Hu_FrPsEVntSidePt_Y = nv.profile_2_nv.items.hu_fr_ps_sidept_y;
			Hu_FrPsEVntCtrPt_X = nv.profile_2_nv.items.hu_fr_ps_ctrpt_x;
			Hu_FrPsEVntCtrPt_Y = nv.profile_2_nv.items.hu_fr_ps_ctrpt_y;
			Hu_RrDrEVntConsPt_X = nv.profile_2_nv.items.hu_rr_dr_constpt_x;
			Hu_RrDrEVntConsPt_Y = nv.profile_2_nv.items.hu_rr_dr_constpt_y;
			Hu_RrPsEVntConsPt_X = nv.profile_2_nv.items.hu_rr_ps_constpt_x;
			Hu_RrPsEVntConsPt_Y = nv.profile_2_nv.items.hu_rr_ps_constpt_x;				
			Last_Mode_dr_fr_set_func();
			break;
		}				
		default:
		break;
	}
}
#endif
///////////////////////////////////////////////
void flexcan0_ErrorCallback(uint8_t instance, flexcan_event_type_t eventType, flexcan_state_t *flexcanState)
{
	volatile uint32_t error;

	(void)flexcanState;
	(void)instance;

	switch(eventType)
	{
		case FLEXCAN_EVENT_ERROR:
		{
			callback_test |= 0x4; // set bit2 to to evidence error ISR hit

			error = FLEXCAN_DRV_GetErrorStatus(INST_CANCOM1);

			can_error_data = error;

			//if(can_error_data !=0)
			//{
			//	Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
			//}	
			#if 0
			if(error&0x4) // if BOFFINT was set
			{
				callback_test |= 0x8; // set bit3 to to evidence bus off ISR hit

				// abort TX MB, after bus off recovery message is not send
				FLEXCAN_DRV_AbortTransfer(INST_CANCOM1,TX_MAILBOX);
			}
			#endif
			if(error & FLEXCAN_BusOffFlag)
			{
				Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
			}
			if(error & FLEXCAN_ErrorPassiveFlag)
			{
				Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
			}
			if(error & FLEXCAN_StuffingErrorFlag)
			{
				Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
			}
			if(error & FLEXCAN_CrcErrorFlag)
			{
				Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
			}
			if(error & FLEXCAN_AckErrorFlag)
			{
				Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
			}
			if(error & FLEXCAN_FormErrorFlag)
			{
				Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
			}
			if(error & FLEXCAN_BitErrorFlag)
			{
				Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
			}				
		break;
		}
		default:
		break;
	}
}

void Amo_Can_Init()
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
	for(int i=0; i<24; i++)
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
	///* set individual masking type */
	//FLEXCAN_DRV_SetRxMaskType(INST_CANCOM1, FLEXCAN_RX_MASK_INDIVIDUAL);

	/* Start receiving data in RX_MAILBOX. */
	//FLEXCAN_DRV_Receive(INST_CANCOM1, RX_MAILBOX, &recvBuff1);
	/* Start receiving data in RX_RXFIFO. */
	FLEXCAN_DRV_RxFifo(INST_CANCOM1,&recvBuff2);

	Init_CanTx_Parameter();
}


#endif /* __UART_AMO_H */

