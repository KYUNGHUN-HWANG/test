
#ifndef AMO_SYSTEM_H_
#define AMO_SYSTEM_H_

/*===========================================================================*/
/* Project   :  AMOSENSE System Software                                                                */
/* File name :  Amo_System_setting.h                                                                             */
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
/* Header file for sleep.                                                                               */
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

void BoardInit(void);
void LVD_SET(void);
void Handle_Manual_Mode(void);
void Handle_Full_close_Mode(void);
void Handle_Foucs_Mode(void);
void Handle_Spread_Mode(void);
void Handle_Cycle_Mode(void);
void Handle_Event_Choice(void);
void Handle_Last_Mode(void);
void Handle_Secure_Mode(void);
void Handle_Sleep_Main(void);
void Handle_Sleep_Enter(void);
void Manual_dr_evt_signal(void);
void Manual_ps_evt_signal(void);
#endif

