
#ifndef AMO_LASTMODE_H_
#define AMO_LASTMODE_H_

/*===========================================================================*/
/* Project   :  AMOSENSE LAST MODE Software                                                                */
/* File name :  Amo_Vent_mode.h                                                                             */
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

extern void Last_full_close_mode(void);
extern void Last_dr_foucs_mode(void);
extern void Last_ps_foucs_mode(void);
extern void Last_dr_spread_mode(void);
extern void Last_ps_spread_mode(void);
extern void Last_dr_cycle_mode(void);
extern void Last_ps_cycle_mode(void);
extern void Last_dr_manual_mode(void);
extern void Last_ps_manual_mode(void);
extern void Last_Mode_dr_fr_set_func(void);
extern void Last_Mode_dr_fr_set_fullclose_func(void);
#endif

