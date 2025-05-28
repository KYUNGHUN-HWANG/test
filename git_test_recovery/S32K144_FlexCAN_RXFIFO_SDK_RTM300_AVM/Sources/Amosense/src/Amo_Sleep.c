#ifndef AMO_SLEEP_C
#define AMO_SLEEP_C

/*===========================================================================*/
/* Project   :  AMOSENSE Sleep function Software                                                                */
/* File name :  Amo_Sleep.c                                                                             */
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
/* Header file for Sleep.                                                                               */
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

/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Amo_main.h"

void Amo_Sleep(void);

void Amo_Sleep(void)
{
	//PINS_DRV_ClearPins(GPIO_PORTC, CAN_STB_MASK);
	Evt_queue_add(DEVICE_SLEEP_ENTER_EVENT);
}
#endif

