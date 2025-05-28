#ifndef AMO_FLASH_H
#define AMO_FLASH_H

/*===========================================================================*/
/* Project   :  AMOSENSE Flash driver Software                                                                */
/* File name :  Amo_Flash.h                                                                             */
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
/* Header file for ADC.                                                                               */
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

typedef union{
	struct{
	uint32_t temp_data;
	uint8_t check_sum;
	};
	uint8_t nv_data[6];
}Eeprom_data_t;

void Amo_Flash_Init(void);
extern flash_ssd_config_t flashSSDConfig;

#endif /*AMO_FLASH_H */


