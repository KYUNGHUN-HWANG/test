#ifndef AMO_FLASH_C
#define AMO_FLASH_C

/*===========================================================================*/
/* Project   :  AMOSENSE Flash driver Software                                                                */
/* File name :  Amo_flash.c                                                                             */
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
/* Header file for FLASH.                                                                               */
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
#include "Amo_flash.h"
#include "Flash1.h"
#include "Amo_Nvm.h"

/* Declare a FLASH config struct which initialized by FlashInit, and will be used by all flash operations */
flash_ssd_config_t flashSSDConfig;

/* Function declarations */
void CCIF_Handler(void);
/*!
  \brief The main function for the project.
  \details The startup initialization sequence is the following:
 * - startup asm routine
 * - main()
*/
void Amo_Flash_Init(void)
{
	/* Write your local variable definition here */
	//status_t ret;
	INT_SYS_InstallHandler(FTFC_IRQn, CCIF_Handler, (isr_t*)NULL);
	INT_SYS_EnableIRQ(FTFC_IRQn);
	INT_SYS_EnableIRQGlobal();
	/* Always initialize the driver before calling other functions */
	FLASH_DRV_Init(&Flash1_InitConfig0, &flashSSDConfig);
	//DEV_ASSERT(STATUS_SUCCESS == ret);
#if ((FEATURE_FLS_HAS_FLEX_NVM == 1u) & (FEATURE_FLS_HAS_FLEX_RAM == 1u))
	/* Config FlexRAM as EEPROM if it is currently used as traditional RAM */
	if (flashSSDConfig.EEESize == 0u)
	{   				
		/* Configure FlexRAM as EEPROM and FlexNVM as EEPROM backup region,
		* DEFlashPartition will be failed if the IFR region isn't blank.
		* Refer to the device document for valid EEPROM Data Size Code
		* and FlexNVM Partition Code. For example on S32K144:
		* - EEEDataSizeCode = 0x02u: EEPROM size = 4 Kbytes
		* - DEPartitionCode = 0x08u: EEPROM backup size = 64 Kbytes */
		FLASH_DRV_DEFlashPartition(&flashSSDConfig, 0x02u, 0x08u, 0x0u, false, true);
		//DEV_ASSERT(STATUS_SUCCESS == ret);
		/* Re-initialize the driver to update the new EEPROM configuration */
		FLASH_DRV_Init(&Flash1_InitConfig0, &flashSSDConfig);
		//DEV_ASSERT(STATUS_SUCCESS == ret);

		/* Make FlexRAM available for EEPROM */
		FLASH_DRV_SetFlexRamFunction(&flashSSDConfig, EEE_ENABLE, 0x00u, NULL);
		//DEV_ASSERT(STATUS_SUCCESS == ret);		
	
	}
	else    /* FLexRAM is already configured as EEPROM */
	{
		/* Make FlexRAM available for EEPROM, make sure that FlexNVM and FlexRAM
		* are already partitioned successfully before */
		FLASH_DRV_SetFlexRamFunction(&flashSSDConfig, EEE_ENABLE, 0x00u, NULL);
		//DEV_ASSERT(STATUS_SUCCESS == ret);	
	}
#endif /* (FEATURE_FLS_HAS_FLEX_NVM == 1u) & (FEATURE_FLS_HAS_FLEX_RAM == 1u) */

	/* Try to write data to EEPROM if FlexRAM is configured as EEPROM */
	if (flashSSDConfig.EEESize != 0u)
	{
	
	}

	/*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
}

/*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/

/*!
  \brief Interrupt handler for Flash Command Complete event
*/
void CCIF_Handler(void)
{
    /* Disable Flash Command Complete interrupt */
    FTFx_FCNFG &= (~FTFx_FCNFG_CCIE_MASK);

    return;
}
#endif /* AMO_FLASH_C */

