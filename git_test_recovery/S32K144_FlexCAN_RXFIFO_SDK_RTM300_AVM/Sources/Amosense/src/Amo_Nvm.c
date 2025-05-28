#ifndef AMO_NVM_C
#define AMO_NVM_C

/*===========================================================================*/
/* Project   :  AMOSENSE Flash driver Software                                                                */
/* File name :  Amo_Nvm.c                                                                             */
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
/* Header file for NVM.                                                                               */
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

typedef struct{
nvmem_idx_type id;
uint16_t save_size;
}nvmem_table_type;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
uint32_t Find_WriteSector(void);
void FLEXRAM_Clear(void);
void Eeprom_read(uint8_t *pData, uint32_t TarAddr, uint16_t NbByte);
void nvmem_find(void);
	
uint32_t set_address = 0;
uint8_t nv_page_cnt = 0;

#if 0
static const nvmem_table_type nvmem_table[] =
{
{NVMEM_WRITE_CNT_IDX,4},
{NVMEM_REGISTRATION_IDX,1},
{NVMEM_NV_INIT_IDX,1},
{NVMEM_PROFILE_ID_IDX,1},
{NVMEM_DRIVE_TYPE_ID_IDX,1},
{NVMEM_PROFILE_ACTIVE_IDX,11},
{NVMEM_DEFAULT_NV_ID_IDX,AMO_SETTING_BUF},
{NVMEM_GUEST_NV_ID_IDX,AMO_SETTING_BUF},
{NVMEM_PROFILE_1_NV_ID_IDX,AMO_SETTING_BUF},
{NVMEM_PROFILE_2_NV_ID_IDX,AMO_SETTING_BUF},
{NVMEM_PROFILE_3_NV_ID_IDX,AMO_SETTING_BUF},
{NVMEM_PROFILE_4_NV_ID_IDX,AMO_SETTING_BUF},
{NVMEM_PROFILE_5_NV_ID_IDX,AMO_SETTING_BUF},
{NVMEM_PROFILE_6_NV_ID_IDX,AMO_SETTING_BUF},
{NVMEM_PROFILE_7_NV_ID_IDX,AMO_SETTING_BUF},
{NVMEM_PROFILE_8_NV_ID_IDX,AMO_SETTING_BUF},
{NVMEM_PROFILE_9_NV_ID_IDX,AMO_SETTING_BUF}, //437 //435
{NVMEM_BATTCAL_IDX,2}, //439
{NVMEM_IGNCAL_IDX,2}, //441
#ifdef AMO_DTC_SETTING
{NVMEM_UDS_NV_ID_IDX,MAX_DTC_COUNT}, //636 //436
#endif
{NVMEM_CHECKSUM_IDX,1}, //442
};
#endif

/*
* @brief Eeprom_read function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Eeprom_read(uint8_t *pData, uint32_t TarAddr, uint16_t NbByte)
{
	uint8_t *temp_data = NULL;
	if(pData != NULL)
	{
		/* Target address setting */
		temp_data = (uint8_t *)TarAddr;
		memcpy(pData, &temp_data[0], NbByte);
	}
	else
	{
	
	}
}

/*
* @brief Find_WriteSector function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
uint32_t Find_WriteSector(void)
{
	nvmem_data_type nvwr;
	uint32_t address = 0;
	uint32_t min_write = WRITE_MAX;
	uint32_t find_page = 0;

	for (uint32_t i = 0; i < PAGE_COUNT; i++)
	{
		/* Target address setting */
		address = (EEPROM_START_ADDRESS+(i*PAGE_SIZE));
		Eeprom_read((uint8_t *)&nvwr, address, PAGE_SIZE); 	
		if(nvwr.nv_item.write_cnt < min_write)
		{
			min_write = nvwr.nv_item.write_cnt;
			find_page = i;
		}
	}
	return find_page;
}	

/*
* @brief Calculate_Chksum function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
uint8_t Calculate_Chksum(const uint8_t *data, unsigned int length)
{
	unsigned int total = 0;
	uint8_t checksum = 0;

	for (unsigned int i = 0; i < length; i++)
	{
		total += data[i];
	}
	/* Checksum setting */
	checksum = (uint8_t)(total & 0x000000FFu);
	return ~checksum;
}

/*
* @brief nvmem_find function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void nvmem_find(void)
{
	nvmem_data_type nvmem;
	uint32_t address = 0;
	uint32_t read_page = 0;
	uint32_t max_write = WRITE_MAX;

	/* Target address setting */
	address = EEPROM_START_ADDRESS;	
	Eeprom_read((uint8_t *)&nvmem, address, PAGE_SIZE);
	
	if(nvmem.nv_item.nvinit == NVMEM_VERSION_INFO)
	{
		for(unsigned int i=0; i < PAGE_COUNT; i++)
		{		
			address = (EEPROM_START_ADDRESS+(uint32_t)(i*PAGE_SIZE));

			Eeprom_read((uint8_t *)&nvmem, address, PAGE_SIZE);		
			if(nvmem.nv_item.write_cnt < max_write)
			{
				max_write = nvmem.nv_item.write_cnt;
				read_page = i;
			}
		}
	}
	else
	{
		read_page = 0;
	}

	set_address = EEPROM_START_ADDRESS + (read_page * PAGE_SIZE);
}

/*
* @brief nvmem_read function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
int nvmem_read(nvmem_data_type *pcBuffer)
{	
	nvmem_data_type nvmem;
	uint32_t address = 0;
	uint32_t read_page = 0;
	uint32_t max_write = 0;

	/* Target address setting */
	address = EEPROM_START_ADDRESS;
	
	Eeprom_read((uint8_t *)&nvmem, address, PAGE_SIZE);

	if(nvmem.nv_item.nvinit == NVMEM_VERSION_INFO)
	{
		for(unsigned int i=0; i < PAGE_COUNT; i++)
		{		
			address = (EEPROM_START_ADDRESS+(i*PAGE_SIZE));
			
			Eeprom_read((uint8_t *)&nvmem, address, PAGE_SIZE);		
			if(nvmem.nv_item.write_cnt > max_write)
			{
				max_write = nvmem.nv_item.write_cnt;
				read_page = i;
			}
		}
	}
	else
	{
		read_page = 0;
	}
	/* Target address setting */
	set_address = EEPROM_START_ADDRESS + (read_page * PAGE_SIZE);
	Eeprom_read((uint8_t *)pcBuffer, set_address, PAGE_SIZE);
	return 0;
}

/*
* @brief nvmem_write function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
status_t nvmem_write(const uint8_t    *nvBuffer, int n_size)
{
	status_t ret; 
	uint32_t NvPage = Find_WriteSector();
	nv_page_cnt = (uint8_t)NvPage;
	/* Target address setting */
	set_address = EEPROM_START_ADDRESS + (NvPage * PAGE_SIZE);
	ret = FLASH_DRV_EEEWrite(&flashSSDConfig, set_address, PAGE_SIZE, (const uint8_t *)nvBuffer);
	if(ret != STATUS_SUCCESS)
	{
		//fail
	}
	return ret;
}

/*
* @brief FLEXRAM_Clear function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void FLEXRAM_Clear(void)
{
	uint8_t *flexramAddress = (uint8_t *)EEPROM_START_ADDRESS;
	for(uint32_t i = 0; i < EEPROM_SIZE; i++)
	{
		/* Clear Ram Init */
		flexramAddress[i] = 0x00;
		while(((FTFC->FSTAT) & (FTFC_FSTAT_CCIF_MASK)) == 0)
		{
		}
	}
}

/*
* @brief nvmem_init_call function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void nvmem_init_call(void)
{
	nvmem_data_type nv;
	nvmem_read(&nv);

	nv.nv_item.write_cnt = 1;
	nv.nv_item.registration = 0;
	nv.nv_item.nvinit = NVMEM_VERSION_INFO;
	nv.nv_item.profile = 0x00;
	nv.nv_item.drive_type = 0x00;
	/* Driver mode setting */
	ATCU_DriverSideType = (Drv_type)nv.nv_item.drive_type;
	memset(nv.nv_item.profile_active, 0x00, sizeof(nv.nv_item.profile_active));

	for(int i=0; i<11; i++)
	{	
		nv.nv_item.profile_item[i].items.fr_dr_mode = 0x00;
		nv.nv_item.profile_item[i].items.fr_ps_mode = 0x00;
#ifdef AMO_GN7_PE_SETTING_NONE		
		nv.nv_item.profile_item[i].items.rear_dr_mode = 0x00;
		nv.nv_item.profile_item[i].items.rear_ps_mode = 0x00;
#endif		
		nv.nv_item.profile_item[i].items.fr_dr_side_opncls = 0x01;
		nv.nv_item.profile_item[i].items.fr_dr_ctr_opncls = 0x01;
		nv.nv_item.profile_item[i].items.fr_ps_side_opncls = 0x01;
		nv.nv_item.profile_item[i].items.fr_ps_ctr_opncls = 0x01;
#ifdef AMO_GN7_PE_SETTING_NONE
		nv.nv_item.profile_item[i].items.rr_dr_side_opncls = 0x01; 
		nv.nv_item.profile_item[i].items.rr_ps_ctr_opncls = 0x01;	
#endif
		nv.nv_item.profile_item[i].items.fr_dr_side_leftright_target = 0x00;
		nv.nv_item.profile_item[i].items.fr_dr_side_updown_target = 0x00;
		nv.nv_item.profile_item[i].items.fr_dr_ctr_leftright_target = 0x00;
		nv.nv_item.profile_item[i].items.fr_dr_ctr_updown_target = 0x00;
		nv.nv_item.profile_item[i].items.fr_ps_side_leftright_target = 0x00;
		nv.nv_item.profile_item[i].items.fr_ps_side_updown_target = 0x00;
		nv.nv_item.profile_item[i].items.fr_ps_ctr_leftright_target = 0x00;
		nv.nv_item.profile_item[i].items.fr_ps_ctr_updown_target = 0x00;
#ifdef AMO_GN7_PE_SETTING_NONE		
		nv.nv_item.profile_item[i].items.rr_dr_side_leftright_target = 0x00;
		nv.nv_item.profile_item[i].items.rr_dr_side_updown_target = 0x00;
		nv.nv_item.profile_item[i].items.rr_ps_ctr_leftright_target = 0x00;
		nv.nv_item.profile_item[i].items.rr_ps_ctr_updown_target = 0x00;
#endif		
		nv.nv_item.profile_item[i].items.hu_fr_dr_sidept_x = 0x00;
		nv.nv_item.profile_item[i].items.hu_fr_dr_sidept_y = 0x00;
		nv.nv_item.profile_item[i].items.hu_fr_dr_ctrpt_x = 0x00;
		nv.nv_item.profile_item[i].items.hu_fr_dr_ctrpt_y = 0x00;
		nv.nv_item.profile_item[i].items.hu_fr_ps_sidept_x = 0x00;
		nv.nv_item.profile_item[i].items.hu_fr_ps_sidept_y = 0x00;
		nv.nv_item.profile_item[i].items.hu_fr_ps_ctrpt_x = 0x00;
		nv.nv_item.profile_item[i].items.hu_fr_ps_ctrpt_y = 0x00;
#ifdef AMO_GN7_PE_SETTING_NONE		
		nv.nv_item.profile_item[i].items.hu_rr_dr_constpt_x = 0x00;
		nv.nv_item.profile_item[i].items.hu_rr_dr_constpt_y = 0x00;
		nv.nv_item.profile_item[i].items.hu_rr_ps_constpt_x = 0x00;
		nv.nv_item.profile_item[i].items.hu_rr_ps_constpt_x = 0x00;			
#endif
	}
	nv.nv_item.nv_battcal = 0;		//439
	nv.nv_item.nv_igncal = 0; 	//441

#ifdef AMO_DTC_SETTING	
	for(int i=0; i < MAX_DTC_COUNT; i++)
	{
		nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcCode = 0x000000;
		nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcStatus = 0x00;
		nv.nv_item.uds_dtc_nv.DtcRecord[i].dtcOccurence = 0x00; 		
	}
	nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 637); 	//436
#else
	nv.nv_item.nv_checksum = Calculate_Chksum(nv.nv_data, 441); 	//438
#endif
	nvmem_write(nv.nv_data, PAGE_SIZE); 	
}

/*
* @brief nvmem_init function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void nvmem_init(void)
{
	nvmem_data_type nv;
	nvmem_read(&nv);

	if(nv.nv_item.nvinit != NVMEM_VERSION_INFO)
	{
		FLEXRAM_Clear();			
		nvmem_init_call();	
	}
	else
	{
		bool nv_checksum_check = false;
		/* Driver mode setting */
		ATCU_DriverSideType = (Drv_type)nv.nv_item.drive_type;
#ifdef AMO_DTC_SETTING	
		nv_checksum_check = (nv.nv_item.nv_checksum == Calculate_Chksum(((uint8_t *)&nv.nv_data), 637));
#else
		nv_checksum_check = (nv.nv_item.nv_checksum == Calculate_Chksum(((uint8_t *)&nv.nv_data), 441));
#endif
#ifdef DEBUG_MODE
		printf("%ld %d %d %02x checksum check = %d \r\n", nv.nv_item.write_cnt, nv.nv_item.registration, nv.nv_item.nvinit, nv.nv_item.nv_checksum, nv_checksum_check);
#endif
		if(nv_checksum_check)
		{
			//return 0;
		}
		else
		{
			nvmem_init_call();
		}	
	}
	//return 0;
}

/*
* @brief nvmem_erase_all function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void nvmem_erase_all(void)
{

}
#endif
