#ifndef MO_NVM_H_
#define MO_NVM_H_

/*===========================================================================*/
/* Project   :  AMOSENSE Flash driver Software                                                                */
/* File name :  Amo_Nvm.h                                                                             */
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
#include "Amo_main.h"

typedef enum{
NVMEM_WRITE_CNT_IDX,
NVMEM_REGISTRATION_IDX,
NVMEM_NV_INIT_IDX,
NVMEM_PROFILE_ID_IDX,
NVMEM_DRIVE_TYPE_ID_IDX,
NVMEM_PROFILE_ACTIVE_IDX,
NVMEM_DEFAULT_NV_ID_IDX,
NVMEM_GUEST_NV_ID_IDX,
NVMEM_PROFILE_1_NV_ID_IDX,
NVMEM_PROFILE_2_NV_ID_IDX,
NVMEM_PROFILE_3_NV_ID_IDX,
NVMEM_PROFILE_4_NV_ID_IDX,
NVMEM_PROFILE_5_NV_ID_IDX,
NVMEM_PROFILE_6_NV_ID_IDX,
NVMEM_PROFILE_7_NV_ID_IDX,
NVMEM_PROFILE_8_NV_ID_IDX,
NVMEM_PROFILE_9_NV_ID_IDX,
#ifdef AMO_DTC_SETTING
NVMEM_UDS_NV_ID_IDX,
#endif
NVMEM_BATTCAL_IDX,
NVMEM_IGNCAL_IDX,
NVMEM_CHECKSUM_IDX,
}nvmem_idx_type;

#define EEPROM_START_ADDRESS 0x14000000u
#define EEPROM_SIZE 4096
#define PAGE_SIZE 1000	//1024
#define PAGE_COUNT (EEPROM_SIZE/PAGE_SIZE)
#define WRITE_MAX 500000

#if 1
typedef union{
	struct{
		uint8_t fr_dr_mode;
		uint8_t fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rear_dr_mode;
		uint8_t rear_ps_mode;
#endif		
		uint8_t fr_dr_side_opncls;
		uint8_t fr_dr_ctr_opncls;
		uint8_t fr_ps_side_opncls;
		uint8_t fr_ps_ctr_opncls;		
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rr_dr_side_opncls;
		uint8_t rr_ps_ctr_opncls;
#endif		
		int16_t fr_dr_side_leftright_target;
		int16_t fr_dr_side_updown_target;
		int16_t fr_dr_ctr_leftright_target;
		int16_t fr_dr_ctr_updown_target;
		int16_t fr_ps_side_leftright_target;
		int16_t fr_ps_side_updown_target;
		int16_t fr_ps_ctr_leftright_target;
		int16_t fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE		
		int16_t rr_dr_side_leftright_target;
		int16_t rr_dr_side_updown_target;
		int16_t rr_ps_ctr_leftright_target;
		int16_t rr_ps_ctr_updown_target;
#endif		
		uint16_t hu_fr_dr_sidept_x;
		uint16_t hu_fr_dr_sidept_y;
		uint16_t hu_fr_dr_ctrpt_x;
		uint16_t hu_fr_dr_ctrpt_y;
		uint16_t hu_fr_ps_sidept_x;
		uint16_t hu_fr_ps_sidept_y;
		uint16_t hu_fr_ps_ctrpt_x;
		uint16_t hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint16_t hu_rr_dr_constpt_x;
		uint16_t hu_rr_dr_constpt_y;
		uint16_t hu_rr_ps_constpt_x;
		uint16_t hu_rr_ps_constpt_y;
#endif		
	}items;

	uint8_t nv_data[AMO_SETTING_BUF];
}Default_Item_nv;
#endif
typedef union{
	struct{
		uint8_t fr_dr_mode;
		uint8_t fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rear_dr_mode;
		uint8_t rear_ps_mode;
#endif		
		uint8_t fr_dr_side_opncls;
		uint8_t fr_dr_ctr_opncls;
		uint8_t fr_ps_side_opncls;
		uint8_t fr_ps_ctr_opncls;		
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rr_dr_side_opncls;
		uint8_t rr_ps_ctr_opncls;
#endif		
		int16_t fr_dr_side_leftright_target;
		int16_t fr_dr_side_updown_target;
		int16_t fr_dr_ctr_leftright_target;
		int16_t fr_dr_ctr_updown_target;
		int16_t fr_ps_side_leftright_target;
		int16_t fr_ps_side_updown_target;
		int16_t fr_ps_ctr_leftright_target;
		int16_t fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE		
		int16_t rr_dr_side_leftright_target;
		int16_t rr_dr_side_updown_target;
		int16_t rr_ps_ctr_leftright_target;
		int16_t rr_ps_ctr_updown_target;
#endif		
		uint16_t hu_fr_dr_sidept_x;
		uint16_t hu_fr_dr_sidept_y;
		uint16_t hu_fr_dr_ctrpt_x;
		uint16_t hu_fr_dr_ctrpt_y;
		uint16_t hu_fr_ps_sidept_x;
		uint16_t hu_fr_ps_sidept_y;
		uint16_t hu_fr_ps_ctrpt_x;
		uint16_t hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint16_t hu_rr_dr_constpt_x;
		uint16_t hu_rr_dr_constpt_y;
		uint16_t hu_rr_ps_constpt_x;
		uint16_t hu_rr_ps_constpt_y;
#endif		
	}items;

	uint8_t nv_data[AMO_SETTING_BUF];
}Guest_Item_nv;

typedef union{
	struct{
		uint8_t fr_dr_mode;
		uint8_t fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rear_dr_mode;
		uint8_t rear_ps_mode;
#endif		
		uint8_t fr_dr_side_opncls;
		uint8_t fr_dr_ctr_opncls;
		uint8_t fr_ps_side_opncls;
		uint8_t fr_ps_ctr_opncls;		
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rr_dr_side_opncls;
		uint8_t rr_ps_ctr_opncls;
#endif		
		int16_t fr_dr_side_leftright_target;
		int16_t fr_dr_side_updown_target;
		int16_t fr_dr_ctr_leftright_target;
		int16_t fr_dr_ctr_updown_target;
		int16_t fr_ps_side_leftright_target;
		int16_t fr_ps_side_updown_target;
		int16_t fr_ps_ctr_leftright_target;
		int16_t fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE		
		int16_t rr_dr_side_leftright_target;
		int16_t rr_dr_side_updown_target;
		int16_t rr_ps_ctr_leftright_target;
		int16_t rr_ps_ctr_updown_target;
#endif		
		uint16_t hu_fr_dr_sidept_x;
		uint16_t hu_fr_dr_sidept_y;
		uint16_t hu_fr_dr_ctrpt_x;
		uint16_t hu_fr_dr_ctrpt_y;
		uint16_t hu_fr_ps_sidept_x;
		uint16_t hu_fr_ps_sidept_y;
		uint16_t hu_fr_ps_ctrpt_x;
		uint16_t hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint16_t hu_rr_dr_constpt_x;
		uint16_t hu_rr_dr_constpt_y;
		uint16_t hu_rr_ps_constpt_x;
		uint16_t hu_rr_ps_constpt_y;
#endif		
	}items;

	uint8_t nv_data[AMO_SETTING_BUF];
}Profile_1_Item_nv;

typedef union{
	struct{
		uint8_t fr_dr_mode;
		uint8_t fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rear_dr_mode;
		uint8_t rear_ps_mode;
#endif		
		uint8_t fr_dr_side_opncls;
		uint8_t fr_dr_ctr_opncls;
		uint8_t fr_ps_side_opncls;
		uint8_t fr_ps_ctr_opncls;		
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rr_dr_side_opncls;
		uint8_t rr_ps_ctr_opncls;
#endif		
		int16_t fr_dr_side_leftright_target;
		int16_t fr_dr_side_updown_target;
		int16_t fr_dr_ctr_leftright_target;
		int16_t fr_dr_ctr_updown_target;
		int16_t fr_ps_side_leftright_target;
		int16_t fr_ps_side_updown_target;
		int16_t fr_ps_ctr_leftright_target;
		int16_t fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE		
		int16_t rr_dr_side_leftright_target;
		int16_t rr_dr_side_updown_target;
		int16_t rr_ps_ctr_leftright_target;
		int16_t rr_ps_ctr_updown_target;
#endif		
		uint16_t hu_fr_dr_sidept_x;
		uint16_t hu_fr_dr_sidept_y;
		uint16_t hu_fr_dr_ctrpt_x;
		uint16_t hu_fr_dr_ctrpt_y;
		uint16_t hu_fr_ps_sidept_x;
		uint16_t hu_fr_ps_sidept_y;
		uint16_t hu_fr_ps_ctrpt_x;
		uint16_t hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint16_t hu_rr_dr_constpt_x;
		uint16_t hu_rr_dr_constpt_y;
		uint16_t hu_rr_ps_constpt_x;
		uint16_t hu_rr_ps_constpt_y;
#endif		
	}items;

	uint8_t nv_data[AMO_SETTING_BUF];
}Profile_2_Item_nv;

typedef union{
	struct{
		uint8_t fr_dr_mode;
		uint8_t fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rear_dr_mode;
		uint8_t rear_ps_mode;
#endif		
		uint8_t fr_dr_side_opncls;
		uint8_t fr_dr_ctr_opncls;
		uint8_t fr_ps_side_opncls;
		uint8_t fr_ps_ctr_opncls;		
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rr_dr_side_opncls;
		uint8_t rr_ps_ctr_opncls;
#endif		
		int16_t fr_dr_side_leftright_target;
		int16_t fr_dr_side_updown_target;
		int16_t fr_dr_ctr_leftright_target;
		int16_t fr_dr_ctr_updown_target;
		int16_t fr_ps_side_leftright_target;
		int16_t fr_ps_side_updown_target;
		int16_t fr_ps_ctr_leftright_target;
		int16_t fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE		
		int16_t rr_dr_side_leftright_target;
		int16_t rr_dr_side_updown_target;
		int16_t rr_ps_ctr_leftright_target;
		int16_t rr_ps_ctr_updown_target;
#endif		
		uint16_t hu_fr_dr_sidept_x;
		uint16_t hu_fr_dr_sidept_y;
		uint16_t hu_fr_dr_ctrpt_x;
		uint16_t hu_fr_dr_ctrpt_y;
		uint16_t hu_fr_ps_sidept_x;
		uint16_t hu_fr_ps_sidept_y;
		uint16_t hu_fr_ps_ctrpt_x;
		uint16_t hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint16_t hu_rr_dr_constpt_x;
		uint16_t hu_rr_dr_constpt_y;
		uint16_t hu_rr_ps_constpt_x;
		uint16_t hu_rr_ps_constpt_y;
#endif		
	}items;

	uint8_t nv_data[AMO_SETTING_BUF];
}Profile_3_Item_nv;

typedef union{
	struct{
		uint8_t fr_dr_mode;
		uint8_t fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rear_dr_mode;
		uint8_t rear_ps_mode;
#endif		
		uint8_t fr_dr_side_opncls;
		uint8_t fr_dr_ctr_opncls;
		uint8_t fr_ps_side_opncls;
		uint8_t fr_ps_ctr_opncls;		
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rr_dr_side_opncls;
		uint8_t rr_ps_ctr_opncls;
#endif		
		int16_t fr_dr_side_leftright_target;
		int16_t fr_dr_side_updown_target;
		int16_t fr_dr_ctr_leftright_target;
		int16_t fr_dr_ctr_updown_target;
		int16_t fr_ps_side_leftright_target;
		int16_t fr_ps_side_updown_target;
		int16_t fr_ps_ctr_leftright_target;
		int16_t fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE		
		int16_t rr_dr_side_leftright_target;
		int16_t rr_dr_side_updown_target;
		int16_t rr_ps_ctr_leftright_target;
		int16_t rr_ps_ctr_updown_target;
#endif		
		uint16_t hu_fr_dr_sidept_x;
		uint16_t hu_fr_dr_sidept_y;
		uint16_t hu_fr_dr_ctrpt_x;
		uint16_t hu_fr_dr_ctrpt_y;
		uint16_t hu_fr_ps_sidept_x;
		uint16_t hu_fr_ps_sidept_y;
		uint16_t hu_fr_ps_ctrpt_x;
		uint16_t hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint16_t hu_rr_dr_constpt_x;
		uint16_t hu_rr_dr_constpt_y;
		uint16_t hu_rr_ps_constpt_x;
		uint16_t hu_rr_ps_constpt_y;
#endif		
	}items;

	uint8_t nv_data[AMO_SETTING_BUF];
}Profile_4_Item_nv;

typedef union{
	struct{
		uint8_t fr_dr_mode;
		uint8_t fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rear_dr_mode;
		uint8_t rear_ps_mode;
#endif		
		uint8_t fr_dr_side_opncls;
		uint8_t fr_dr_ctr_opncls;
		uint8_t fr_ps_side_opncls;
		uint8_t fr_ps_ctr_opncls;		
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rr_dr_side_opncls;
		uint8_t rr_ps_ctr_opncls;
#endif		
		int16_t fr_dr_side_leftright_target;
		int16_t fr_dr_side_updown_target;
		int16_t fr_dr_ctr_leftright_target;
		int16_t fr_dr_ctr_updown_target;
		int16_t fr_ps_side_leftright_target;
		int16_t fr_ps_side_updown_target;
		int16_t fr_ps_ctr_leftright_target;
		int16_t fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE		
		int16_t rr_dr_side_leftright_target;
		int16_t rr_dr_side_updown_target;
		int16_t rr_ps_ctr_leftright_target;
		int16_t rr_ps_ctr_updown_target;
#endif		
		uint16_t hu_fr_dr_sidept_x;
		uint16_t hu_fr_dr_sidept_y;
		uint16_t hu_fr_dr_ctrpt_x;
		uint16_t hu_fr_dr_ctrpt_y;
		uint16_t hu_fr_ps_sidept_x;
		uint16_t hu_fr_ps_sidept_y;
		uint16_t hu_fr_ps_ctrpt_x;
		uint16_t hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint16_t hu_rr_dr_constpt_x;
		uint16_t hu_rr_dr_constpt_y;
		uint16_t hu_rr_ps_constpt_x;
		uint16_t hu_rr_ps_constpt_y;
#endif		
	}items;

	uint8_t nv_data[AMO_SETTING_BUF];
}Profile_5_Item_nv;

typedef union{
	struct{
		uint8_t fr_dr_mode;
		uint8_t fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rear_dr_mode;
		uint8_t rear_ps_mode;
#endif		
		uint8_t fr_dr_side_opncls;
		uint8_t fr_dr_ctr_opncls;
		uint8_t fr_ps_side_opncls;
		uint8_t fr_ps_ctr_opncls;		
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rr_dr_side_opncls;
		uint8_t rr_ps_ctr_opncls;
#endif		
		int16_t fr_dr_side_leftright_target;
		int16_t fr_dr_side_updown_target;
		int16_t fr_dr_ctr_leftright_target;
		int16_t fr_dr_ctr_updown_target;
		int16_t fr_ps_side_leftright_target;
		int16_t fr_ps_side_updown_target;
		int16_t fr_ps_ctr_leftright_target;
		int16_t fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE		
		int16_t rr_dr_side_leftright_target;
		int16_t rr_dr_side_updown_target;
		int16_t rr_ps_ctr_leftright_target;
		int16_t rr_ps_ctr_updown_target;
#endif		
		uint16_t hu_fr_dr_sidept_x;
		uint16_t hu_fr_dr_sidept_y;
		uint16_t hu_fr_dr_ctrpt_x;
		uint16_t hu_fr_dr_ctrpt_y;
		uint16_t hu_fr_ps_sidept_x;
		uint16_t hu_fr_ps_sidept_y;
		uint16_t hu_fr_ps_ctrpt_x;
		uint16_t hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint16_t hu_rr_dr_constpt_x;
		uint16_t hu_rr_dr_constpt_y;
		uint16_t hu_rr_ps_constpt_x;
		uint16_t hu_rr_ps_constpt_y;
#endif		
	}items;

	uint8_t nv_data[AMO_SETTING_BUF];
}Profile_6_Item_nv;

typedef union{
	struct{
		uint8_t fr_dr_mode;
		uint8_t fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rear_dr_mode;
		uint8_t rear_ps_mode;
#endif		
		uint8_t fr_dr_side_opncls;
		uint8_t fr_dr_ctr_opncls;
		uint8_t fr_ps_side_opncls;
		uint8_t fr_ps_ctr_opncls;		
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rr_dr_side_opncls;
		uint8_t rr_ps_ctr_opncls;
#endif		
		int16_t fr_dr_side_leftright_target;
		int16_t fr_dr_side_updown_target;
		int16_t fr_dr_ctr_leftright_target;
		int16_t fr_dr_ctr_updown_target;
		int16_t fr_ps_side_leftright_target;
		int16_t fr_ps_side_updown_target;
		int16_t fr_ps_ctr_leftright_target;
		int16_t fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE		
		int16_t rr_dr_side_leftright_target;
		int16_t rr_dr_side_updown_target;
		int16_t rr_ps_ctr_leftright_target;
		int16_t rr_ps_ctr_updown_target;
#endif		
		uint16_t hu_fr_dr_sidept_x;
		uint16_t hu_fr_dr_sidept_y;
		uint16_t hu_fr_dr_ctrpt_x;
		uint16_t hu_fr_dr_ctrpt_y;
		uint16_t hu_fr_ps_sidept_x;
		uint16_t hu_fr_ps_sidept_y;
		uint16_t hu_fr_ps_ctrpt_x;
		uint16_t hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint16_t hu_rr_dr_constpt_x;
		uint16_t hu_rr_dr_constpt_y;
		uint16_t hu_rr_ps_constpt_x;
		uint16_t hu_rr_ps_constpt_y;
#endif		
	}items;

	uint8_t nv_data[AMO_SETTING_BUF];
}Profile_7_Item_nv;

typedef union{
	struct{
		uint8_t fr_dr_mode;
		uint8_t fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rear_dr_mode;
		uint8_t rear_ps_mode;
#endif		
		uint8_t fr_dr_side_opncls;
		uint8_t fr_dr_ctr_opncls;
		uint8_t fr_ps_side_opncls;
		uint8_t fr_ps_ctr_opncls;		
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rr_dr_side_opncls;
		uint8_t rr_ps_ctr_opncls;
#endif		
		int16_t fr_dr_side_leftright_target;
		int16_t fr_dr_side_updown_target;
		int16_t fr_dr_ctr_leftright_target;
		int16_t fr_dr_ctr_updown_target;
		int16_t fr_ps_side_leftright_target;
		int16_t fr_ps_side_updown_target;
		int16_t fr_ps_ctr_leftright_target;
		int16_t fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE		
		int16_t rr_dr_side_leftright_target;
		int16_t rr_dr_side_updown_target;
		int16_t rr_ps_ctr_leftright_target;
		int16_t rr_ps_ctr_updown_target;
#endif		
		uint16_t hu_fr_dr_sidept_x;
		uint16_t hu_fr_dr_sidept_y;
		uint16_t hu_fr_dr_ctrpt_x;
		uint16_t hu_fr_dr_ctrpt_y;
		uint16_t hu_fr_ps_sidept_x;
		uint16_t hu_fr_ps_sidept_y;
		uint16_t hu_fr_ps_ctrpt_x;
		uint16_t hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint16_t hu_rr_dr_constpt_x;
		uint16_t hu_rr_dr_constpt_y;
		uint16_t hu_rr_ps_constpt_x;
		uint16_t hu_rr_ps_constpt_y;
#endif		
	}items;

	uint8_t nv_data[AMO_SETTING_BUF];
}Profile_8_Item_nv;

typedef union{
	struct{
		uint8_t fr_dr_mode;
		uint8_t fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rear_dr_mode;
		uint8_t rear_ps_mode;
#endif		
		uint8_t fr_dr_side_opncls;
		uint8_t fr_dr_ctr_opncls;
		uint8_t fr_ps_side_opncls;
		uint8_t fr_ps_ctr_opncls;		
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rr_dr_side_opncls;
		uint8_t rr_ps_ctr_opncls;
#endif		
		int16_t fr_dr_side_leftright_target;
		int16_t fr_dr_side_updown_target;
		int16_t fr_dr_ctr_leftright_target;
		int16_t fr_dr_ctr_updown_target;
		int16_t fr_ps_side_leftright_target;
		int16_t fr_ps_side_updown_target;
		int16_t fr_ps_ctr_leftright_target;
		int16_t fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE		
		int16_t rr_dr_side_leftright_target;
		int16_t rr_dr_side_updown_target;
		int16_t rr_ps_ctr_leftright_target;
		int16_t rr_ps_ctr_updown_target;
#endif		
		uint16_t hu_fr_dr_sidept_x;
		uint16_t hu_fr_dr_sidept_y;
		uint16_t hu_fr_dr_ctrpt_x;
		uint16_t hu_fr_dr_ctrpt_y;
		uint16_t hu_fr_ps_sidept_x;
		uint16_t hu_fr_ps_sidept_y;
		uint16_t hu_fr_ps_ctrpt_x;
		uint16_t hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint16_t hu_rr_dr_constpt_x;
		uint16_t hu_rr_dr_constpt_y;
		uint16_t hu_rr_ps_constpt_x;
		uint16_t hu_rr_ps_constpt_y;
#endif		
	}items;

	uint8_t nv_data[AMO_SETTING_BUF];
}Profile_9_Item_nv;

typedef union{
	struct{
		uint8_t fr_dr_mode;
		uint8_t fr_ps_mode;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rear_dr_mode;
		uint8_t rear_ps_mode;
#endif		
		uint8_t fr_dr_side_opncls;
		uint8_t fr_dr_ctr_opncls;
		uint8_t fr_ps_side_opncls;
		uint8_t fr_ps_ctr_opncls;		
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint8_t rr_dr_side_opncls;
		uint8_t rr_ps_ctr_opncls;
#endif		
		int16_t fr_dr_side_leftright_target;
		int16_t fr_dr_side_updown_target;
		int16_t fr_dr_ctr_leftright_target;
		int16_t fr_dr_ctr_updown_target;
		int16_t fr_ps_side_leftright_target;
		int16_t fr_ps_side_updown_target;
		int16_t fr_ps_ctr_leftright_target;
		int16_t fr_ps_ctr_updown_target;
#ifdef AMO_GN7_PE_SETTING_NONE		
		int16_t rr_dr_side_leftright_target;
		int16_t rr_dr_side_updown_target;
		int16_t rr_ps_ctr_leftright_target;
		int16_t rr_ps_ctr_updown_target;
#endif		
		uint16_t hu_fr_dr_sidept_x;
		uint16_t hu_fr_dr_sidept_y;
		uint16_t hu_fr_dr_ctrpt_x;
		uint16_t hu_fr_dr_ctrpt_y;
		uint16_t hu_fr_ps_sidept_x;
		uint16_t hu_fr_ps_sidept_y;
		uint16_t hu_fr_ps_ctrpt_x;
		uint16_t hu_fr_ps_ctrpt_y;
#ifdef AMO_GN7_PE_SETTING_NONE		
		uint16_t hu_rr_dr_constpt_x;
		uint16_t hu_rr_dr_constpt_y;
		uint16_t hu_rr_ps_constpt_x;
		uint16_t hu_rr_ps_constpt_y;
#endif		
	}items;

	uint8_t nv_data[AMO_SETTING_BUF];
}Profile_Item_nv;

#ifdef AMO_DTC_SETTING
#pragma pack(push, 1)
typedef struct _dtc_info{
	uint32_t dtcCode;
	uint8_t dtcStatus;
	uint32_t dtcOccurence;
}DTC_Info;

typedef struct uds_dtc_info{
	DTC_Info DtcRecord[MAX_DTC_COUNT];
}UDS_DTC_Info;
#pragma pack(pop)
#endif

#if 0
typedef struct{
	uint32_t write_cnt;
	uint8_t registration;
	uint8_t nvinit;
	uint8_t profile;
	uint8_t drive_type;
	uint8_t profile_active[11];
	Default_Item_nv default_nv;	
	Guest_Item_nv guest_nv;	
	Profile_1_Item_nv profile_1_nv;	
	Profile_2_Item_nv profile_2_nv;	
	Profile_3_Item_nv profile_3_nv;	
	Profile_4_Item_nv profile_4_nv;	
	Profile_5_Item_nv profile_5_nv;	
	Profile_6_Item_nv profile_6_nv;	
	Profile_7_Item_nv profile_7_nv;	
	Profile_8_Item_nv profile_8_nv;	
	Profile_9_Item_nv profile_9_nv;	
	uint16_t nv_battcal;
	uint16_t nv_igncal;
	#ifdef AMO_DTC_SETTING
	UDS_DTC_Info uds_dtc_nv;
	#endif
	uint8_t nv_checksum;
}Nv_data_item;
#endif

typedef struct{
	uint32_t write_cnt;
	uint8_t registration;
	uint8_t nvinit;
	uint8_t profile;
	uint8_t drive_type;
	uint8_t profile_active[11];
	Profile_Item_nv	profile_item[11];	
	uint16_t nv_battcal;
	uint16_t nv_igncal;
	#ifdef AMO_DTC_SETTING
	UDS_DTC_Info uds_dtc_nv;
	#endif
	uint8_t nv_checksum;
}Nv_data_item;

typedef union{
	Nv_data_item nv_item;
	uint8_t nv_data[PAGE_SIZE];
}nvmem_data_type;

extern uint32_t set_address;
extern uint8_t nv_page_cnt;
void nvmem_init(void);
void nvmem_init_call(void);
int nvmem_read(nvmem_data_type *pcBuffer);
status_t nvmem_write(const uint8_t *nvBuffer, int n_size);
void nvmem_erase_all(void);
uint8_t Calculate_Chksum(const uint8_t *data, unsigned int length);
#endif
