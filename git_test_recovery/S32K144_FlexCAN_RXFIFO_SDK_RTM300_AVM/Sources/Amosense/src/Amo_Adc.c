#ifndef AMO_ADC_C
#define AMO_ADC_C

/*===========================================================================*/
/* Project   :  AMOSENSE CAN driver Software                                                                */
/* File name :  Amo_Adc.c                                                                             */
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
#include "helper_functions.h"
#include "Amo_Adc.h"
#include "Amo_Nvm.h"

uint16_t FloatToUint16(float val);
/******************************************************************************
 * Groval variable prototypes
 ******************************************************************************/
uint16_t IGN3_Value = 0;
uint8_t Evnt_IGN2_Onoff = 0,Evnt_IGN3_Onoff = 0, Evnt_VBATT_Over = 0,Evnt_VBATT_Under = 0,Evnt_COM_Onoff = 0,Evnt_COM_Over = 0,Evnt_COM_Under = 0, Vatt_low_his = 0, Vatt_high_his = 0;
volatile uint8_t Evnt_VBATT_Onoff = 1;
static uint8_t adc_1st_skip = 1;
/******************************************************************************
 * Local variable Define
 ******************************************************************************/
uint16_t Ign2_adcRawValue = 0,Ign3_adcRawValue = 0,Vatt_adc_RawValue = 0,adcMax = 0,Ign2_AdcMinValue = 0,Ign3_AdcMinValue = 0,Vatt_LimitValue = 0,batt_data = 0,ign_data = 0;
float Ign2_adcValue = 0.0,Ign3_adcValue = 0.0,Vatt_adcValue = 0.0;
uint16_t IGN2_adcValues[NUM_SAMPLE];
uint16_t VATT_adcValues[NUM_SAMPLE];

/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/
/**
 * @brief  compare function
 * @param  Event: None
 * @retval Ack: Return whether the Event has been managed or not
 * @example  None
 * @note This function polls until conversion is complete.
 */
int compare(const void *a, const void *b) {
		int result = 0;
    int16_t val_a = *(const int16_t *)a;
		int16_t val_b = *(const int16_t *)b;
		if(val_a < val_b)
		{
			result =  -1;
		}
		else if(val_a > val_b)
		{
			result =  1;
		}
		else
		{
			result =  0;
		}
		return result;
}

/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/
/**
	* @brief  FloatToUint16 function
	* @param  Event: None
	* @retval Ack: Return whether the Event has been managed or not
	* @example  None
	* @details
	*         - If the input value is negative, returns 0.
	*         - If the input value exceeds UINT16_MAX, returns UINT16_MAX.
	*         - Otherwise, casts the float value to uint32_t, then to uint16_t. 
	* @note This function polls until conversion is complete.	
 */
uint16_t FloatToUint16(float val)
{
		uint16_t ret = 0;
    if (val < 0.0f)
    {
        ret = 0;
    }
    else if (val > (float)UINT16_MAX)
    {
        ret = UINT16_MAX;
    }
    else
    {
        ret = (uint16_t)(uint32_t)val;
    }

		return ret;
}

/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/
/**
 * @brief  Adc_Value function
 * @param  Event: None
 * @retval Ack: Return whether the Event has been managed or not
 * @example  None
 * @note This function polls until conversion is complete.
 */
void Adc_Value(void)
{
	//Start ADC conversion on selected channel
	memset(IGN2_adcValues, 0x00, sizeof(IGN2_adcValues));
	memset(VATT_adcValues, 0x00, sizeof(VATT_adcValues));
	//memset(IGN3_adcValues, 0x00, sizeof(IGN3_adcValues));
	ADC_DRV_ConfigChan(ADC_INSTANCE, 0U, &adConv1_ChnConfig0);
	ADC_DRV_WaitConvDone(ADC_INSTANCE);
	for(int i=0; i<NUM_SAMPLE; i++)
	{
		ADC_DRV_GetChanResult(ADC_INSTANCE, 0U, &IGN2_adcValues[i]);
	} 
	qsort(IGN2_adcValues, NUM_SAMPLE, sizeof(int16_t), compare);

	Ign2_adcRawValue = IGN2_adcValues[NUM_SAMPLE/2];
	Ign2_adcValue = ((float)Ign2_adcRawValue * ((float) 5 / (float)adcMax));
	//Ign2_AdcMinValue = (uint16_t)(Ign2_adcValue * 1000.0f);
	Ign2_AdcMinValue = FloatToUint16(Ign2_adcValue * 1000.0f); 
	Ign2_AdcMinValue = Ign2_AdcMinValue + ign_data;

	ADC_DRV_ConfigChan(ADC_INSTANCE, 0U, &adConv1_ChnConfig1);
	ADC_DRV_WaitConvDone(ADC_INSTANCE);
	for(int i=0; i<NUM_SAMPLE; i++)
	{
		ADC_DRV_GetChanResult(ADC_INSTANCE, 0U, &VATT_adcValues[i]);
	}
	qsort(VATT_adcValues, NUM_SAMPLE, sizeof(int16_t), compare);
	Vatt_adc_RawValue = VATT_adcValues[NUM_SAMPLE/2];
	Vatt_adcValue = ((float)Vatt_adc_RawValue * ((float) 5 / (float)adcMax));
	//Vatt_LimitValue = (uint16_t)(Vatt_adcValue * 1000.0f);
	Vatt_LimitValue = FloatToUint16(Vatt_adcValue * 1000.0f); 
	Vatt_LimitValue = Vatt_LimitValue + batt_data;
#ifdef AMO_GN7_PE_SETTING_NONE
	ADC_DRV_ConfigChan(ADC_INSTANCE, 0U, &adConv1_ChnConfig2);
	ADC_DRV_WaitConvDone(ADC_INSTANCE);
	for(int i=0; i<NUM_SAMPLE; i++)
	{
		ADC_DRV_GetChanResult(ADC_INSTANCE, 0U, &IGN3_adcValues[i]);
	}
	qsort(IGN3_adcValues, NUM_SAMPLE, sizeof(int16_t), compare);
	Ign3_adcRawValue = IGN3_adcValues[NUM_SAMPLE/2];
	Ign3_adcValue = ((float)Ign3_adcRawValue * ((float) 5 / (float)adcMax));
	Ign3_AdcMinValue = (uint16_t)(Ign3_adcValue * 1000.0f);
	///////////////////////////////////////////////////////////////////////////////////
#endif
//Start adc/igm level value choice
#ifdef BATT_CAL_NONE
	if(Ign2_AdcMinValue > 1690)  //1705//1500
	{
		Evnt_IGN2_Onoff = 1;
	}

	if(Ign2_AdcMinValue < 1500)  //1000
	{
		Evnt_IGN2_Onoff = 0;
	}
#else
	if(Ign2_AdcMinValue > 1725)  //1705//1500
	{
		Evnt_IGN2_Onoff = 1;
	}

	if(Ign2_AdcMinValue < 1457)  //1000
	{
		Evnt_IGN2_Onoff = 0;
	} 
#endif

	if(adc_1st_skip)
	{		
		adc_1st_skip = 0;
	}
	else
	{
		if((Vatt_LimitValue >= VATT_CAL_UNDER_LIMIT) && (Vatt_LimitValue <= VATT_CAL_OVER_LIMIT))
		{
			Vatt_high_his = 1;
		}		
	}
#ifdef AMO_GN7_PE_SETTING_NONE	
	if(Ign3_AdcMinValue < 1000)
	{
		Evnt_IGN3_Onoff = 0;
	}
	else
	{
		Evnt_IGN3_Onoff = 1;
	}
#endif
	if((batt_data == 0) && (ign_data == 0))
	{
		if(Vatt_LimitValue > VATT_COM_OVER_LIMIT)
		{
			Evnt_COM_Over = 1;
			Evnt_COM_Onoff = 0;
		}
		if(Vatt_LimitValue < VATT_COM_UNDER_LIMIT)
		{
			Evnt_COM_Under = 1;
			Evnt_COM_Onoff = 0;
		} 
		if(Evnt_COM_Over == 1)
		{
			if(Vatt_LimitValue <= 4110)
			{
				Evnt_COM_Onoff = 1;
				Evnt_COM_Over = 0;
			}
		}
		else if(Evnt_COM_Under == 1)
		{
			if(Vatt_LimitValue >= 1401) 
			{
				Evnt_COM_Onoff = 1;
				Evnt_COM_Under = 0;
			}
		}
		else
		{
			if((Vatt_LimitValue >=VATT_COM_UNDER_LIMIT) && (Vatt_LimitValue <=VATT_COM_OVER_LIMIT))
			{
				Evnt_COM_Onoff = 1;
			}
		} 
	}
	else
	{
		if(Vatt_LimitValue > VATT_CAL_OVER_LIMIT)
		{
			Evnt_COM_Over = 1;
			Evnt_COM_Onoff = 0;
		}
		if(Vatt_LimitValue < VATT_CAL_UNDER_LIMIT)
		{
			Evnt_COM_Under = 1;
			Evnt_COM_Onoff = 0;
		} 
		if(Evnt_COM_Over == 1)
		{
			if((Vatt_LimitValue <= 4595) && (Vatt_high_his == 1))
			{
				Evnt_COM_Onoff = 1;
				Evnt_COM_Over = 0;
			}
			else
			{
				Vatt_high_his = 0;
				Evnt_COM_Over = 0;				
			}		

			if(Vatt_LimitValue > 4595)
			{
				Vatt_high_his = 0;
			}
		}
		else if(Evnt_COM_Under == 1)
		{
			if((Vatt_LimitValue >= 1615) && (Vatt_low_his == 0))
			{
				Evnt_COM_Onoff = 1;
				Evnt_COM_Under = 0;
			}
			else
			{
				Vatt_low_his = 1;
				Evnt_COM_Under = 0;
			}
			
			if((Vatt_LimitValue >= 1615) && (Evnt_IGN2_Onoff == 0))
			{
				Evnt_COM_Onoff = 0;
				Evnt_COM_Under = 1;
			}
		}
		else
		{
			if((Vatt_LimitValue >=VATT_CAL_UNDER_LIMIT) && (Vatt_LimitValue <=VATT_CAL_OVER_LIMIT))
			{
				Evnt_COM_Onoff = 1;
				Vatt_low_his = 0;
				//Vatt_high_his = 0;
			}
		} 	
	}
#ifdef DEBUG_MODE
	//Evt_queue_add(DEVICE_CAN_RXDATA_TIMEOUT_EVENT);
#endif
}

/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/
/**
 * @brief  read_device_nv function
 * @param  Event: None
 * @retval Ack: Return whether the Event has been managed or not
 * @example  None
 * @note This function polls until conversion is complete.
 */
static int read_device_nv(nvmem_data_type *pcBuffer)
{
	return nvmem_read(pcBuffer);
}

/* Functions Definition ------------------------------------------------------*/
/* Private functions ----------------------------------------------------------*/
/**
 * @brief  Adc_main function
 * @param  Event: None
 * @retval Ack: Return whether the Event has been managed or not
 * @example  None
 * @note This function polls until conversion is complete.
 */
void Amo_Adc_Init(void)
{
	nvmem_data_type nv;
	/* Enable the floating point unit */
	enableFPU();
	/* Buffer used to store processed data for serial communication */
	/* Get ADC max value from the resolution */
	adcMax = (uint16_t)((uint32_t)1u << 12); 														
	ADC_DRV_ConfigConverter(ADC_INSTANCE, &adConv1_ConvConfig0);
	ADC_DRV_ConfigConverter(ADC_INSTANCE, &adConv1_ConvConfig1);
	#ifdef AMO_GN7_PE_SETTING_NONE
	ADC_DRV_ConfigConverter(ADC_INSTANCE, &adConv1_ConvConfig2);
	#endif
	ADC_DRV_AutoCalibration(ADC_INSTANCE);
	Amo_timer_Start(timer_69, 1000, true, Adc_Value);
	//nvmem_read(&nv);
	read_device_nv(&nv);
	batt_data = nv.nv_item.nv_battcal;
	ign_data = nv.nv_item.nv_igncal;
}
#endif /* AMO_ADC_C */

