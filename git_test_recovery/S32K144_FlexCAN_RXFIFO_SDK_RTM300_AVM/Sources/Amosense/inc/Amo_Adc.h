#ifndef AMO_ADC_H
#define AMO_ADC_H

/*===========================================================================*/
/* Project   :  AMOSENSE ADC driver Software                                                                */
/* File name :  Amo_ADC.h                                                                             */
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

#define ADC_INSTANCE		0UL
#define ADC_CHN 				12U
#define ADC_VREFH 			12.0f
#define ADC_VREFL 			0.0f
#define NUM_SAMPLE 10

#define VATT_OVER_LIMIT 3755	//3460 //3455
#define VATT_UNDER_LIMIT 2010 //1855


#define VATT_COM_OVER_LIMIT 4276 //4255 //3915
#define VATT_COM_UNDER_LIMIT 1512 //1510	//1400 //1855

#define VATT_CAL_OVER_LIMIT 4473	//4465 
#define VATT_CAL_UNDER_LIMIT 1733	//1730	

#define SAFE_SHIFT(val, idx)	(((idx) < 32u) ? ((val) << (idx)) : 0u)

typedef struct{
uint16_t ign2_limit;
uint16_t ign3_limit;
uint16_t min;
uint16_t max;
} EvntVoltType;

extern uint16_t IGN2_adcValues[NUM_SAMPLE];
extern uint16_t VATT_adcValues[NUM_SAMPLE];

volatile extern uint8_t Evnt_VBATT_Onoff;
extern uint8_t Evnt_IGN2_Onoff,Evnt_IGN3_Onoff,Evnt_VBATT_Over,Evnt_VBATT_Under,Evnt_COM_Onoff,Evnt_COM_Over,Evnt_COM_Under,Vatt_low_his;
extern uint16_t Ign2_adcRawValue, Ign3_adcRawValue,Vatt_adc_RawValue,batt_data,ign_data, adcMax,Ign2_AdcMinValue,Ign3_AdcMinValue,Vatt_LimitValue,IGN3_Value;
extern float Ign2_adcValue, Ign3_adcValue, Vatt_adcValue;

extern int compare(const void *a, const void *b);
void Amo_Adc_Init(void);
void Adc_Value(void);
#endif /*AMO_ADC_H */


