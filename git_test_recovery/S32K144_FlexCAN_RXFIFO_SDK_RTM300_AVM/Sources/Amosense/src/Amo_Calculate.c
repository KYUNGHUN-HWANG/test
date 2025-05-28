#define Amo_CALCULATE_C_

#include <stdio.h>

#include "Cpu.h"
#include "Amo_main.h"

volatile int16_t previousValue = 0;
volatile int16_t currentValue = 0;

int16_t Cal_CanToLin_FrDr_SidePt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5))
{
	int16_t res;

	res = fp(CanLinValue[FrDrSidePt_X].Pt_X.Can_RangePt_Center_X, CanLinValue[FrDrSidePt_X].Pt_X.Can_RangePt_X, CanLinValue[FrDrSidePt_X].Pt_X.Can_EVntPt_X, \
						CanLinValue[FrDrSidePt_X].Pt_X.CanData_Pt_X, linRangePt_X, CanLinValue[FrDrSidePt_X].Pt_X.linFactor_Pt_X);

	return res;
}

int16_t Cal_CanToLin_FrDr_SidePt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5))
{
	int16_t res;

	res = fp(CanLinValue[FrDrSidePt_Y].Pt_Y.Can_RangePt_Center_Y, CanLinValue[FrDrSidePt_Y].Pt_Y.Can_RangePt_Y, CanLinValue[FrDrSidePt_Y].Pt_Y.Can_EVntPt_Y, \
					CanLinValue[FrDrSidePt_Y].Pt_Y.CanData_Pt_Y, linRangePt_Y, CanLinValue[FrDrSidePt_Y].Pt_Y.linFactor_Pt_Y);
	
	return res;
}


int16_t Cal_CanToLin_FrDr_CtrPt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5))
{
	int16_t res;

	res = fp(CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_Center_X, CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_X, CanLinValue[FrDrCtrPt_X].Pt_X.Can_EVntPt_X, \
						CanLinValue[FrDrCtrPt_X].Pt_X.CanData_Pt_X, linRangePt_X, CanLinValue[FrDrCtrPt_X].Pt_X.linFactor_Pt_X);

	return res;
}

int16_t Cal_CanToLin_FrDr_CtrPt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5))
{
	int16_t res;

	res = fp(CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_RangePt_Center_Y, CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_RangePt_Y, CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_EVntPt_Y, \
					CanLinValue[FrDrCtrPt_Y].Pt_Y.CanData_Pt_Y, linRangePt_Y, CanLinValue[FrDrCtrPt_Y].Pt_Y.linFactor_Pt_Y);
	
	return res;
}



int16_t Cal_CanToLin_FrPs_SidePt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5))
{
	int16_t res;

	res = fp(CanLinValue[FrPsSidePt_X].Pt_X.Can_RangePt_Center_X, CanLinValue[FrPsSidePt_X].Pt_X.Can_RangePt_X, CanLinValue[FrPsSidePt_X].Pt_X.Can_EVntPt_X, \
						CanLinValue[FrPsSidePt_X].Pt_X.CanData_Pt_X, linRangePt_X, CanLinValue[FrPsSidePt_X].Pt_X.linFactor_Pt_X);

	return res;
}

int16_t Cal_CanToLin_FrPs_SidePt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5))
{
	int16_t res;

	res = fp(CanLinValue[FrPsSidePt_Y].Pt_Y.Can_RangePt_Center_Y, CanLinValue[FrPsSidePt_Y].Pt_Y.Can_RangePt_Y, CanLinValue[FrPsSidePt_Y].Pt_Y.Can_EVntPt_Y, \
					CanLinValue[FrPsSidePt_Y].Pt_Y.CanData_Pt_Y, linRangePt_Y, CanLinValue[FrPsSidePt_Y].Pt_Y.linFactor_Pt_Y);
	
	return res;
}


int16_t Cal_CanToLin_FrPs_CtrPt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5))
{
	int16_t res;

	res = fp(CanLinValue[FrPsCtrPt_X].Pt_X.Can_RangePt_Center_X, CanLinValue[FrPsCtrPt_X].Pt_X.Can_RangePt_X, CanLinValue[FrPsCtrPt_X].Pt_X.Can_EVntPt_X, \
						CanLinValue[FrPsCtrPt_X].Pt_X.CanData_Pt_X, linRangePt_X, CanLinValue[FrPsCtrPt_X].Pt_X.linFactor_Pt_X);

	return res;
}

int16_t Cal_CanToLin_FrPs_CtrPt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5))
{
	int16_t res;

	res = fp(CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_RangePt_Center_Y, CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_RangePt_Y, CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_EVntPt_Y, \
					CanLinValue[FrPsCtrPt_Y].Pt_Y.CanData_Pt_Y, linRangePt_Y, CanLinValue[FrPsCtrPt_Y].Pt_Y.linFactor_Pt_Y);
	
	return res;
}

#ifdef AMO_GN7_PE_SETTING_NONE		
int16_t Cal_CanToLin_RrDr_ConsPt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5))
{
	int16_t res;

	res = fp(CanLinValue[RrDrCons_X].Pt_X.Can_RangePt_Center_X, CanLinValue[RrDrCons_X].Pt_X.Can_RangePt_X, CanLinValue[RrDrCons_X].Pt_X.Can_EVntPt_X, \
						CanLinValue[RrDrCons_X].Pt_X.CanData_Pt_X, linRangePt_X, CanLinValue[RrDrCons_X].Pt_X.linFactor_Pt_X);

	return res;
}

int16_t Cal_CanToLin_RrDr_ConsPt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5))
{
	int16_t res;

	res = fp(CanLinValue[RrDrCons_Y].Pt_Y.Can_RangePt_Center_Y, CanLinValue[RrDrCons_Y].Pt_Y.Can_RangePt_Y, CanLinValue[RrDrCons_Y].Pt_Y.Can_EVntPt_Y, \
					CanLinValue[RrDrCons_Y].Pt_Y.CanData_Pt_Y, linRangePt_Y, CanLinValue[RrDrCons_Y].Pt_Y.linFactor_Pt_Y);
	
	return res;
}


int16_t Cal_CanToLin_RrPs_ConsPt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5))
{
	int16_t res;

	res = fp(CanLinValue[RrPsCons_X].Pt_X.Can_RangePt_Center_X, CanLinValue[RrPsCons_X].Pt_X.Can_RangePt_X, CanLinValue[RrPsCons_X].Pt_X.Can_EVntPt_X, \
						CanLinValue[RrPsCons_X].Pt_X.CanData_Pt_X, linRangePt_X, CanLinValue[RrPsCons_X].Pt_X.linFactor_Pt_X);

	return res;
}

int16_t Cal_CanToLin_RrPs_ConsPt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5))
{
	int16_t res;

	res = fp(CanLinValue[RrPsCons_Y].Pt_Y.Can_RangePt_Center_Y, CanLinValue[RrPsCons_Y].Pt_Y.Can_RangePt_Y, CanLinValue[RrPsCons_Y].Pt_Y.Can_EVntPt_Y, \
					CanLinValue[RrPsCons_Y].Pt_Y.CanData_Pt_Y, linRangePt_Y, CanLinValue[RrPsCons_Y].Pt_Y.linFactor_Pt_Y);
	
	return res;
}
#endif

int16_t Conv_Can_Pt_X(int16_t Can_RangePt_Center, int16_t Can_RangePt, int16_t Can_EVntPt, int16_t CanData_Pt, int16_t LinRangePt_X, float linFactor_Pt)
{
	int16_t res;
	int16_t LinData_Pt;

	Can_RangePt_Center = (int16_t)((uint16_t)Can_RangePt >> 1);
	CanData_Pt = (Can_EVntPt - Can_RangePt_Center);
	linFactor_Pt = (float)(LinRangePt_X) / (float)(Can_RangePt);
	
	#if (defined ACT_ANGLE_UPDATE || defined ACT_ANGLE_UPDATE_2 || defined ACT_ANGLE_UPDATE_3)
	if (CanData_Pt < 0)
	{
		LinData_Pt = -(int16_t)((uint16_t)((float)(-CanData_Pt) * linFactor_Pt) & 0xFFFu); //12bit mask
	}
	else
	{
		LinData_Pt = (int16_t)((uint16_t)((float)(CanData_Pt) * linFactor_Pt) & 0xFFFu); //12bit mask
	}
	#elif defined(OLD_ANGLE)
	LinData_Pt = (int16_t)((uint16_t)((float)(CanData_Pt) * linFactor_Pt) & 0x7FFu);  //11bit mask
	#endif
	
	res = LinData_Pt;
	
	return res;
}



int16_t Conv_Can_Pt_Y(int16_t Can_RangePt_Center, int16_t Can_RangePt, int16_t Can_EVntPt, int16_t CanData_Pt, int16_t LinRangePt_Y, float linFactor_Pt)
{
	int16_t res;
	int16_t LinData_Pt;

	Can_RangePt_Center = (int16_t)((uint16_t)Can_RangePt >> 1);
	CanData_Pt = Can_EVntPt - Can_RangePt_Center;
	linFactor_Pt = (float)(LinRangePt_Y) / (float)(Can_RangePt);
	
	#if (defined ACT_ANGLE_UPDATE || defined ACT_ANGLE_UPDATE_2 || defined ACT_ANGLE_UPDATE_3)
	if (CanData_Pt < 0)
	{
		LinData_Pt = (int16_t)((uint16_t)((float)(-CanData_Pt) * linFactor_Pt) & 0xFFFu); //2's complement  //12bit mask
	}
	else
	{
		LinData_Pt = -(int16_t)((uint16_t)((float)CanData_Pt * linFactor_Pt) & 0xFFFu); //2's complement  //12bit mask
	}

	#elif defined(OLD_ANGLE)
	LinData_Pt = (int16_t)((uint16_t)((~(uint16_t)CanData_Pt + 1) * (int32_t)linFactor_Pt) & 0x7FFu);  //11bit mask
	#endif
	
	res = LinData_Pt;
	
	return res;

	
}

uint16_t Cal_LinToCan_EVntSidePt_X(const uint64_t *LINData, const uint64_t *data_Bdry_Up, const uint64_t *data_Bdry_Lo)
{
	uint16_t CANRange;
	uint16_t CANCenter;
//	uint8_t factor = 10;
//	int16_t LINRange = 600;
	uint16_t LINCenter;
	uint16_t CANData;
	uint16_t result;

	CANRange = (uint16_t)((*data_Bdry_Up * canFactor)-(*data_Bdry_Lo * canFactor));
	CANCenter = CANRange >> 1;
	LINCenter = linRangePt_X >> 1;
	if((((uint16_t)(4096-LINCenter))<=(*LINData)) && ((*LINData)<=(4096))  )//2048
	{
		CANData=(uint16_t)(((double)4096-(double)*LINData)*((double)CANRange/(double)linRangePt_X));
		result = CANData+CANCenter;
	}
	else if((*LINData)<=(LINCenter))
	{
		CANData=(uint16_t)(((double)*LINData)*((double)CANRange/(double)linRangePt_X));
		result = CANCenter-CANData;
	}
	else
	{
	result=CANCenter;
	}

	return result;
}


uint16_t Cal_CAN_CenterPt(const uint64_t *data_Bdry_Up, const uint64_t *data_Bdry_Lo)
{
	uint16_t CANRange;
	uint16_t CANCenter;
	uint16_t result;

	CANRange = (uint16_t)((*data_Bdry_Up * canFactor)-(*data_Bdry_Lo * canFactor));
	CANCenter = CANRange >> 1;
	result = CANCenter;

	return result;
}


uint16_t Cal_LinToCan_EVntSidePt_Y(const uint64_t *LINData, const uint64_t *data_Bdry_Up, const uint64_t *data_Bdry_Lo)
{
	uint16_t CANRange;
	uint16_t CANCenter;
//	uint8_t factor = 10;
//	int16_t LINRange = 520;
	uint16_t LINCenter;
	uint16_t CANData;
	uint16_t result;

	CANRange = (uint16_t)((*data_Bdry_Up * canFactor)-(*data_Bdry_Lo * canFactor));
	CANCenter = CANRange >> 1;
	LINCenter = linRangePt_Y >> 1;
	if((*LINData) <= (LINCenter))
	{
		CANData=(uint16_t)((((double)LINCenter-(double)*LINData))*((double)CANRange/(double)linRangePt_Y));
		result = CANData;
	}
	else if(((uint16_t)(4096-LINCenter)<=(*LINData)) && ((*LINData)<=(4096)))
	{
		CANData=(uint16_t)(((double)4096-(double)*LINData)*((double)CANRange/(double)linRangePt_Y));
		result = CANData+CANCenter;
	}
	else
	{
	result=CANCenter;
	}

	return result;
}

uint16_t Cal_LinToCan_EVntSidePt_Reverse_Y(const uint64_t *LINData, const uint64_t *data_Bdry_Up, const uint64_t *data_Bdry_Lo)
{
	uint16_t CANRange;
	uint16_t CANCenter;
//	uint8_t factor = 10;
// int16_t LINRange = 520;
	uint16_t LINCenter;
	uint16_t CANData;
	uint16_t result;

	CANRange = (uint16_t)((*data_Bdry_Up * canFactor)-(*data_Bdry_Lo * canFactor));
	CANCenter = CANRange >> 1;
	LINCenter = linRangePt_Y >> 1;
	if((*LINData)<=(LINCenter))
	{
		CANData=(uint16_t)(((double)LINCenter+(double)*LINData)*((double)CANRange/(double)linRangePt_Y));
		result = CANData;
	}
	else if(((uint16_t)(4096-LINCenter)<=(*LINData)) && ((*LINData)<=(4096)))
	{
		CANData=(uint16_t)(((double)4096-(double)*LINData)*((double)CANRange/(double)linRangePt_Y));
		result = CANCenter-CANData;
	}	
	else
	{
		result=CANCenter;
	}

	return result;
}




/* [] END OF FILE */

