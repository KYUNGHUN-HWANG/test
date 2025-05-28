#define Amo_CALCULATE_C_

#include <stdio.h>

#include "Cpu.h"
#include "Amo_main.h"

volatile int16_t previousValue = 0;
volatile int16_t currentValue = 0;


int16_t Cal_CanToLin_FrDr_SidePt_X(int16_t (*fp)(int16_t, int16_t, int16_t, int16_t, int16_t, float))
{
	int16_t res;

	res = fp(CanLinValue[FrDrSidePt_X].Pt_X.Can_RangePt_Center_X, CanLinValue[FrDrSidePt_X].Pt_X.Can_RangePt_X, CanLinValue[FrDrSidePt_X].Pt_X.Can_EVntPt_X, \
						CanLinValue[FrDrSidePt_X].Pt_X.CanData_Pt_X, linRangePt_X, CanLinValue[FrDrSidePt_X].Pt_X.linFactor_Pt_X);

	return res;
}

int16_t Cal_CanToLin_FrDr_SidePt_Y(int16_t (*fp)(int16_t, int16_t, int16_t, int16_t, int16_t, float))
{
	int16_t res;

	res = fp(CanLinValue[FrDrSidePt_Y].Pt_Y.Can_RangePt_Center_Y, CanLinValue[FrDrSidePt_Y].Pt_Y.Can_RangePt_Y, CanLinValue[FrDrSidePt_Y].Pt_Y.Can_EVntPt_Y, \
					CanLinValue[FrDrSidePt_Y].Pt_Y.CanData_Pt_Y, linRangePt_Y, CanLinValue[FrDrSidePt_Y].Pt_Y.linFactor_Pt_Y);
	
	return res;
}


int16_t Cal_CanToLin_FrDr_CtrPt_X(int16_t (*fp)(int16_t, int16_t, int16_t, int16_t, int16_t, float))
{
	int16_t res;

	res = fp(CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_Center_X, CanLinValue[FrDrCtrPt_X].Pt_X.Can_RangePt_X, CanLinValue[FrDrCtrPt_X].Pt_X.Can_EVntPt_X, \
						CanLinValue[FrDrCtrPt_X].Pt_X.CanData_Pt_X, linRangePt_X, CanLinValue[FrDrCtrPt_X].Pt_X.linFactor_Pt_X);

	return res;
}

int16_t Cal_CanToLin_FrDr_CtrPt_Y(int16_t (*fp)(int16_t, int16_t, int16_t, int16_t, int16_t, float))
{
	int16_t res;

	res = fp(CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_RangePt_Center_Y, CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_RangePt_Y, CanLinValue[FrDrCtrPt_Y].Pt_Y.Can_EVntPt_Y, \
					CanLinValue[FrDrCtrPt_Y].Pt_Y.CanData_Pt_Y, linRangePt_Y, CanLinValue[FrDrCtrPt_Y].Pt_Y.linFactor_Pt_Y);
	
	return res;
}



int16_t Cal_CanToLin_FrPs_SidePt_X(int16_t (*fp)(int16_t, int16_t, int16_t, int16_t, int16_t, float))
{
	int16_t res;

	res = fp(CanLinValue[FrPsSidePt_X].Pt_X.Can_RangePt_Center_X, CanLinValue[FrPsSidePt_X].Pt_X.Can_RangePt_X, CanLinValue[FrPsSidePt_X].Pt_X.Can_EVntPt_X, \
						CanLinValue[FrPsSidePt_X].Pt_X.CanData_Pt_X, linRangePt_X, CanLinValue[FrPsSidePt_X].Pt_X.linFactor_Pt_X);

	return res;
}

int16_t Cal_CanToLin_FrPs_SidePt_Y(int16_t (*fp)(int16_t, int16_t, int16_t, int16_t, int16_t, float))
{
	int16_t res;

	res = fp(CanLinValue[FrPsSidePt_Y].Pt_Y.Can_RangePt_Center_Y, CanLinValue[FrPsSidePt_Y].Pt_Y.Can_RangePt_Y, CanLinValue[FrPsSidePt_Y].Pt_Y.Can_EVntPt_Y, \
					CanLinValue[FrPsSidePt_Y].Pt_Y.CanData_Pt_Y, linRangePt_Y, CanLinValue[FrPsSidePt_Y].Pt_Y.linFactor_Pt_Y);
	
	return res;
}


int16_t Cal_CanToLin_FrPs_CtrPt_X(int16_t (*fp)(int16_t, int16_t, int16_t, int16_t, int16_t, float))
{
	int16_t res;

	res = fp(CanLinValue[FrPsCtrPt_X].Pt_X.Can_RangePt_Center_X, CanLinValue[FrPsCtrPt_X].Pt_X.Can_RangePt_X, CanLinValue[FrPsCtrPt_X].Pt_X.Can_EVntPt_X, \
						CanLinValue[FrPsCtrPt_X].Pt_X.CanData_Pt_X, linRangePt_X, CanLinValue[FrPsCtrPt_X].Pt_X.linFactor_Pt_X);

	return res;
}

int16_t Cal_CanToLin_FrPs_CtrPt_Y(int16_t (*fp)(int16_t, int16_t, int16_t, int16_t, int16_t, float))
{
	int16_t res;

	res = fp(CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_RangePt_Center_Y, CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_RangePt_Y, CanLinValue[FrPsCtrPt_Y].Pt_Y.Can_EVntPt_Y, \
					CanLinValue[FrPsCtrPt_Y].Pt_Y.CanData_Pt_Y, linRangePt_Y, CanLinValue[FrPsCtrPt_Y].Pt_Y.linFactor_Pt_Y);
	
	return res;
}


int16_t Cal_CanToLin_RrDr_ConsPt_X(int16_t (*fp)(int16_t, int16_t, int16_t, int16_t, int16_t, float))
{
	int16_t res;

	res = fp(CanLinValue[RrDrCons_X].Pt_X.Can_RangePt_Center_X, CanLinValue[RrDrCons_X].Pt_X.Can_RangePt_X, CanLinValue[RrDrCons_X].Pt_X.Can_EVntPt_X, \
						CanLinValue[RrDrCons_X].Pt_X.CanData_Pt_X, linRangePt_X, CanLinValue[RrDrCons_X].Pt_X.linFactor_Pt_X);

	return res;
}

int16_t Cal_CanToLin_RrDr_ConsPt_Y(int16_t (*fp)(int16_t, int16_t, int16_t, int16_t, int16_t, float))
{
	int16_t res;

	res = fp(CanLinValue[RrDrCons_Y].Pt_Y.Can_RangePt_Center_Y, CanLinValue[RrDrCons_Y].Pt_Y.Can_RangePt_Y, CanLinValue[RrDrCons_Y].Pt_Y.Can_EVntPt_Y, \
					CanLinValue[RrDrCons_Y].Pt_Y.CanData_Pt_Y, linRangePt_Y, CanLinValue[RrDrCons_Y].Pt_Y.linFactor_Pt_Y);
	
	return res;
}


int16_t Cal_CanToLin_RrPs_ConsPt_X(int16_t (*fp)(int16_t, int16_t, int16_t, int16_t, int16_t, float))
{
	int16_t res;

	res = fp(CanLinValue[RrPsCons_X].Pt_X.Can_RangePt_Center_X, CanLinValue[RrPsCons_X].Pt_X.Can_RangePt_X, CanLinValue[RrPsCons_X].Pt_X.Can_EVntPt_X, \
						CanLinValue[RrPsCons_X].Pt_X.CanData_Pt_X, linRangePt_X, CanLinValue[RrPsCons_X].Pt_X.linFactor_Pt_X);

	return res;
}

int16_t Cal_CanToLin_RrPs_ConsPt_Y(int16_t (*fp)(int16_t, int16_t, int16_t, int16_t, int16_t, float))
{
	int16_t res;

	res = fp(CanLinValue[RrPsCons_Y].Pt_Y.Can_RangePt_Center_Y, CanLinValue[RrPsCons_Y].Pt_Y.Can_RangePt_Y, CanLinValue[RrPsCons_Y].Pt_Y.Can_EVntPt_Y, \
					CanLinValue[RrPsCons_Y].Pt_Y.CanData_Pt_Y, linRangePt_Y, CanLinValue[RrPsCons_Y].Pt_Y.linFactor_Pt_Y);
	
	return res;
}


int16_t Conv_Can_Pt_X(int16_t Can_RangePt_Center, int16_t Can_RangePt, int16_t Can_EVntPt, int16_t CanData_Pt, int16_t LinRangePt_X, float linFactor_Pt)
{
	int16_t res;
	int16_t LinData_Pt;

	Can_RangePt_Center = Can_RangePt >> 1;
	CanData_Pt = (Can_EVntPt - Can_RangePt_Center);
	linFactor_Pt = (float)(LinRangePt_X) / (float)(Can_RangePt);
	#if defined(ACT_ANGLE_UPDATE)
	LinData_Pt = (int16_t)(CanData_Pt * linFactor_Pt) & 0xFFF;  //12bit mask
	#elif defined(ACT_ANGLE_UPDATE_2)
	LinData_Pt = (int16_t)(CanData_Pt * linFactor_Pt) & 0xFFF;
	#else
	LinData_Pt = (int16_t)(CanData_Pt * linFactor_Pt) & 0x7FF;  //11bit mask
	#endif
	res = LinData_Pt;
	
	return res;
}



int16_t Conv_Can_Pt_Y(int16_t Can_RangePt_Center, int16_t Can_RangePt, int16_t Can_EVntPt, int16_t CanData_Pt, int16_t LinRangePt_Y, float linFactor_Pt)
{
	int16_t res;
	int16_t LinData_Pt;
	
	Can_RangePt_Center = Can_RangePt >> 1;
	CanData_Pt = Can_EVntPt - Can_RangePt_Center;
	linFactor_Pt = (float)(LinRangePt_Y) / (float)(Can_RangePt);
	#if defined(ACT_ANGLE_UPDATE)
	LinData_Pt = (int16_t)((~CanData_Pt+1) * linFactor_Pt) & 0xFFF;
	#elif defined(ACT_ANGLE_UPDATE_2)
	LinData_Pt = (int16_t)((~CanData_Pt+1) * linFactor_Pt) & 0xFFF; //2's complement  //12bit mask
	#elif defined(ACT_ANGLE_UPDATE_2)
	LinData_Pt = (int16_t)((~CanData_Pt+1) * linFactor_Pt) & 0xFFF; 
	#else
	LinData_Pt = (int16_t)((~CanData_Pt+1) * linFactor_Pt) & 0x7FF;  //11bit mask
	#endif
	res = LinData_Pt;
	
	return res;
}

int16_t Cal_LinToCan_EVntSidePt_X(uint64_t *LINData, uint64_t *data_Bdry_Up, uint64_t *data_Bdry_Lo)
{
	int16_t CANRange;
	int16_t CANCenter;
//	uint8_t factor = 10;
//	int16_t LINRange = 600;
	int16_t LINCenter;
	int16_t CANData;
	int16_t result;

	CANRange = (*data_Bdry_Up * canFactor)-(*data_Bdry_Lo * canFactor);
	CANCenter = CANRange>>1;
	LINCenter = linRangePt_X>>1;
	if((4096-LINCenter)<=*LINData && *LINData<=4096)  //2048
	{
		CANData=(int16_t)(-((double)4096-*LINData)*((double)CANRange/linRangePt_X));
		result = CANData+CANCenter;
	}
	else if(0<=*LINData && *LINData<=LINCenter)
	{
		CANData=(int16_t)(*LINData)*((double)CANRange/linRangePt_X);
		result = CANData+CANCenter;
	}
	else
	{
	result=CANRange;
	}

	return result;
}


int16_t Cal_CAN_CenterPt(uint64_t *data_Bdry_Up, uint64_t *data_Bdry_Lo)
{
	int16_t CANRange;
	int16_t CANCenter;
	int16_t result;

	CANRange = (*data_Bdry_Up * canFactor)-(*data_Bdry_Lo * canFactor);
	CANCenter = CANRange>>1;
	result = CANCenter;

	return result;
}


int16_t Cal_LinToCan_EVntSidePt_Y(uint64_t *LINData, uint64_t *data_Bdry_Up, uint64_t *data_Bdry_Lo)
{
	int16_t CANRange;
	int16_t CANCenter;
//	uint8_t factor = 10;
//	int16_t LINRange = 520;
	int16_t LINCenter;
	int16_t CANData;
	int16_t result;

	CANRange = (*data_Bdry_Up * canFactor)-(*data_Bdry_Lo * canFactor);
	CANCenter = CANRange>>1;
	LINCenter = linRangePt_Y>>1;
	if(0<=*LINData && *LINData<=LINCenter)
	{
		CANData=(int16_t)((double)LINCenter-*LINData)*((double)CANRange/linRangePt_Y);
		result = CANData;
	}
	else if((4096-LINCenter)<=*LINData && *LINData<=4096)
	{
		CANData=(int16_t)((double)4096-*LINData)*((double)CANRange/linRangePt_Y);
		result = CANData+CANCenter;
	}
	else
	{
	result=CANCenter;
	}

	return result;
}

int16_t Cal_LinToCan_EVntSidePt_Reverse_Y(uint64_t *LINData, uint64_t *data_Bdry_Up, uint64_t *data_Bdry_Lo)
{
	int16_t CANRange;
	int16_t CANCenter;
//	uint8_t factor = 10;
// int16_t LINRange = 520;
	int16_t LINCenter;
	int16_t CANData;
	int16_t result;

	CANRange = (*data_Bdry_Up * canFactor)-(*data_Bdry_Lo * canFactor);
	CANCenter = CANRange>>1;
	LINCenter = linRangePt_Y>>1;
	if(0<=*LINData && *LINData<=LINCenter)
	{
		CANData=(int16_t)((double)LINCenter+*LINData)*((double)CANRange/linRangePt_Y);
		result = CANData;
	}
	else if((4096-LINCenter)<=*LINData && *LINData<=4096)
	{
		CANData=(int16_t)((double)4096-*LINData)*((double)CANRange/linRangePt_Y);
		result = CANCenter-CANData;
	}	
	else
	{
		result=CANCenter;
	}

	return result;
}




/* [] END OF FILE */

