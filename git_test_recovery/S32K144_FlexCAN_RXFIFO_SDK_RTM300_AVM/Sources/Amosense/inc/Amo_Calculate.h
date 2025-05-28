#ifndef Amo_CALCULATE_H_
#define Amo_CALCULATE_H_



#ifdef	Amo_CALCULATE_C_

int16_t Cal_CanToLin_FrDr_SidePt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
int16_t Cal_CanToLin_FrDr_SidePt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
int16_t Cal_CanToLin_FrDr_CtrPt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
int16_t Cal_CanToLin_FrDr_CtrPt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
int16_t Cal_CanToLin_FrPs_SidePt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
int16_t Cal_CanToLin_FrPs_SidePt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
int16_t Cal_CanToLin_FrPs_CtrPt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
int16_t Cal_CanToLin_FrPs_CtrPt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
#ifdef AMO_GN7_PE_SETTING_NONE		
int16_t Cal_CanToLin_RrDr_ConsPt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
int16_t Cal_CanToLin_RrDr_ConsPt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
int16_t Cal_CanToLin_RrPs_ConsPt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
int16_t Cal_CanToLin_RrPs_ConsPt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
#endif

int16_t Conv_Can_Pt_X(int16_t Can_RangePt_Center, int16_t Can_RangePt, int16_t Can_EVntPt, int16_t CanData_Pt, int16_t LinRangePt_X, float linFactor_Pt);
int16_t Conv_Can_Pt_Y(int16_t Can_RangePt_Center, int16_t Can_RangePt, int16_t Can_EVntPt, int16_t CanData_Pt, int16_t LinRangePt_Y, float linFactor_Pt);

uint16_t Cal_LinToCan_EVntSidePt_X(const uint64_t *LINData, const uint64_t *data_Bdry_Up, const uint64_t *data_Bdry_Lo);
uint16_t Cal_LinToCan_EVntSidePt_Y(const uint64_t *LINData, const uint64_t *data_Bdry_Up, const uint64_t *data_Bdry_Lo);
uint16_t Cal_LinToCan_EVntSidePt_Reverse_Y(const uint64_t *LINData, const uint64_t *data_Bdry_Up, const uint64_t *data_Bdry_Lo);
uint16_t Cal_CAN_CenterPt(const uint64_t *data_Bdry_Up, const uint64_t *data_Bdry_Lo);




#else

extern int16_t Cal_CanToLin_FrDr_SidePt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
extern int16_t Cal_CanToLin_FrDr_SidePt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
extern int16_t Cal_CanToLin_FrDr_CtrPt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
extern int16_t Cal_CanToLin_FrDr_CtrPt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
extern int16_t Cal_CanToLin_FrPs_SidePt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
extern int16_t Cal_CanToLin_FrPs_SidePt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
extern int16_t Cal_CanToLin_FrPs_CtrPt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
extern int16_t Cal_CanToLin_FrPs_CtrPt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
#ifdef AMO_GN7_PE_SETTING_NONE		
extern int16_t Cal_CanToLin_RrDr_ConsPt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
extern int16_t Cal_CanToLin_RrDr_ConsPt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
extern int16_t Cal_CanToLin_RrPs_ConsPt_X(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
extern int16_t Cal_CanToLin_RrPs_ConsPt_Y(int16_t (*fp)(int16_t param, int16_t param1, int16_t param2, int16_t param3, int16_t param4, float param5));
#endif

extern int16_t Conv_Can_Pt_X(int16_t Can_RangePt_Center, int16_t Can_RangePt, int16_t Can_EVntPt, int16_t CanData_Pt, int16_t LinRangePt_X, float linFactor_Pt);
extern int16_t Conv_Can_Pt_Y(int16_t Can_RangePt_Center, int16_t Can_RangePt, int16_t Can_EVntPt, int16_t CanData_Pt, int16_t LinRangePt_Y, float linFactor_Pt);

extern uint16_t Cal_LinToCan_EVntSidePt_X(const uint64_t *LINData, const uint64_t *data_Bdry_Up, const uint64_t *data_Bdry_Lo);
extern uint16_t Cal_LinToCan_EVntSidePt_Y(const uint64_t *LINData, const uint64_t *data_Bdry_Up, const uint64_t *data_Bdry_Lo);
extern uint16_t Cal_LinToCan_EVntSidePt_Reverse_Y(const uint64_t *LINData, const uint64_t *data_Bdry_Up, const uint64_t *data_Bdry_Lo);
extern uint16_t Cal_CAN_CenterPt(const uint64_t *data_Bdry_Up, const uint64_t *data_Bdry_Lo);

#endif /* Amo_CALCULATE_C_ */

extern volatile int16_t previousValue;
extern volatile int16_t currentValue;

#endif /* Amo_CALCULATE_H_ */

