#ifndef AMO_CYCLE_H_
#define AMO_CYCLE_H_


#ifdef	AMO_CYCLE_C_

void lin_FrDrPsCycleMode_task(void);
void lin_FrDrCycleMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_FrDrSwingData_Scenario0(void);
void lin_FrDrSwingData_Scenario1(void);
void lin_FrDrSwingData_Scenario2(void);
void lin_FrDrSwingData_Scenario3(void);
void lin_FrDrSwingData_Scenario4(void);
void lin_FrDrSwingData_Scenario5(void);
void lin_FrDrCycleMode_TimerStop(void);
void lin_FrPsCycleMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void lin_FrPsSwingData_Scenario0(void);
void lin_FrPsSwingData_Scenario1(void);
void lin_FrPsSwingData_Scenario2(void);
void lin_FrPsSwingData_Scenario3(void);
void lin_FrPsSwingData_Scenario4(void);
void lin_FrPsSwingData_Scenario5(void);
void lin_FrPsCycleMode_TimerStop(void);
void lin_RrCycleMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
void lin_RrSwingData_Scenario0(void);
void lin_RrSwingData_Scenario1(void);
void lin_RrSwingData_Scenario2(void);
void lin_RrSwingData_Scenario3(void);
void lin_RrSwingData_Scenario4(void);
void lin_RrSwingData_Scenario5(void);
void lin_RrCycleMode_TimerStop(void);
void Set_LinToCan_Disp_FrDrCycle_XY(void);
uint16_t SET_L_EVNT_HU_CycleFrDrEVntSidePt_X(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_CycleFrDrEVntSidePt_Y(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_CycleFrDrEVntCtrPt_X(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_CycleFrDrEVntCtrPt_Y(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf);
void Set_LinToCan_Disp_FrPsCycle_XY(void);
uint16_t SET_L_EVNT_HU_CycleFrPsEVntSidePt_X(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_CycleFrPsEVntSidePt_Y(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_CycleFrPsEVntCtrPt_X(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_CycleFrPsEVntCtrPt_Y(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf);
void Set_LinToCan_Disp_RrCycle_XY(void);
uint16_t SET_L_EVNT_HU_CycleRrDrEVntSidePt_X(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_CycleRrDrEVntSidePt_Y(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_CycleRrPsEVntCtrPt_X(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf);
uint16_t SET_L_EVNT_HU_CycleRrPsEVntCtrPt_Y(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf);
void OpDone_Check(void);
void OpDone_Check1(void);
void OpDone_Check2(void);
void OpDone_Check3(void);
void OpDone_Check4(void);
void OpDone_Check5(void);
void OpDone_Check6(void);
void OpDone_Check11(void);
void OpDone_Check12(void);
void OpDone_Check13(void);
void OpDone_Check14(void);
void OpDone_Check15(void);
void OpDone_Check16(void);
void OpDone_Check31(void);
void OpDone_Check32(void);
void OpDone_Check33(void);
void OpDone_Check34(void);
void OpDone_Check35(void);
void OpDone_Check36(void);


#else


extern void lin_FrDrPsCycleMode_task(void);
extern void lin_FrDrPsSwingData_Scenario0(void);
extern void lin_FrDrPsSwingData_Scenario1(void);
extern void lin_FrDrPsSwingData_Scenario2(void);
extern void lin_FrDrPsSwingData_Scenario3(void);
extern void lin_FrDrPsSwingData_Scenario4(void);
extern void lin_FrDrPsSwingData_Scenario5(void);
extern void lin_FrDrCycleMode_task(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void lin_FrDrSwingData_Scenario0(void);
extern void lin_FrDrSwingData_Scenario1(void);
extern void lin_FrDrSwingData_Scenario2(void);
extern void lin_FrDrSwingData_Scenario3(void);
extern void lin_FrDrSwingData_Scenario4(void);
extern void lin_FrDrSwingData_Scenario5(void);
extern void lin_FrDrCycleMode_TimerStop(void);
extern void lin_FrPsCycleMode_task(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void lin_FrPsSwingData_Scenario0(void);
extern void lin_FrPsSwingData_Scenario1(void);
extern void lin_FrPsSwingData_Scenario2(void);
extern void lin_FrPsSwingData_Scenario3(void);
extern void lin_FrPsSwingData_Scenario4(void);
extern void lin_FrPsSwingData_Scenario5(void);
extern void lin_FrPsCycleMode_TimerStop(void);
extern void lin_RrCycleMode_task(lin_RC_EVNT_MASTER_CMD_31 *lin_Ctrl);
extern void lin_RrSwingData_Scenario0(void);
extern void lin_RrSwingData_Scenario1(void);
extern void lin_RrSwingData_Scenario2(void);
extern void lin_RrSwingData_Scenario3(void);
extern void lin_RrSwingData_Scenario4(void);
extern void lin_RrSwingData_Scenario5(void);
extern void lin_RrCycleMode_TimerStop(void);
extern void Set_LinToCan_Disp_FrDrCycle_XY(void);
extern uint16_t SET_L_EVNT_HU_CycleFrDrEVntSidePt_X(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_CycleFrDrEVntSidePt_Y(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_CycleFrDrEVntCtrPt_X(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_CycleFrDrEVntCtrPt_Y(uint64_t lin_Ctrl, Atcu_23_data *canBdryDataBuf);
extern void Set_LinToCan_Disp_FrPsCycle_XY(void);
extern uint16_t SET_L_EVNT_HU_CycleFrPsEVntSidePt_X(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_CycleFrPsEVntSidePt_Y(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_CycleFrPsEVntCtrPt_X(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_CycleFrPsEVntCtrPt_Y(uint64_t lin_Ctrl, Atcu_24_data *canBdryDataBuf);
extern void Set_LinToCan_Disp_RrCycle_XY(void);
extern uint16_t SET_L_EVNT_HU_CycleRrDrEVntSidePt_X(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_CycleRrDrEVntSidePt_Y(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_CycleRrPsEVntCtrPt_X(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf);
extern uint16_t SET_L_EVNT_HU_CycleRrPsEVntCtrPt_Y(uint64_t lin_Ctrl, Atcu_25_data *canBdryDataBuf);
extern void OpDone_Check1(void);
extern void OpDone_Check2(void);
extern void OpDone_Check3(void);
extern void OpDone_Check4(void);
extern void OpDone_Check5(void);
extern void OpDone_Check6(void);
extern void OpDone_Check11(void);
extern void OpDone_Check12(void);
extern void OpDone_Check13(void);
extern void OpDone_Check14(void);
extern void OpDone_Check15(void);
extern void OpDone_Check16(void);
extern void OpDone_Check31(void);
extern void OpDone_Check32(void);
extern void OpDone_Check33(void);
extern void OpDone_Check34(void);
extern void OpDone_Check35(void);
extern void OpDone_Check36(void);


#endif /* AMO_CYCLE_C_ */

#endif /* AMO_ADC_H_ */



