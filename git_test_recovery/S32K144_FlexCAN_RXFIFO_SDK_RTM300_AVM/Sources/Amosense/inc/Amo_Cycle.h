#ifndef AMO_CYCLE_H_
#define AMO_CYCLE_H_

#define	CYCLE_OPDONE
//#define CYCLE_TIMER
//#define NOISE_TEST //10rpm


extern uint8_t FrDrCycleMode_Flag;
extern uint8_t FrPsCycleMode_Flag;
extern uint8_t FrDrPsCycleMode_Flag;


extern volatile bool SLLR_OP_check;
extern volatile bool SLUD_OP_check;
extern volatile bool CLLR_OP_check;
extern volatile bool CLUD_OP_check;
extern volatile bool SRLR_OP_check;
extern volatile bool SRUD_OP_check;
extern volatile bool CRLR_OP_check;
extern volatile bool CRUD_OP_check;

extern int isFirstCall;


#ifdef	AMO_CYCLE_C_

void lin_FrDrPsCycleMode_task(void);
void lin_FrDrPsCycleMode_task_Backup(void);
void lin_FrDrCycleMode_task(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_FrDrCycleMode_task_Backup(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
void lin_FrDrCycleMode_TimerStop(void);
void lin_FrPsCycleMode_task(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void lin_FrPsCycleMode_task_Backup(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
void lin_FrPsCycleMode_TimerStop(void);
void lin_RrCycleMode_TimerStop(void);
void Set_LinToCan_Disp_FrDrCycle_XY(void);
uint16_t SET_CycleFrDrEVntSidePt_X(uint64_t lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_CycleFrDrEVntSidePt_Y(uint64_t lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_CycleFrDrEVntCtrPt_X(uint64_t lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
uint16_t SET_CycleFrDrEVntCtrPt_Y(uint64_t lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
void Set_LinToCan_Disp_FrPsCycle_XY(void);
uint16_t SET_CycleFrPsEVntSidePt_X(uint64_t lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_CycleFrPsEVntSidePt_Y(uint64_t lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_CycleFrPsEVntCtrPt_X(uint64_t lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
uint16_t SET_CycleFrPsEVntCtrPt_Y(uint64_t lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
void lin_CycleMode_CanDisp_TimerStop(void);

void OpDone_Check(void);
void lin_FrDrPsSwingData_Scenario(void);
void lin_FrDrPsSwingData(uint16_t LH_Side_LR, uint16_t LH_Side_UD, uint16_t LH_Ctr_LR, uint16_t LH_Ctr_UD, uint16_t RH_Side_LR, uint16_t RH_Side_UD, uint16_t RH_Ctr_LR, uint16_t RH_Ctr_UD);
void OpDone_Check1(void);
void lin_FrDrSwingData_Scenario(void);
void lin_FrDrSwingData(uint16_t LH_Side_LR, uint16_t LH_Side_UD, uint16_t LH_Ctr_LR, uint16_t LH_Ctr_UD);
void OpDone_Check2(void);
void lin_FrPsSwingData_Scenario(void);
void lin_FrPsSwingData(uint16_t RH_Side_LR, uint16_t RH_Side_UD, uint16_t RH_Ctr_LR, uint16_t RH_Ctr_UD);









#else


extern void lin_FrDrPsCycleMode_task(void);
extern void lin_FrDrPsCycleMode_task_Backup(void);
extern void lin_FrDrCycleMode_task(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void lin_FrDrCycleMode_task_Backup(const lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl);
extern void lin_FrDrCycleMode_TimerStop(void);
extern void lin_FrPsCycleMode_task(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void lin_FrPsCycleMode_task_Backup(const lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl);
extern void lin_FrPsCycleMode_TimerStop(void);
extern void lin_RrCycleMode_TimerStop(void);
extern void Set_LinToCan_Disp_FrDrCycle_XY(void);
extern uint16_t SET_CycleFrDrEVntSidePt_X(uint64_t lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_CycleFrDrEVntSidePt_Y(uint64_t lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_CycleFrDrEVntCtrPt_X(uint64_t lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
extern uint16_t SET_CycleFrDrEVntCtrPt_Y(uint64_t lin_Ctrl, const Atcu_23_data *canBdryDataBuf);
extern void Set_LinToCan_Disp_FrPsCycle_XY(void);
extern uint16_t SET_CycleFrPsEVntSidePt_X(uint64_t lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_CycleFrPsEVntSidePt_Y(uint64_t lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_CycleFrPsEVntCtrPt_X(uint64_t lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
extern uint16_t SET_CycleFrPsEVntCtrPt_Y(uint64_t lin_Ctrl, const Atcu_24_data *canBdryDataBuf);
extern void lin_CycleMode_CanDisp_TimerStop(void);

extern void OpDone_Check(void);
extern void lin_FrDrPsSwingData_Scenario(void);
extern void lin_FrDrPsSwingData(uint16_t LH_Side_LR, uint16_t LH_Side_UD, uint16_t LH_Ctr_LR, uint16_t LH_Ctr_UD, uint16_t RH_Side_LR, uint16_t RH_Side_UD, uint16_t RH_Ctr_LR, uint16_t RH_Ctr_UD);
extern void OpDone_Check1(void);
extern void lin_FrDrSwingData_Scenario(void);
extern void lin_FrDrSwingData(uint16_t LH_Side_LR, uint16_t LH_Side_UD, uint16_t LH_Ctr_LR, uint16_t LH_Ctr_UD);
extern void OpDone_Check2(void);
extern void lin_FrPsSwingData_Scenario(void);
extern void lin_FrPsSwingData(uint16_t RH_Side_LR, uint16_t RH_Side_UD, uint16_t RH_Ctr_LR, uint16_t RH_Ctr_UD);



#endif /* AMO_CYCLE_C_ */

#endif /* AMO_ADC_H_ */



