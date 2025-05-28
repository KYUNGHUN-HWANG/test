#ifndef Amo_LIN_RECOVERY_H_ /* to interprete header file only once */
#define Amo_LIN_RECOVERY_H_


#define	HARD_STOP_ACT 	0x85
#define HARD_STOP_INIT 	0x55
#define LIN_SLAVE_SLEEP	0x00

#define OPDONE_TIMEOUT_MANUAL_30_DEGREE_UNDER	1500U // 1s + 0.5s(margin)
#define OPDONE_TIMEOUT_MANUAL_60_DEGREE_UNDER 2000U // 1.5s + 0.5s(margin)
#define OPDONE_TIMEOUT_MANUAL_109_DEGREE_UNDER 2500U //2800U // 2.3s + 0.5s(margin)
#define OPDONE_TIMEOUT_CYCLE_109_DEGREE_UNDER 3500U // 6.6s + 0.5s(margin)

#define OPDONE_TIMEOUT_MANUAL_160_DEGREE_UNDER 3500U



#define OPDONE_TIMEOUT_CYCLE	3800U //3.8s

#define BROAD_CAST	0x0
#define UNI_CAST		0x1

/*
#define EVNT_ADDR_FR_SLLR	0x11
#define EVNT_ADDR_FR_SLUD	0x12
#define EVNT_ADDR_FR_CLLR	0x13
#define EVNT_ADDR_FR_CLUD	0x14

#define EVNT_ADDR_FR_CRLR	0x21
#define EVNT_ADDR_FR_CRUD	0x22
#define EVNT_ADDR_FR_SRLR	0x23
#define EVNT_ADDR_FR_SRUD	0x24

#define EVNT_ADDR_RR_CLLR	0x31
#define EVNT_ADDR_RR_CLUD	0x32
#define EVNT_ADDR_RR_CRLR	0x33
#define EVNT_ADDR_RR_CRUD	0x34
*/


typedef enum
{
	Evnt_Addr_FR_SLLR = 0x11,
	Evnt_Addr_FR_SLUD = 0x12,
	Evnt_Addr_FR_CLLR = 0x13,
	Evnt_Addr_FR_CLUD = 0x14,
	
	Evnt_Addr_FR_CRLR = 0x21,
	Evnt_Addr_FR_CRUD = 0x22,
	Evnt_Addr_FR_SRLR = 0x23,
	Evnt_Addr_FR_SRUD = 0x24,
	
	Evnt_Addr_RR_CLLR = 0x31,
	Evnt_Addr_RR_CLUD = 0x32,
	Evnt_Addr_RR_CRLR = 0x33,
	Evnt_Addr_RR_CRUD = 0x34
	
}Evnt_Addr_t;


typedef struct
{
	uint16_t FR_SLLR_Recov_Flag : 1;
	uint16_t FR_SLUD_Recov_Flag : 1;
	uint16_t FR_CLLR_Recov_Flag : 1;
	uint16_t FR_CLUD_Recov_Flag : 1;

	uint16_t FR_CRLR_Recov_Flag : 1;
	uint16_t FR_CRUD_Recov_Flag : 1;
	uint16_t FR_SRLR_Recov_Flag : 1;
	uint16_t FR_SRUD_Recov_Flag : 1;
	
	uint16_t RR_CLLR_Recov_Flag : 1;
	uint16_t RR_CLUD_Recov_Flag : 1;
	uint16_t RR_CRLR_Recov_Flag : 1;
	uint16_t RR_CRUD_Recov_Flag : 1;
	uint16_t reserved16 : 4;
}Recovery_flag_t;


typedef enum
{
	Cant_go_to_zeroPt = 1,
	Repeat_zeroPt

}Recovery_Pattern;

typedef enum
{
	minusDegree = 1,
	plusDegree

}Recovery_Pt;




#ifdef	Amo_LIN_RECOVERY_C_
void Set_LH_Special_Cmd_Init(lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl);
void Set_RH_Special_Cmd_Init(lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl);
void Set_RC_Special_Cmd_Init(lin_RC_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl);
void Lin_LH_Scheduler_Normal_Start(void);
void Lin_RH_Scheduler_Normal_Start(void);
void Lin_RR_Scheduler_Normal_Start(void);
void Lin_LH_HardStop(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl);
void Lin_LH_HardStop_1_cllr(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl);
void Lin_LH_HardStop_1_clud(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl);
void Lin_LH_HardStop_1_sllr(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl);
void Lin_LH_HardStop_1_slud(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl);
void Lin_RH_HardStop_1_crlr(t_lin_RH_FR_STATUS *lin_LH_Status, lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RH_EVNT_MASTER_CMD_21 *lin_LH_Ctrl);
void Lin_RH_HardStop_1_crud(t_lin_RH_FR_STATUS *lin_LH_Status, lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RH_EVNT_MASTER_CMD_21 *lin_LH_Ctrl);
void Lin_RH_HardStop_1_srlr(t_lin_RH_FR_STATUS *lin_LH_Status, lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RH_EVNT_MASTER_CMD_21 *lin_LH_Ctrl);
void Lin_RH_HardStop_1_srud(t_lin_RH_FR_STATUS *lin_LH_Status, lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RH_EVNT_MASTER_CMD_21 *lin_LH_Ctrl);

void Lin_RH_HardStop(t_lin_RH_FR_STATUS *lin_RH_Status, lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RH_EVNT_MASTER_CMD_21 *lin_RH_Ctrl);
void Lin_RC_HardStop(t_lin_RC_RR_STATUS *lin_RC_Status, lin_RC_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RC_EVNT_MASTER_CMD_31 *lin_RC_Ctrl);
void Evnt_LH_Special_Cmd(void);
void Evnt_RH_Special_Cmd(void);
void Evnt_RC_Special_Cmd(void);
void Lin_Scheduler_SpecialCmd_Start(void);
void Lin_Scheduler_SpecialCmd_Start_1(void);

void lin_Read_FR_LH_OpDone(t_lin_LH_FR_STATUS *lin_LH_Status);
void lin_Read_FR_RH_OpDone(t_lin_RH_FR_STATUS *lin_RH_Status);
void lin_Read_RR_RC_OpDone(t_lin_RC_RR_STATUS *lin_RR_Status);
void Opdone_TimeOut_LH_Check(void);
void Opdone_TimeOut_RH_Check(void);
void Opdone_TimeOut_RC_Check(void);
void Steploss_check_cllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Steploss_check_clud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Steploss_check_sllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Steploss_check_slud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Steploss_check_crlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Steploss_check_crud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Steploss_check_srlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Steploss_check_srud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);

uint16_t GET_CanCenterPt_CLLR(Atcu_23_data *canBdryDataBuf);
uint16_t GET_CanCenterPt_CLUD(Atcu_23_data *canBdryDataBuf);
uint16_t GET_CanCenterPt_SLLR(Atcu_23_data *canBdryDataBuf);
uint16_t GET_CanCenterPt_SLUD(Atcu_23_data *canBdryDataBuf);
uint16_t GET_CanCenterPt_CRLR(Atcu_24_data *canBdryDataBuf);
uint16_t GET_CanCenterPt_CRUD(Atcu_24_data *canBdryDataBuf);
uint16_t GET_CanCenterPt_SRLR(Atcu_24_data *canBdryDataBuf);
uint16_t GET_CanCenterPt_SRUD(Atcu_24_data *canBdryDataBuf);

void Recovery_Clear_FlagCountReset_cllr(void);
void Recovery_Clear_FlagCountReset_clud(void);
void Recovery_Clear_FlagCountReset_sllr(void);
void Recovery_Clear_FlagCountReset_slud(void);
void Recovery_Clear_FlagCountReset_crlr(void);
void Recovery_Clear_FlagCountReset_crud(void);
void Recovery_Clear_FlagCountReset_srlr(void);
void Recovery_Clear_FlagCountReset_srud(void);

void Handle_Recovery_StopPtCheck_cllr(Lin_EVntPt_t *lin_Pt);
void Handle_Recovery_StopPtCheck_clud(Lin_EVntPt_t *lin_Pt);
void Handle_Recovery_StopPtCheck_sllr(Lin_EVntPt_t *lin_Pt);
void Handle_Recovery_StopPtCheck_slud(Lin_EVntPt_t *lin_Pt);
void Handle_Recovery_StopPtCheck_crlr(Lin_EVntPt_t *lin_Pt);
void Handle_Recovery_StopPtCheck_crud(Lin_EVntPt_t *lin_Pt);
void Handle_Recovery_StopPtCheck_srlr(Lin_EVntPt_t *lin_Pt);
void Handle_Recovery_StopPtCheck_srud(Lin_EVntPt_t *lin_Pt);

void Recovery_Pattern_CantGoToZeroPt_cllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_CantGoToZeroPt_clud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_CantGoToZeroPt_sllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_CantGoToZeroPt_slud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_CantGoToZeroPt_crlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_CantGoToZeroPt_crud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_CantGoToZeroPt_srlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_CantGoToZeroPt_srud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);

void Recovery_Pattern_Repeat_zeroPt_cllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_Repeat_zeroPt_clud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_Repeat_zeroPt_sllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_Repeat_zeroPt_slud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_Repeat_zeroPt_crlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_Repeat_zeroPt_crud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_Repeat_zeroPt_srlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Recovery_Pattern_Repeat_zeroPt_srud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);

void FR_DR_ExtFlt_SET(void);
void FR_PS_ExtFlt_SET(void);

void GO_To_TargetPt_cllr(void);
void GO_To_TargetPt_clud(void);
void GO_To_TargetPt_sllr(void);
void GO_To_TargetPt_slud(void);
void GO_To_TargetPt_crlr(void);
void GO_To_TargetPt_crud(void);
void GO_To_TargetPt_srlr(void);
void GO_To_TargetPt_srud(void);

void Recovery_goToTargetPt_FR_CLLR_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void Recovery_goToTargetPt_FR_CLUD_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void Recovery_goToTargetPt_FR_SLLR_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void Recovery_goToTargetPt_FR_SLUD_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void Recovery_goToTargetPt_FR_CRLR_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void Recovery_goToTargetPt_FR_CRUD_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void Recovery_goToTargetPt_FR_SRLR_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void Recovery_goToTargetPt_FR_SRUD_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);

void goToTargetPt_OpDone_Check_cllr(void);
void goToTargetPt_OpDone_Check_clud(void);
void goToTargetPt_OpDone_Check_sllr(void);
void goToTargetPt_OpDone_Check_slud(void);
void goToTargetPt_OpDone_Check_crlr(void);
void goToTargetPt_OpDone_Check_crud(void);
void goToTargetPt_OpDone_Check_srlr(void);
void goToTargetPt_OpDone_Check_srud(void);

void RecoveryStart_goToZero_FR_CLUD_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RecoveryStart_goToZero_FR_CLLR_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RecoveryStart_goToZero_FR_SLLR_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RecoveryStart_goToZero_FR_SLUD_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RecoveryStart_goToZero_FR_CRLR_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RecoveryStart_goToZero_FR_CRUD_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RecoveryStart_goToZero_FR_SRLR_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
void RecoveryStart_goToZero_FR_SRUD_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);



void Check_300ms_cllr(void);
void Check_300ms_1_cllr(void);
void Check_300ms_2_cllr(void);
void Check_300ms_3_cllr(void);
void Check_300ms_4_cllr(void);
void Check_300ms_5_cllr(void);
void Check_300ms_clud(void);
void Check_300ms_1_clud(void);
void Check_300ms_2_clud(void);
void Check_300ms_3_clud(void);
void Check_300ms_4_clud(void);
void Check_300ms_5_clud(void);
void Check_300ms_sllr(void);
void Check_300ms_1_sllr(void);
void Check_300ms_2_sllr(void);
void Check_300ms_3_sllr(void);
void Check_300ms_4_sllr(void);
void Check_300ms_5_sllr(void);
void Check_300ms_slud(void);
void Check_300ms_1_slud(void);
void Check_300ms_2_slud(void);
void Check_300ms_3_slud(void);
void Check_300ms_4_slud(void);
void Check_300ms_5_slud(void);
void Check_300ms_crlr(void);
void Check_300ms_1_crlr(void);
void Check_300ms_2_crlr(void);
void Check_300ms_3_crlr(void);
void Check_300ms_4_crlr(void);
void Check_300ms_5_crlr(void);
void Check_300ms_crud(void);
void Check_300ms_1_crud(void);
void Check_300ms_2_crud(void);
void Check_300ms_3_crud(void);
void Check_300ms_4_crud(void);
void Check_300ms_5_crud(void);
void Check_300ms_srlr(void);
void Check_300ms_1_srlr(void);
void Check_300ms_2_srlr(void);
void Check_300ms_3_srlr(void);
void Check_300ms_4_srlr(void);
void Check_300ms_5_srlr(void);
void Check_300ms_slud(void);
void Check_300ms_1_slud(void);
void Check_300ms_2_slud(void);
void Check_300ms_3_slud(void);
void Check_300ms_4_slud(void);
void Check_300ms_5_slud(void);
void Handle_RecoveryMode_Steploss(void);

void Steploss_TimeOut_Stop(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
void Check_5000ms_srud(void);




uint16_t absValue(int16_t num);



#else
extern void Set_LH_Special_Cmd_Init(lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl);
extern void Set_RH_Special_Cmd_Init(lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl);
extern void Set_RC_Special_Cmd_Init(lin_RC_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl);
extern void Lin_LH_Scheduler_Normal_Start(void);
extern void Lin_RH_Scheduler_Normal_Start(void);
extern void Lin_RR_Scheduler_Normal_Start(void);
extern void Lin_LH_HardStop(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl);
extern void Lin_LH_HardStop_1(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl);
extern void Lin_LH_HardStop_1_clud(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl);
extern void Lin_RH_HardStop(t_lin_RH_FR_STATUS *lin_RH_Status, lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RH_EVNT_MASTER_CMD_21 *lin_RH_Ctrl);
extern void Lin_RC_HardStop(t_lin_RC_RR_STATUS *lin_RC_Status, lin_RC_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_RC_EVNT_MASTER_CMD_31 *lin_RC_Ctrl);
extern void Evnt_LH_Special_Cmd(void);
extern void Evnt_RH_Special_Cmd(void);
extern void Evnt_RC_Special_Cmd(void);
extern void Lin_Scheduler_SpecialCmd_Start(void);
extern void Lin_Scheduler_SpecialCmd_Start_1(void);

extern void lin_Read_FR_LH_OpDone(t_lin_LH_FR_STATUS *lin_LH_Status);
extern void lin_Read_FR_RH_OpDone(t_lin_RH_FR_STATUS *lin_RH_Status);
extern void lin_Read_RR_RC_OpDone(t_lin_RC_RR_STATUS *lin_RR_Status);
extern void Opdone_TimeOut_LH_Check(void);
extern void Opdone_TimeOut_RH_Check(void);
extern void Opdone_TimeOut_RC_Check(void);
extern void Steploss_check_cllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Steploss_check_clud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Steploss_check_sllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Steploss_check_slud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Steploss_check_crlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Steploss_check_crud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Steploss_check_srlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Steploss_check_srud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);

extern uint16_t GET_CanCenterPt_CLLR(Atcu_23_data *canBdryDataBuf);
extern uint16_t GET_CanCenterPt_CLUD(Atcu_23_data *canBdryDataBuf);
extern uint16_t GET_CanCenterPt_SLLR(Atcu_23_data *canBdryDataBuf);
extern uint16_t GET_CanCenterPt_SLUD(Atcu_23_data *canBdryDataBuf);
extern uint16_t GET_CanCenterPt_CRLR(Atcu_24_data *canBdryDataBuf);
extern uint16_t GET_CanCenterPt_CRUD(Atcu_24_data *canBdryDataBuf);
extern uint16_t GET_CanCenterPt_SRLR(Atcu_24_data *canBdryDataBuf);
extern uint16_t GET_CanCenterPt_SRUD(Atcu_24_data *canBdryDataBuf);

extern void Recovery_Clear_FlagCountReset_cllr(void);
extern void Recovery_Clear_FlagCountReset_clud(void);
extern void Recovery_Clear_FlagCountReset_sllr(void);
extern void Recovery_Clear_FlagCountReset_slud(void);
extern void Recovery_Clear_FlagCountReset_crlr(void);
extern void Recovery_Clear_FlagCountReset_crud(void);
extern void Recovery_Clear_FlagCountReset_srlr(void);
extern void Recovery_Clear_FlagCountReset_srud(void);

extern void Handle_Recovery_StopPtCheck_cllr(Lin_EVntPt_t *lin_Pt);
extern void Handle_Recovery_StopPtCheck_clud(Lin_EVntPt_t *lin_Pt);
extern void Handle_Recovery_StopPtCheck_sllr(Lin_EVntPt_t *lin_Pt);
extern void Handle_Recovery_StopPtCheck_slud(Lin_EVntPt_t *lin_Pt);
extern void Handle_Recovery_StopPtCheck_crlr(Lin_EVntPt_t *lin_Pt);
extern void Handle_Recovery_StopPtCheck_crud(Lin_EVntPt_t *lin_Pt);
extern void Handle_Recovery_StopPtCheck_srlr(Lin_EVntPt_t *lin_Pt);
extern void Handle_Recovery_StopPtCheck_srud(Lin_EVntPt_t *lin_Pt);

extern void Recovery_Pattern_CantGoToZeroPt_cllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_CantGoToZeroPt_clud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_CantGoToZeroPt_sllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_CantGoToZeroPt_slud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_CantGoToZeroPt_crlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_CantGoToZeroPt_crud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_CantGoToZeroPt_srlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_CantGoToZeroPt_srud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);

extern void Recovery_Pattern_Repeat_zeroPt_cllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_Repeat_zeroPt_clud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_Repeat_zeroPt_sllr(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_Repeat_zeroPt_slud(t_lin_LH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_Repeat_zeroPt_crlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_Repeat_zeroPt_crud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_Repeat_zeroPt_srlr(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Recovery_Pattern_Repeat_zeroPt_srud(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);

extern void FR_DR_ExtFlt_SET(void);
extern void FR_PS_ExtFlt_SET(void);

extern void GO_To_TargetPt_cllr(void);
extern void GO_To_TargetPt_clud(void);
extern void GO_To_TargetPt_sllr(void);
extern void GO_To_TargetPt_slud(void);
extern void GO_To_TargetPt_crlr(void);
extern void GO_To_TargetPt_crud(void);
extern void GO_To_TargetPt_srlr(void);
extern void GO_To_TargetPt_srud(void);

extern void Recovery_goToTargetPt_FR_CLLR_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void Recovery_goToTargetPt_FR_CLUD_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void Recovery_goToTargetPt_FR_SLLR_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void Recovery_goToTargetPt_FR_SLUD_1(lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void Recovery_goToTargetPt_FR_CRLR_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void Recovery_goToTargetPt_FR_CRUD_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void Recovery_goToTargetPt_FR_SRLR_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);
extern void Recovery_goToTargetPt_FR_SRUD_1(lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl, Lin_EVntPt_t *lin_Pt);

extern void goToTargetPt_OpDone_Check_cllr(void);
extern void goToTargetPt_OpDone_Check_clud(void);
extern void goToTargetPt_OpDone_Check_sllr(void);
extern void goToTargetPt_OpDone_Check_slud(void);
extern void goToTargetPt_OpDone_Check_crlr(void);
extern void goToTargetPt_OpDone_Check_crud(void);
extern void goToTargetPt_OpDone_Check_srlr(void);
extern void goToTargetPt_OpDone_Check_srud(void);

extern void Check_300ms_cllr(void);
extern void Check_300ms_1_cllr(void);
extern void Check_300ms_2_cllr(void);
extern void Check_300ms_3_cllr(void);
extern void Check_300ms_4_cllr(void);
extern void Check_300ms_5_cllr(void);
extern void Check_300ms_clud(void);
extern void Check_300ms_1_clud(void);
extern void Check_300ms_2_clud(void);
extern void Check_300ms_3_clud(void);
extern void Check_300ms_4_clud(void);
extern void Check_300ms_5_clud(void);
extern void Check_300ms_sllr(void);
extern void Check_300ms_1_sllr(void);
extern void Check_300ms_2_sllr(void);
extern void Check_300ms_3_sllr(void);
extern void Check_300ms_4_sllr(void);
extern void Check_300ms_5_sllr(void);
extern void Check_300ms_slud(void);
extern void Check_300ms_1_slud(void);
extern void Check_300ms_2_slud(void);
extern void Check_300ms_3_slud(void);
extern void Check_300ms_4_slud(void);
extern void Check_300ms_5_slud(void);
extern void Check_300ms_crlr(void);
extern void Check_300ms_1_crlr(void);
extern void Check_300ms_2_crlr(void);
extern void Check_300ms_3_crlr(void);
extern void Check_300ms_4_crlr(void);
extern void Check_300ms_5_crlr(void);
extern void Check_300ms_crud(void);
extern void Check_300ms_1_crud(void);
extern void Check_300ms_2_crud(void);
extern void Check_300ms_3_crud(void);
extern void Check_300ms_4_crud(void);
extern void Check_300ms_5_crud(void);
extern void Check_300ms_srlr(void);
extern void Check_300ms_1_srlr(void);
extern void Check_300ms_2_srlr(void);
extern void Check_300ms_3_srlr(void);
extern void Check_300ms_4_srlr(void);
extern void Check_300ms_5_srlr(void);
extern void Check_300ms_slud(void);
extern void Check_300ms_1_slud(void);
extern void Check_300ms_2_slud(void);
extern void Check_300ms_3_slud(void);
extern void Check_300ms_4_slud(void);
extern void Check_300ms_5_slud(void);

extern uint16_t absValue(int16_t num);
extern void Handle_RecoveryMode_Steploss(void);

extern void Steploss_TimeOut_Stop(t_lin_RH_FR_STATUS *lin_Status, Lin_EVntPt_t *lin_Pt);
extern void Check_5000ms_srud(void);

#endif /* Amo_LIN_RECOVERY_C_ */


#endif /* Amo_LIN_RECOVERY_H_ */



