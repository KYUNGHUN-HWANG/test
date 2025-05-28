#ifndef Amo_LIN_RECOVERY_H_ /* to interprete header file only once */
#define Amo_LIN_RECOVERY_H_


#define	HARD_STOP_ACT 	0x85
#define HARD_STOP_INIT 	0x55

#define OPDONE_TIMEOUT_MANUAL_30_DEGREE_UNDER	1500U // 1s + 0.5s(margin)
#define OPDONE_TIMEOUT_MANUAL_60_DEGREE_UNDER 2000U // 1.5s + 0.5s(margin)
#define OPDONE_TIMEOUT_MANUAL_109_DEGREE_UNDER 2500U //2800U // 2.3s + 0.5s(margin)


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





#ifdef	Amo_LIN_RECOVERY_C_
void Set_LH_Special_Cmd_Init(lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl);
void Set_RH_Special_Cmd_Init(lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl);
void Lin_LH_Scheduler_Normal_Start(void);
void Lin_RH_Scheduler_Normal_Start(void);
void Lin_LH_HardStop(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl);
void Evnt_LH_Special_Cmd(void);
void Evnt_RH_Special_Cmd(void);
void Lin_Scheduler_SpecialCmd_Start(void);
void lin_Read_FR_LH_OpDone(t_lin_LH_FR_STATUS *lin_LH_Status);
void lin_Read_FR_RH_OpDone(t_lin_RH_FR_STATUS *lin_RH_Status);
void lin_Read_RR_OpDone(t_lin_RC_RR_STATUS *lin_RR_Status);
void Opdone_TimeOut_LH_Check(void);
void Opdone_TimeOut_RH_Check(void);
void OpDone_LH_Move_Check(void);
void OpDone_RH_Move_Check(void);





#else
extern void Set_LH_Special_Cmd_Init(lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl);
extern void Set_RH_Special_Cmd_Init(lin_RH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl);
extern void Lin_LH_Scheduler_Normal_Start(void);
extern void Lin_RH_Scheduler_Normal_Start(void);
extern void Lin_LH_HardStop(t_lin_LH_FR_STATUS *lin_LH_Status, lin_LH_EVNT_SPECIAL_CMD_0 *lin_SpecialCtrl, lin_LH_EVNT_MASTER_CMD_11 *lin_LH_Ctrl);
extern void Evnt_LH_Special_Cmd(void);
extern void Evnt_RH_Special_Cmd(void);
extern void Lin_Scheduler_SpecialCmd_Start(void);
extern void lin_Read_FR_LH_OpDone(t_lin_LH_FR_STATUS *lin_LH_Status);
extern void lin_Read_FR_RH_OpDone(t_lin_RH_FR_STATUS *lin_RH_Status);
extern void lin_Read_RR_OpDone(t_lin_RC_RR_STATUS *lin_RR_Status);
extern void Opdone_TimeOut_LH_Check(void);
extern void Opdone_TimeOut_RH_Check(void);
extern void OpDone_LH_Move_Check(void);
extern void OpDone_RH_Move_Check(void);


#endif /* Amo_LIN_RECOVERY_C_ */


#endif /* Amo_LIN_RECOVERY_H_ */



