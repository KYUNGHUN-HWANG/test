#ifndef Amo_STATUS_H_ /* to interprete header file only once */
#define Amo_STATUS_H_


typedef enum {
    EVNT_STATUS_OK = 0,
		EVNT_STATUS_ERROR,
		EVNT_STATUS_ERROR_2,
		EVNT_STATUS_BUSY,
		EVNT_STATUS_TIMEOUT
}Status;

typedef enum {
    E_DEFAULT = 0,
		E_NO_ERROR,
		E_ERROR
}Error_Value;


#ifdef	Amo_STATUS_C_

void Start_Evnt_StatusCheck(void);

void handleOk_FR_SLLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleOk_FR_SLUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleOk_FR_CLLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleOk_FR_CLUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleOk_FR_CRLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, Lvnt_9_data *L9_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleOk_FR_CRUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, Lvnt_9_data *L9_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleOk_FR_SRLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleOk_FR_SRUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);

void handleError_FR_SLLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleError_FR_SLUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleError_FR_CLLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleError_FR_CLUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleError_FR_CRLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, Lvnt_9_data *L9_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleError_FR_CRUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, Lvnt_9_data *L9_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleError_FR_SRLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
void handleError_FR_SRUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);



void handleBusy(Lvnt_3_data *L3_DataBuf);
void handleTimeout(Lvnt_3_data *L3_DataBuf);

void handleStatus_FR_SLLR(Status v_status);
void handleStatus_FR_SLUD(Status v_status);
void handleStatus_FR_CLLR(Status v_status);
void handleStatus_FR_CLUD(Status v_status);
void handleStatus_FR_CRLR(Status v_status);
void handleStatus_FR_CRUD(Status v_status);
void handleStatus_FR_SRLR(Status v_status);
void handleStatus_FR_SRUD(Status v_status);


void Evnt_StatusError_Check(void);

Status Return_Evnt_Fr_SLLR_Status(t_lin_LH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
Status Return_Evnt_Fr_SLUD_Status(t_lin_LH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
Status Return_Evnt_Fr_CLLR_Status(t_lin_LH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
Status Return_Evnt_Fr_CLUD_Status(t_lin_LH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
Status Return_Evnt_Fr_CRLR_Status(t_lin_RH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
Status Return_Evnt_Fr_CRUD_Status(t_lin_RH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
Status Return_Evnt_Fr_SRLR_Status(t_lin_RH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
Status Return_Evnt_Fr_SRUD_Status(t_lin_RH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);

bool Contains_ONE_Fr_sllr(const uint8_t *array, uint8_t size);
bool Contains_ONE_Fr_slud(const uint8_t *array, uint8_t size);
bool Contains_ONE_Fr_cllr(const uint8_t *array, uint8_t size);
bool Contains_ONE_Fr_clud(const uint8_t *array, uint8_t size);
bool Contains_ONE_Fr_crlr(const uint8_t *array, uint8_t size);
bool Contains_ONE_Fr_crud(const uint8_t *array, uint8_t size);
bool Contains_ONE_Fr_srlr(const uint8_t *array, uint8_t size);
bool Contains_ONE_Fr_srud(const uint8_t *array, uint8_t size);

void Read_LIN_Fr_SLLR_Status(t_lin_LH_FR_STATUS *lin_Status);
void Read_LIN_Fr_SLUD_Status(t_lin_LH_FR_STATUS *lin_Status);
void Read_LIN_Fr_CLLR_Status(t_lin_LH_FR_STATUS *lin_Status);
void Read_LIN_Fr_CLUD_Status(t_lin_LH_FR_STATUS *lin_Status);
void Read_LIN_Fr_CRLR_Status(t_lin_RH_FR_STATUS *lin_Status);
void Read_LIN_Fr_CRUD_Status(t_lin_RH_FR_STATUS *lin_Status);
void Read_LIN_Fr_SRLR_Status(t_lin_RH_FR_STATUS *lin_Status);
void Read_LIN_Fr_SRUD_Status(t_lin_RH_FR_STATUS *lin_Status);

bool AllBits_Zero_Fr_SLLR(uint8_t value);
bool AllBits_Zero_Fr_SLUD(uint8_t value);
bool AnyBit_Set_Fr_SLLR(uint8_t value);
bool AnyBit_Set_Fr_SLUD(uint8_t value);
bool AllBits_Zero_Fr_CLLR(uint8_t value);
bool AllBits_Zero_Fr_CLUD(uint8_t value);
bool AnyBit_Set_Fr_CLLR(uint8_t value);
bool AnyBit_Set_Fr_CLUD(uint8_t value);

bool AllBits_Zero_Fr_CRLR(uint8_t value);
bool AllBits_Zero_Fr_CRUD(uint8_t value);
bool AnyBit_Set_Fr_CRLR(uint8_t value);
bool AnyBit_Set_Fr_CRUD(uint8_t value);
bool AllBits_Zero_Fr_SRLR(uint8_t value);
bool AllBits_Zero_Fr_SRUD(uint8_t value);
bool AnyBit_Set_Fr_SRLR(uint8_t value);
bool AnyBit_Set_Fr_SRUD(uint8_t value);


bool AllBits_Zero_Fr_ExtFlt(uint8_t value);




#else
void Start_Evnt_StatusCheck(void);
extern void handleOk_FR_SLLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleOk_FR_SLUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleOk_FR_CLLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleOk_FR_CLUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleOk_FR_CRLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, Lvnt_9_data *L9_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleOk_FR_CRUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, Lvnt_9_data *L9_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleOk_FR_SRLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleOk_FR_SRUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);

extern void handleError_FR_SLLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleError_FR_SLUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleError_FR_CLLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleError_FR_CLUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleError_FR_CRLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, Lvnt_9_data *L9_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleError_FR_CRUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, Lvnt_9_data *L9_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleError_FR_SRLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);
extern void handleError_FR_SRUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag);


extern void handleBusy(Lvnt_3_data *L3_DataBuf);
extern void handleTimeout(Lvnt_3_data *L3_DataBuf);

extern void handleStatus_FR_SLLR(Status v_status);
extern void handleStatus_FR_SLUD(Status v_status);
extern void handleStatus_FR_CLLR(Status v_status);
extern void handleStatus_FR_CLUD(Status v_status);
extern void handleStatus_FR_CRLR(Status v_status);
extern void handleStatus_FR_CRUD(Status v_status);
extern void handleStatus_FR_SRLR(Status v_status);
extern void handleStatus_FR_SRUD(Status v_status);

extern void Evnt_StatusError_Check(void);

extern Status Return_Evnt_Fr_SLLR_Status(t_lin_LH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
extern Status Return_Evnt_Fr_SLUD_Status(t_lin_LH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
extern Status Return_Evnt_Fr_CLLR_Status(t_lin_LH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
extern Status Return_Evnt_Fr_CLUD_Status(t_lin_LH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
extern Status Return_Evnt_Fr_CRLR_Status(t_lin_RH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
extern Status Return_Evnt_Fr_CRUD_Status(t_lin_RH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
extern Status Return_Evnt_Fr_SRLR_Status(t_lin_RH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);
extern Status Return_Evnt_Fr_SRUD_Status(t_lin_RH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag);

extern bool Contains_ONE_Fr_sllr(const uint8_t *array, uint8_t size);
extern bool Contains_ONE_Fr_slud(const uint8_t *array, uint8_t size);
extern bool Contains_ONE_Fr_cllr(const uint8_t *array, uint8_t size);
extern bool Contains_ONE_Fr_clud(const uint8_t *array, uint8_t size);
extern bool Contains_ONE_Fr_crlr(const uint8_t *array, uint8_t size);
extern bool Contains_ONE_Fr_crud(const uint8_t *array, uint8_t size);
extern bool Contains_ONE_Fr_srlr(const uint8_t *array, uint8_t size);
extern bool Contains_ONE_Fr_srud(const uint8_t *array, uint8_t size);

extern void Read_LIN_Fr_SLLR_Status(t_lin_LH_FR_STATUS *lin_Status);
extern void Read_LIN_Fr_SLUD_Status(t_lin_LH_FR_STATUS *lin_Status);
extern void Read_LIN_Fr_CLLR_Status(t_lin_LH_FR_STATUS *lin_Status);
extern void Read_LIN_Fr_CLUD_Status(t_lin_LH_FR_STATUS *lin_Status);
extern void Read_LIN_Fr_CRLR_Status(t_lin_RH_FR_STATUS *lin_Status);
extern void Read_LIN_Fr_CRUD_Status(t_lin_RH_FR_STATUS *lin_Status);
extern void Read_LIN_Fr_SRLR_Status(t_lin_RH_FR_STATUS *lin_Status);
extern void Read_LIN_Fr_SRUD_Status(t_lin_RH_FR_STATUS *lin_Status);

extern bool AllBits_Zero_Fr_SLLR(uint8_t value);
extern bool AllBits_Zero_Fr_SLUD(uint8_t value);
extern bool AnyBit_Set_Fr_SLLR(uint8_t value);
extern bool AnyBit_Set_Fr_SLUD(uint8_t value);
extern bool AllBits_Zero_Fr_CLLR(uint8_t value);
extern bool AllBits_Zero_Fr_CLUD(uint8_t value);
extern bool AnyBit_Set_Fr_CLLR(uint8_t value);
extern bool AnyBit_Set_Fr_CLUD(uint8_t value);

extern bool AllBits_Zero_Fr_CRLR(uint8_t value);
extern bool AllBits_Zero_Fr_CRUD(uint8_t value);
extern bool AnyBit_Set_Fr_CRLR(uint8_t value);
extern bool AnyBit_Set_Fr_CRUD(uint8_t value);
extern bool AllBits_Zero_Fr_SRLR(uint8_t value);
extern bool AllBits_Zero_Fr_SRUD(uint8_t value);
extern bool AnyBit_Set_Fr_SRLR(uint8_t value);
extern bool AnyBit_Set_Fr_SRUD(uint8_t value);

extern bool AllBits_Zero_Fr_ExtFlt(uint8_t value);

#endif /* Amo_STATUS_C_ */

extern uint8_t linbusOff_FltCheck_Start_Flag;

extern uint8_t errorCount_Fr_Sllr;
extern uint8_t errorCount_Fr_Slud;
extern uint8_t errorCount_Fr_Cllr;
extern uint8_t errorCount_Fr_Clud;

extern uint8_t errorCount_Fr_Crlr;
extern uint8_t errorCount_Fr_Crud;
extern uint8_t errorCount_Fr_Srlr;
extern uint8_t errorCount_Fr_Srud;
extern uint8_t flt_onoff;

extern uint8_t linbusOff_FltCheck_Start_Flag;

#endif /* Amo_STATUS_H_ */




