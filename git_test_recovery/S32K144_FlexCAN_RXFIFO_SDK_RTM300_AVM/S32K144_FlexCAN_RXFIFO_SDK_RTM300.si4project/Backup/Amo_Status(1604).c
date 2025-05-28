#define Amo_STATUS_C_

#include "Cpu.h"
#include "Amo_main.h"
#include "Amo_timer.h"

uint8_t errorCount_Fr_Sllr = 0;
uint8_t errorCount_Fr_Slud = 0;
uint8_t errorCount_Fr_Cllr = 0;
uint8_t errorCount_Fr_Clud = 0;

uint8_t errorCount_Fr_Crlr = 0;
uint8_t errorCount_Fr_Crud = 0;
uint8_t errorCount_Fr_Srlr = 0;
uint8_t errorCount_Fr_Srud = 0;



//uint8_t fr_sllr_comFlt_Flag = true;
//uint8_t interFlt_Flag = true;


extern Lvnt_1_data Can_Tx_Evnt_1;
extern Lvnt_2_data Can_Tx_Evnt_2;
extern Lvnt_3_data Can_Tx_Evnt_3;
extern Lvnt_4_data Can_Tx_Evnt_4;
extern Lvnt_5_data Can_Tx_Evnt_5;
extern Lvnt_6_data Can_Tx_Evnt_6;
extern Lvnt_7_data Can_Tx_Evnt_7;
extern Lvnt_8_data Can_Tx_Evnt_8;
extern Lvnt_9_data Can_Tx_Evnt_9;

//extern uint16_t selectEVnt;

bool AllBits_Zero_Fr_SLLR(uint8_t value) { return value == 0; }
bool AllBits_Zero_Fr_SLUD(uint8_t value) { return value == 0; }
bool AnyBit_Set_Fr_SLLR(uint8_t value) { return value != 0; }
bool AnyBit_Set_Fr_SLUD(uint8_t value) { return value != 0; }

bool AllBits_Zero_Fr_CLLR(uint8_t value) { return value == 0; }
bool AllBits_Zero_Fr_CLUD(uint8_t value) { return value == 0; }
bool AnyBit_Set_Fr_CLLR(uint8_t value) { return value != 0; }
bool AnyBit_Set_Fr_CLUD(uint8_t value) { return value != 0; }

bool AllBits_Zero_Fr_CRLR(uint8_t value) { return value == 0; }
bool AllBits_Zero_Fr_CRUD(uint8_t value) { return value == 0; }
bool AnyBit_Set_Fr_CRLR(uint8_t value) { return value != 0; }
bool AnyBit_Set_Fr_CRUD(uint8_t value) { return value != 0; }

bool AllBits_Zero_Fr_SRLR(uint8_t value) { return value == 0; }
bool AllBits_Zero_Fr_SRUD(uint8_t value) { return value == 0; }
bool AnyBit_Set_Fr_SRLR(uint8_t value) { return value != 0; }
bool AnyBit_Set_Fr_SRUD(uint8_t value) { return value != 0; }


bool AllBits_Zero_Fr_ExtFlt(uint8_t value) { return value == 0; }


void Start_Evnt_StatusCheck(void)
{
	Amo_timer_Stop(timer_22);
	Amo_timer_Start(timer_22, 50, true, Evnt_StatusError_Check);
}

void handleOk_FR_SLLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AllBits_Zero_Fr_SLLR(fault_flag->FR_DR_SD_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrDrEvntSideFlt = E_NO_ERROR;
	}
	
	if(fault_flag->FR_DR_SD_FLT.fltBit.FR_SLLR_IntFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_DrvSdHorIntFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_DR_SD_FLT.fltBit.FR_SLLR_ExtFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_DrvSdHorExtFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_DrvSdHorComFlt = E_NO_ERROR;
	}
	
}

void handleOk_FR_SLUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AllBits_Zero_Fr_SLUD(fault_flag->FR_DR_SD_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrDrEvntSideFlt = E_NO_ERROR;
	}
	
	if(fault_flag->FR_DR_SD_FLT.fltBit.FR_SLUD_IntFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_DrvSdVertIntFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_DR_SD_FLT.fltBit.FR_SLUD_ExtFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_DrvSdVertExtFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_DrvSdVertComFlt = E_NO_ERROR;
	}
	
}


void handleOk_FR_CLLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AllBits_Zero_Fr_CLLR(fault_flag->FR_DR_CTR_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrDrEvntCtrFlt = E_NO_ERROR;
	}
	
	if(fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLLR_IntFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_DrvCntrHorIntFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLLR_ExtFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_DrvCntrHorExtFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_DrvCntrHorComFlt = E_NO_ERROR;
	}
	
}

void handleOk_FR_CLUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AllBits_Zero_Fr_CLUD(fault_flag->FR_DR_CTR_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrDrEvntCtrFlt = E_NO_ERROR;
	}
	
	if(fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLUD_IntFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_DrvCntrVertIntFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLUD_ExtFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_DrvCntrVertExtFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_DrvCntrVertComFlt = E_NO_ERROR;
	}
	
}

void handleOk_FR_CRLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, Lvnt_9_data *L9_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AllBits_Zero_Fr_CRLR(fault_flag->FR_PS_CTR_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrPsEvntCtrFlt = E_NO_ERROR;
	}
	
	if(fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRLR_IntFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_AstCntrHorIntFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRLR_ExtFlt_Flag == false)
	{
		L9_DataBuf->data.L_EVNT_AstCntrHorExtFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_AstCntrHorComFlt = E_NO_ERROR;
	}
	
}

void handleOk_FR_CRUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, Lvnt_9_data *L9_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AllBits_Zero_Fr_CRUD(fault_flag->FR_PS_CTR_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrPsEvntCtrFlt = E_NO_ERROR;
	}
	
	if(fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRUD_IntFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_AstSdVertIntFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRUD_ExtFlt_Flag == false)
	{
		L9_DataBuf->data.L_EVNT_AstCntrVertExtFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_AstCntrVertComFlt = E_NO_ERROR;
	}
	
}


void handleOk_FR_SRLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AllBits_Zero_Fr_SRLR(fault_flag->FR_PS_SD_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrPsEvntSideFlt = E_NO_ERROR;
	}
	
	if(fault_flag->FR_PS_SD_FLT.fltBit.FR_SRLR_IntFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_AstSdHorIntFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_PS_SD_FLT.fltBit.FR_SRLR_ExtFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_AstSdHorExtFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_AstSdHorComFlt = E_NO_ERROR;
	}
	
}

void handleOk_FR_SRUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AllBits_Zero_Fr_SRUD(fault_flag->FR_PS_SD_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrPsEvntSideFlt = E_NO_ERROR;
	}
	
	if(fault_flag->FR_PS_SD_FLT.fltBit.FR_SRUD_IntFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_AstSdVertIntFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_PS_SD_FLT.fltBit.FR_SRUD_ExtFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_AstSdVertExtFlt = E_NO_ERROR;
	}

	if(fault_flag->FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag == false)
	{
		L8_DataBuf->data.L_EVNT_AstSdVertComFlt = E_NO_ERROR;
	}
}


void handleError_FR_SLLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AnyBit_Set_Fr_SLLR(fault_flag->FR_DR_SD_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrDrEvntSideFlt = E_ERROR;
	}
	if(fault_flag->FR_DR_SD_FLT.fltBit.FR_SLLR_IntFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_DrvSdHorIntFlt = E_ERROR;
	}
	if(fault_flag->FR_DR_SD_FLT.fltBit.FR_SLLR_ExtFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_DrvSdHorExtFlt = E_ERROR;
	}
	if(fault_flag->FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_DrvSdHorComFlt = E_ERROR;
	}
}

void handleError_FR_SLUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AnyBit_Set_Fr_SLUD(fault_flag->FR_DR_SD_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrDrEvntSideFlt = E_ERROR;
	}
	if(fault_flag->FR_DR_SD_FLT.fltBit.FR_SLUD_IntFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_DrvSdVertIntFlt = E_ERROR;
	}
	if(fault_flag->FR_DR_SD_FLT.fltBit.FR_SLUD_ExtFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_DrvSdVertExtFlt = E_ERROR;
	}
	if(fault_flag->FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_DrvSdVertComFlt = E_ERROR;
	}
}


void handleError_FR_CLLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AnyBit_Set_Fr_CLLR(fault_flag->FR_DR_CTR_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrDrEvntCtrFlt = E_ERROR;
	}
	if(fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLLR_IntFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_DrvCntrHorIntFlt = E_ERROR;
	}
	if(fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLLR_ExtFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_DrvCntrHorExtFlt = E_ERROR;
	}
	if(fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_DrvCntrHorComFlt = E_ERROR;
	}
}

void handleError_FR_CLUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AnyBit_Set_Fr_CLUD(fault_flag->FR_DR_CTR_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrDrEvntCtrFlt = E_ERROR;
	}
	if(fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLUD_IntFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_DrvCntrVertIntFlt = E_ERROR;
	}
	if(fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLUD_ExtFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_DrvCntrVertExtFlt = E_ERROR;
	}
	if(fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_DrvCntrVertComFlt = E_ERROR;
	}
}

void handleError_FR_CRLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, Lvnt_9_data *L9_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AnyBit_Set_Fr_CRLR(fault_flag->FR_PS_CTR_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrPsEvntCtrFlt = E_ERROR;
	}
	if(fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRLR_IntFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_AstCntrHorIntFlt = E_ERROR;
	}
	if(fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRLR_ExtFlt_Flag == true)
	{
		L9_DataBuf->data.L_EVNT_AstCntrHorExtFlt = E_ERROR;
	}
	if(fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_AstCntrHorComFlt = E_ERROR;
	}
}

void handleError_FR_CRUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, Lvnt_9_data *L9_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AnyBit_Set_Fr_CRUD(fault_flag->FR_PS_CTR_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrPsEvntCtrFlt = E_ERROR;
	}
	if(fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRUD_IntFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_AstCntrVertIntFlt = E_ERROR;
	}
	if(fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRUD_ExtFlt_Flag == true)
	{
		L9_DataBuf->data.L_EVNT_AstCntrVertExtFlt = E_ERROR;
	}
	if(fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_AstCntrVertComFlt = E_ERROR;
	}
}

void handleError_FR_SRLR(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AnyBit_Set_Fr_SRLR(fault_flag->FR_PS_SD_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrPsEvntSideFlt = E_ERROR;
	}
	if(fault_flag->FR_PS_SD_FLT.fltBit.FR_SRLR_IntFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_AstSdHorIntFlt = E_ERROR;
	}
	if(fault_flag->FR_PS_SD_FLT.fltBit.FR_SRLR_ExtFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_AstSdHorExtFlt = E_ERROR;
	}
	if(fault_flag->FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_AstSdHorComFlt = E_ERROR;
	}
}

void handleError_FR_SRUD(Lvnt_3_data *L3_DataBuf, Lvnt_8_data *L8_DataBuf, FR_DTC_Flt_flag_t *fault_flag)
{
	if(AnyBit_Set_Fr_SRUD(fault_flag->FR_PS_SD_FLT.fltByte))
	{
		L3_DataBuf->data.L_EVNT_FrPsEvntSideFlt = E_ERROR;
	}
	if(fault_flag->FR_PS_SD_FLT.fltBit.FR_SRUD_IntFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_AstSdVertIntFlt = E_ERROR;
	}
	if(fault_flag->FR_PS_SD_FLT.fltBit.FR_SRUD_ExtFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_AstSdVertExtFlt = E_ERROR;
	}
	if(fault_flag->FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag == true)
	{
		L8_DataBuf->data.L_EVNT_AstSdVertComFlt = E_ERROR;
	}
}


void handleBusy(Lvnt_3_data *L3_DataBuf)
{
}

void handleTimeout(Lvnt_3_data *L3_DataBuf)
{
}

void handleStatus_FR_SLLR(Status status)
{
    switch (status)
    {
        case EVNT_STATUS_OK:
					
					errorCount_Fr_Sllr = 0;

					handleOk_FR_SLLR(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &FR_DTC_Flt_flag);
					//printf("Status: OK\n");
	        break;
						
        case EVNT_STATUS_ERROR:

//					PINS_DRV_SetPins(GPIO_PORTA, TEST_OUT1_MASK);

					errorCount_Fr_Sllr++;

					if(errorCount_Fr_Sllr >= 21) //21: 1000ms
					{
						handleError_FR_SLLR(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &FR_DTC_Flt_flag);
					}
          break;
						
        case EVNT_STATUS_BUSY:
          break;
						
        case EVNT_STATUS_TIMEOUT:
          break;
						
        default:
//            printf("Status: UNKNOWN\n");
          break;
    }
}


void handleStatus_FR_SLUD(Status status)
{
    switch (status)
    {
        case EVNT_STATUS_OK:
					
					errorCount_Fr_Slud = 0;

					handleOk_FR_SLUD(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &FR_DTC_Flt_flag);
	        break;
						
        case EVNT_STATUS_ERROR:
					
					errorCount_Fr_Slud++;

					if(errorCount_Fr_Slud >= 21) //21: 1000ms
					{
						handleError_FR_SLUD(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &FR_DTC_Flt_flag);
					}
          break;
						
        case EVNT_STATUS_BUSY:
          break;
						
        case EVNT_STATUS_TIMEOUT:
          break;
						
        default:
          break;
    }
}

void handleStatus_FR_CLLR(Status status)
{
    switch (status)
    {
        case EVNT_STATUS_OK:
					
					errorCount_Fr_Cllr = 0;

					handleOk_FR_CLLR(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &FR_DTC_Flt_flag);
	        break;
						
        case EVNT_STATUS_ERROR:
					
					errorCount_Fr_Cllr++;

					if(errorCount_Fr_Cllr >= 21) //21: 1000ms
					{
						handleError_FR_CLLR(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &FR_DTC_Flt_flag);

					}
          break;
						
        case EVNT_STATUS_BUSY:
          break;
						
        case EVNT_STATUS_TIMEOUT:
          break;
						
        default:
          break;
    }
}

void handleStatus_FR_CLUD(Status status)
{
    switch (status)
    {
        case EVNT_STATUS_OK:
					
					errorCount_Fr_Clud = 0;

					handleOk_FR_CLUD(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &FR_DTC_Flt_flag);
	        break;
						
        case EVNT_STATUS_ERROR:
					
					errorCount_Fr_Clud++;

					if(errorCount_Fr_Clud >= 21) //21: 1000ms
					{
						handleError_FR_CLUD(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &FR_DTC_Flt_flag);
					}
          break;
						
        case EVNT_STATUS_BUSY:
          break;
						
        case EVNT_STATUS_TIMEOUT:
          break;
						
        default:
          break;
    }
}

void handleStatus_FR_CRLR(Status status)
{
    switch (status)
    {
        case EVNT_STATUS_OK:
					
					errorCount_Fr_Crlr = 0;

					handleOk_FR_CRLR(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &Can_Tx_Evnt_9, &FR_DTC_Flt_flag);
					//printf("Status: OK\n");
	        break;
						
        case EVNT_STATUS_ERROR:

//					PINS_DRV_SetPins(GPIO_PORTA, TEST_OUT1_MASK);

					errorCount_Fr_Crlr++;

					if(errorCount_Fr_Crlr >= 21) //21: 1000ms
					{
						handleError_FR_CRLR(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &Can_Tx_Evnt_9, &FR_DTC_Flt_flag);
					}
          break;
						
        case EVNT_STATUS_BUSY:
          break;
						
        case EVNT_STATUS_TIMEOUT:
          break;
						
        default:
//            printf("Status: UNKNOWN\n");
          break;
    }
}


void handleStatus_FR_CRUD(Status status)
{
    switch (status)
    {
        case EVNT_STATUS_OK:
					
					errorCount_Fr_Crud = 0;

					handleOk_FR_CRUD(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &Can_Tx_Evnt_9, &FR_DTC_Flt_flag);
	        break;
						
        case EVNT_STATUS_ERROR:
					
					errorCount_Fr_Crud++;

					if(errorCount_Fr_Crud >= 21) //21: 1000ms
					{
						handleError_FR_CRUD(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &Can_Tx_Evnt_9, &FR_DTC_Flt_flag);
					}
          break;
						
        case EVNT_STATUS_BUSY:
          break;
						
        case EVNT_STATUS_TIMEOUT:
          break;
						
        default:
          break;
    }
}

void handleStatus_FR_SRLR(Status status)
{
    switch (status)
    {
        case EVNT_STATUS_OK:
					
					errorCount_Fr_Srlr = 0;

					handleOk_FR_SRLR(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &FR_DTC_Flt_flag);
	        break;
						
        case EVNT_STATUS_ERROR:
					
					errorCount_Fr_Srlr++;

					if(errorCount_Fr_Srlr >= 21) //21: 1000ms
					{
						handleError_FR_SRLR(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &FR_DTC_Flt_flag);

					}
          break;
						
        case EVNT_STATUS_BUSY:
          break;
						
        case EVNT_STATUS_TIMEOUT:
          break;
						
        default:
          break;
    }
}

void handleStatus_FR_SRUD(Status status)
{
    switch (status)
    {
        case EVNT_STATUS_OK:
					
					errorCount_Fr_Srud = 0;

					handleOk_FR_SRUD(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &FR_DTC_Flt_flag);
	        break;
						
        case EVNT_STATUS_ERROR:
					
					errorCount_Fr_Srud++;

					if(errorCount_Fr_Srud >= 21) //21: 1000ms
					{
						handleError_FR_SRUD(&Can_Tx_Evnt_3, &Can_Tx_Evnt_8, &FR_DTC_Flt_flag);
					}
          break;
						
        case EVNT_STATUS_BUSY:
          break;
						
        case EVNT_STATUS_TIMEOUT:
          break;
						
        default:
          break;
    }
}



bool Contains_ONE_Fr_sllr(const uint8_t *array, uint8_t size)
{
	for(uint8_t i = 0; i < size; i++)
	{
	    if ((array[i] == EVNT_STATUS_ERROR) || (array[i] == EVNT_STATUS_ERROR_2)) 
			{
	        return true;
	    }
	}
	return false;
}

bool Contains_ONE_Fr_slud(const uint8_t *array, uint8_t size)
{
	for(uint8_t i = 0; i < size; i++)
	{
	    if ((array[i] == EVNT_STATUS_ERROR) || (array[i] == EVNT_STATUS_ERROR_2)) 
			{
	        return true;
	    }
	}
	return false;
}

bool Contains_ONE_Fr_cllr(const uint8_t *array, uint8_t size)
{
	for(uint8_t i = 0; i < size; i++)
	{
	    if ((array[i] == EVNT_STATUS_ERROR) || (array[i] == EVNT_STATUS_ERROR_2)) 
			{
	        return true;
	    }
	}
	return false;
}

bool Contains_ONE_Fr_clud(const uint8_t *array, uint8_t size)
{
	for(uint8_t i = 0; i < size; i++)
	{
	    if ((array[i] == EVNT_STATUS_ERROR) || (array[i] == EVNT_STATUS_ERROR_2)) 
			{
	        return true;
	    }
	}
	return false;
}

bool Contains_ONE_Fr_crlr(const uint8_t *array, uint8_t size)
{
	for(uint8_t i = 0; i < size; i++)
	{
	    if ((array[i] == EVNT_STATUS_ERROR) || (array[i] == EVNT_STATUS_ERROR_2)) 
			{
	        return true;
	    }
	}
	return false;
}

bool Contains_ONE_Fr_crud(const uint8_t *array, uint8_t size)
{
	for(uint8_t i = 0; i < size; i++)
	{
	    if ((array[i] == EVNT_STATUS_ERROR) || (array[i] == EVNT_STATUS_ERROR_2)) 
			{
	        return true;
	    }
	}
	return false;
}

bool Contains_ONE_Fr_srlr(const uint8_t *array, uint8_t size)
{
	for(uint8_t i = 0; i < size; i++)
	{
	    if ((array[i] == EVNT_STATUS_ERROR) || (array[i] == EVNT_STATUS_ERROR_2)) 
			{
	        return true;
	    }
	}
	return false;
}

bool Contains_ONE_Fr_srud(const uint8_t *array, uint8_t size)
{
	for(uint8_t i = 0; i < size; i++)
	{
	    if ((array[i] == EVNT_STATUS_ERROR) || (array[i] == EVNT_STATUS_ERROR_2)) 
			{
	        return true;
	    }
	}
	return false;
}



void Read_LIN_Fr_SLLR_Status(t_lin_LH_FR_STATUS *lin_Status)
{
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_EIDef = l_bool_rd_LI0_FR_SLLR_EIDef();
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_TSD = l_bool_rd_LI0_FR_SLLR_TSD();
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_STATE = l_bool_rd_LI0_FR_SLLR_STATE();
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_Sensor = l_bool_rd_LI0_FR_SLLR_Sensor();
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_OV = l_bool_rd_LI0_FR_SLLR_OV();
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_UV = l_bool_rd_LI0_FR_SLLR_UV();
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN2 = l_bool_rd_LI0_FR_SLLR_OPEN2();
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN1 = l_bool_rd_LI0_FR_SLLR_OPEN1();
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC2 = l_bool_rd_LI0_FR_SLLR_OVC2();
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC1 = l_bool_rd_LI0_FR_SLLR_OVC1();
}

void Read_LIN_Fr_SLUD_Status(t_lin_LH_FR_STATUS *lin_Status)
{
	lin_Status->fr_slud_status.LDATA.FR_SLUD_EIDef = l_bool_rd_LI0_FR_SLUD_EIDef();
	lin_Status->fr_slud_status.LDATA.FR_SLUD_TSD = l_bool_rd_LI0_FR_SLUD_TSD();
	lin_Status->fr_slud_status.LDATA.FR_SLUD_STATE = l_bool_rd_LI0_FR_SLUD_STATE();
	lin_Status->fr_slud_status.LDATA.FR_SLUD_Sensor = l_bool_rd_LI0_FR_SLUD_Sensor();
	lin_Status->fr_slud_status.LDATA.FR_SLUD_OV = l_bool_rd_LI0_FR_SLUD_OV();
	lin_Status->fr_slud_status.LDATA.FR_SLUD_UV = l_bool_rd_LI0_FR_SLUD_UV();
	lin_Status->fr_slud_status.LDATA.FR_SLUD_OPEN2 = l_bool_rd_LI0_FR_SLUD_OPEN2();
	lin_Status->fr_slud_status.LDATA.FR_SLUD_OPEN1 = l_bool_rd_LI0_FR_SLUD_OPEN1();
	lin_Status->fr_slud_status.LDATA.FR_SLUD_OVC2 = l_bool_rd_LI0_FR_SLUD_OVC2();
	lin_Status->fr_slud_status.LDATA.FR_SLUD_OVC1 = l_bool_rd_LI0_FR_SLUD_OVC1();
}

void Read_LIN_Fr_CLLR_Status(t_lin_LH_FR_STATUS *lin_Status)
{
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_EIDef = l_bool_rd_LI0_FR_CLLR_EIDef();
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_TSD = l_bool_rd_LI0_FR_CLLR_TSD();
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_STATE = l_bool_rd_LI0_FR_CLLR_STATE();
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_Sensor = l_bool_rd_LI0_FR_CLLR_Sensor();
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_OV = l_bool_rd_LI0_FR_CLLR_OV();
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_UV = l_bool_rd_LI0_FR_CLLR_UV();
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_OPEN2 = l_bool_rd_LI0_FR_CLLR_OPEN2();
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_OPEN1 = l_bool_rd_LI0_FR_CLLR_OPEN1();
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_OVC2 = l_bool_rd_LI0_FR_CLLR_OVC2();
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_OVC1 = l_bool_rd_LI0_FR_CLLR_OVC1();
}

void Read_LIN_Fr_CLUD_Status(t_lin_LH_FR_STATUS *lin_Status)
{
	lin_Status->fr_clud_status.LDATA.FR_CLUD_EIDef = l_bool_rd_LI0_FR_CLUD_EIDef();
	lin_Status->fr_clud_status.LDATA.FR_CLUD_TSD = l_bool_rd_LI0_FR_CLUD_TSD();
	lin_Status->fr_clud_status.LDATA.FR_CLUD_STATE = l_bool_rd_LI0_FR_CLUD_STATE();
	lin_Status->fr_clud_status.LDATA.FR_CLUD_Sensor = l_bool_rd_LI0_FR_CLUD_Sensor();
	lin_Status->fr_clud_status.LDATA.FR_CLUD_OV = l_bool_rd_LI0_FR_CLUD_OV();
	lin_Status->fr_clud_status.LDATA.FR_CLUD_UV = l_bool_rd_LI0_FR_CLUD_UV();
	lin_Status->fr_clud_status.LDATA.FR_CLUD_OPEN2 = l_bool_rd_LI0_FR_CLUD_OPEN2();
	lin_Status->fr_clud_status.LDATA.FR_CLUD_OPEN1 = l_bool_rd_LI0_FR_CLUD_OPEN1();
	lin_Status->fr_clud_status.LDATA.FR_CLUD_OVC2 = l_bool_rd_LI0_FR_CLUD_OVC2();
	lin_Status->fr_clud_status.LDATA.FR_CLUD_OVC1 = l_bool_rd_LI0_FR_CLUD_OVC1();
}

void Read_LIN_Fr_CRLR_Status(t_lin_RH_FR_STATUS *lin_Status)
{
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_EIDef = l_bool_rd_LI1_FR_CRLR_EIDef();
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_TSD = l_bool_rd_LI1_FR_CRLR_TSD();
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_STATE = l_bool_rd_LI1_FR_CRLR_STATE();
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_Sensor = l_bool_rd_LI1_FR_CRLR_Sensor();
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_OV = l_bool_rd_LI1_FR_CRLR_OV();
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_UV = l_bool_rd_LI1_FR_CRLR_UV();
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_OPEN2 = l_bool_rd_LI1_FR_CRLR_OPEN2();
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_OPEN1 = l_bool_rd_LI1_FR_CRLR_OPEN1();
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_OVC2 = l_bool_rd_LI1_FR_CRLR_OVC2();
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_OVC1 = l_bool_rd_LI1_FR_CRLR_OVC1();
}

void Read_LIN_Fr_CRUD_Status(t_lin_RH_FR_STATUS *lin_Status)
{
	lin_Status->fr_crud_status.LDATA.FR_CRUD_EIDef = l_bool_rd_LI1_FR_CRUD_EIDef();
	lin_Status->fr_crud_status.LDATA.FR_CRUD_TSD = l_bool_rd_LI1_FR_CRUD_TSD();
	lin_Status->fr_crud_status.LDATA.FR_CRUD_STATE = l_bool_rd_LI1_FR_CRUD_STATE();
	lin_Status->fr_crud_status.LDATA.FR_CRUD_Sensor = l_bool_rd_LI1_FR_CRUD_Sensor();
	lin_Status->fr_crud_status.LDATA.FR_CRUD_OV = l_bool_rd_LI1_FR_CRUD_OV();
	lin_Status->fr_crud_status.LDATA.FR_CRUD_UV = l_bool_rd_LI1_FR_CRUD_UV();
	lin_Status->fr_crud_status.LDATA.FR_CRUD_OPEN2 = l_bool_rd_LI1_FR_CRUD_OPEN2();
	lin_Status->fr_crud_status.LDATA.FR_CRUD_OPEN1 = l_bool_rd_LI1_FR_CRUD_OPEN1();
	lin_Status->fr_crud_status.LDATA.FR_CRUD_OVC2 = l_bool_rd_LI1_FR_CRUD_OVC2();
	lin_Status->fr_crud_status.LDATA.FR_CRUD_OVC1 = l_bool_rd_LI1_FR_CRUD_OVC1();
}

void Read_LIN_Fr_SRLR_Status(t_lin_RH_FR_STATUS *lin_Status)
{
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_EIDef = l_bool_rd_LI1_FR_SRLR_EIDef();
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_TSD = l_bool_rd_LI1_FR_SRLR_TSD();
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_STATE = l_bool_rd_LI1_FR_SRLR_STATE();
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_Sensor = l_bool_rd_LI1_FR_SRLR_Sensor();
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_OV = l_bool_rd_LI1_FR_SRLR_OV();
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_UV = l_bool_rd_LI1_FR_SRLR_UV();
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_OPEN2 = l_bool_rd_LI1_FR_SRLR_OPEN2();
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_OPEN1 = l_bool_rd_LI1_FR_SRLR_OPEN1();
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_OVC2 = l_bool_rd_LI1_FR_SRLR_OVC2();
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_OVC1 = l_bool_rd_LI1_FR_SRLR_OVC1();
}

void Read_LIN_Fr_SRUD_Status(t_lin_RH_FR_STATUS *lin_Status)
{
	lin_Status->fr_srud_status.LDATA.FR_SRUD_EIDef = l_bool_rd_LI1_FR_SRUD_EIDef();
	lin_Status->fr_srud_status.LDATA.FR_SRUD_TSD = l_bool_rd_LI1_FR_SRUD_TSD();
	lin_Status->fr_srud_status.LDATA.FR_SRUD_STATE = l_bool_rd_LI1_FR_SRUD_STATE();
	lin_Status->fr_srud_status.LDATA.FR_SRUD_Sensor = l_bool_rd_LI1_FR_SRUD_Sensor();
	lin_Status->fr_srud_status.LDATA.FR_SRUD_OV = l_bool_rd_LI1_FR_SRUD_OV();
	lin_Status->fr_srud_status.LDATA.FR_SRUD_UV = l_bool_rd_LI1_FR_SRUD_UV();
	lin_Status->fr_srud_status.LDATA.FR_SRUD_OPEN2 = l_bool_rd_LI1_FR_SRUD_OPEN2();
	lin_Status->fr_srud_status.LDATA.FR_SRUD_OPEN1 = l_bool_rd_LI1_FR_SRUD_OPEN1();
	lin_Status->fr_srud_status.LDATA.FR_SRUD_OVC2 = l_bool_rd_LI1_FR_SRUD_OVC2();
	lin_Status->fr_srud_status.LDATA.FR_SRUD_OVC1 = l_bool_rd_LI1_FR_SRUD_OVC1();
}

Status Return_Evnt_Fr_SLLR_Status(t_lin_LH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag)
{
	uint8_t res;
	uint8_t values[] = {
		
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_EIDef, lin_Status->fr_sllr_status.LDATA.FR_SLLR_TSD,
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_STATE, lin_Status->fr_sllr_status.LDATA.FR_SLLR_Sensor, 
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_OV, lin_Status->fr_sllr_status.LDATA.FR_SLLR_UV, 
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN2, lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN1, 
	lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC2, lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC1,
	fault_flag->FR_DR_SD_FLT.fltBit.FR_SLLR_ExtFlt_Flag, fault_flag->FR_DR_SD_FLT.fltBit.FR_SLLR_IntFlt_Flag, fault_flag->FR_DR_SD_FLT.fltBit.FR_SLLR_ComFlt_Flag };
	
	uint8_t size = sizeof(values) / sizeof(values[0]);
	uint8_t size_IntFlt = (sizeof(values)-3) / sizeof(values[0]);
	
	Read_LIN_Fr_SLLR_Status(&LIN_LH_FR_STATUS);

	// fr_sllr_internal Error DTC Check	
	if(Contains_ONE_Fr_sllr(values, size_IntFlt)) { fault_flag->FR_DR_SD_FLT.fltBit.FR_SLLR_IntFlt_Flag = true; }
	else { fault_flag->FR_DR_SD_FLT.fltBit.FR_SLLR_IntFlt_Flag = false; }

	if(Contains_ONE_Fr_sllr(values, size)) { res = EVNT_STATUS_ERROR; }
	else { res = EVNT_STATUS_OK; }

	return res;
}


Status Return_Evnt_Fr_SLUD_Status(t_lin_LH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag)
{
	uint8_t res;
	uint8_t values[] = {
		
	lin_Status->fr_slud_status.LDATA.FR_SLUD_EIDef, lin_Status->fr_slud_status.LDATA.FR_SLUD_TSD,
	lin_Status->fr_slud_status.LDATA.FR_SLUD_STATE, lin_Status->fr_slud_status.LDATA.FR_SLUD_Sensor, 
	lin_Status->fr_slud_status.LDATA.FR_SLUD_OV, lin_Status->fr_slud_status.LDATA.FR_SLUD_UV, 
	lin_Status->fr_slud_status.LDATA.FR_SLUD_OPEN2, lin_Status->fr_slud_status.LDATA.FR_SLUD_OPEN1, 
	lin_Status->fr_slud_status.LDATA.FR_SLUD_OVC2, lin_Status->fr_slud_status.LDATA.FR_SLUD_OVC1,
	fault_flag->FR_DR_SD_FLT.fltBit.FR_SLUD_ExtFlt_Flag, fault_flag->FR_DR_SD_FLT.fltBit.FR_SLUD_IntFlt_Flag, fault_flag->FR_DR_SD_FLT.fltBit.FR_SLUD_ComFlt_Flag };

	uint8_t size = sizeof(values) / sizeof(values[0]);
	uint8_t size_IntFlt = (sizeof(values)-3) / sizeof(values[0]);
	
	Read_LIN_Fr_SLUD_Status(&LIN_LH_FR_STATUS);
	
	// fr_slud_internal Error DTC Check
	if(Contains_ONE_Fr_slud(values, size_IntFlt)) { fault_flag->FR_DR_SD_FLT.fltBit.FR_SLUD_IntFlt_Flag = true; }
	else { fault_flag->FR_DR_SD_FLT.fltBit.FR_SLUD_IntFlt_Flag = false; }

	if(Contains_ONE_Fr_slud(values, size)) { res = EVNT_STATUS_ERROR; }
	else { res = EVNT_STATUS_OK; }

	return res;
}


Status Return_Evnt_Fr_CLLR_Status(t_lin_LH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag)
{
	uint8_t res;
	uint8_t values[] = {
		
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_EIDef, lin_Status->fr_cllr_status.LDATA.FR_CLLR_TSD,
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_STATE, lin_Status->fr_cllr_status.LDATA.FR_CLLR_Sensor, 
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_OV, lin_Status->fr_cllr_status.LDATA.FR_CLLR_UV, 
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_OPEN2, lin_Status->fr_cllr_status.LDATA.FR_CLLR_OPEN1, 
	lin_Status->fr_cllr_status.LDATA.FR_CLLR_OVC2, lin_Status->fr_cllr_status.LDATA.FR_CLLR_OVC1,
	fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLLR_ExtFlt_Flag, fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLLR_IntFlt_Flag, fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLLR_ComFlt_Flag };
	
	uint8_t size = sizeof(values) / sizeof(values[0]);
	uint8_t size_IntFlt = (sizeof(values)-3) / sizeof(values[0]);
	
	Read_LIN_Fr_CLLR_Status(&LIN_LH_FR_STATUS);

	// fr_cllr_internal Error DTC Check	
	if(Contains_ONE_Fr_cllr(values, size_IntFlt)) { fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLLR_IntFlt_Flag = true; }
	else { fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLLR_IntFlt_Flag = false; }

	if(Contains_ONE_Fr_cllr(values, size)) { res = EVNT_STATUS_ERROR; }
	else { res = EVNT_STATUS_OK; }

	return res;
}


Status Return_Evnt_Fr_CLUD_Status(t_lin_LH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag)
{
	uint8_t res;
	uint8_t values[] = {
		
	lin_Status->fr_clud_status.LDATA.FR_CLUD_EIDef, lin_Status->fr_clud_status.LDATA.FR_CLUD_TSD,
	lin_Status->fr_clud_status.LDATA.FR_CLUD_STATE, lin_Status->fr_clud_status.LDATA.FR_CLUD_Sensor, 
	lin_Status->fr_clud_status.LDATA.FR_CLUD_OV, lin_Status->fr_clud_status.LDATA.FR_CLUD_UV, 
	lin_Status->fr_clud_status.LDATA.FR_CLUD_OPEN2, lin_Status->fr_clud_status.LDATA.FR_CLUD_OPEN1, 
	lin_Status->fr_clud_status.LDATA.FR_CLUD_OVC2, lin_Status->fr_clud_status.LDATA.FR_CLUD_OVC1,
	fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLUD_ExtFlt_Flag, fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLUD_IntFlt_Flag, fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLUD_ComFlt_Flag };

	uint8_t size = sizeof(values) / sizeof(values[0]);
	uint8_t size_IntFlt = (sizeof(values)-3) / sizeof(values[0]);
	
	Read_LIN_Fr_CLUD_Status(&LIN_LH_FR_STATUS);
	
	// fr_clud_internal Error DTC Check
	if(Contains_ONE_Fr_clud(values, size_IntFlt)) { fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLUD_IntFlt_Flag = true; }
	else { fault_flag->FR_DR_CTR_FLT.fltBit.FR_CLUD_IntFlt_Flag = false; }

	if(Contains_ONE_Fr_clud(values, size)) { res = EVNT_STATUS_ERROR; }
	else { res = EVNT_STATUS_OK; }

	return res;
}



Status Return_Evnt_Fr_CRLR_Status(t_lin_RH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag)
{
	uint8_t res;
	uint8_t values[] = {
		
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_EIDef, lin_Status->fr_crlr_status.LDATA.FR_CRLR_TSD,
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_STATE, lin_Status->fr_crlr_status.LDATA.FR_CRLR_Sensor, 
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_OV, lin_Status->fr_crlr_status.LDATA.FR_CRLR_UV, 
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_OPEN2, lin_Status->fr_crlr_status.LDATA.FR_CRLR_OPEN1, 
	lin_Status->fr_crlr_status.LDATA.FR_CRLR_OVC2, lin_Status->fr_crlr_status.LDATA.FR_CRLR_OVC1,
	fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRLR_ExtFlt_Flag, fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRLR_IntFlt_Flag, fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRLR_ComFlt_Flag };
	
	uint8_t size = sizeof(values) / sizeof(values[0]);
	uint8_t size_IntFlt = (sizeof(values)-3) / sizeof(values[0]);
	
	Read_LIN_Fr_CRLR_Status(&LIN_RH_FR_STATUS);

	// fr_crlr_internal Error DTC Check	
	if(Contains_ONE_Fr_crlr(values, size_IntFlt)) { fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRLR_IntFlt_Flag = true; }
	else { fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRLR_IntFlt_Flag = false; }

	if(Contains_ONE_Fr_crlr(values, size)) { res = EVNT_STATUS_ERROR; }
	else { res = EVNT_STATUS_OK; }

	return res;
}


Status Return_Evnt_Fr_CRUD_Status(t_lin_RH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag)
{
	uint8_t res;
	uint8_t values[] = {
		
	lin_Status->fr_crud_status.LDATA.FR_CRUD_EIDef, lin_Status->fr_crud_status.LDATA.FR_CRUD_TSD,
	lin_Status->fr_crud_status.LDATA.FR_CRUD_STATE, lin_Status->fr_crud_status.LDATA.FR_CRUD_Sensor, 
	lin_Status->fr_crud_status.LDATA.FR_CRUD_OV, lin_Status->fr_crud_status.LDATA.FR_CRUD_UV, 
	lin_Status->fr_crud_status.LDATA.FR_CRUD_OPEN2, lin_Status->fr_crud_status.LDATA.FR_CRUD_OPEN1, 
	lin_Status->fr_crud_status.LDATA.FR_CRUD_OVC2, lin_Status->fr_crud_status.LDATA.FR_CRUD_OVC1,
	fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRUD_ExtFlt_Flag, fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRUD_IntFlt_Flag, fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRUD_ComFlt_Flag };
	
	uint8_t size = sizeof(values) / sizeof(values[0]);
	uint8_t size_IntFlt = (sizeof(values)-3) / sizeof(values[0]);
	
	Read_LIN_Fr_CRUD_Status(&LIN_RH_FR_STATUS);

	// fr_crud_internal Error DTC Check	
	if(Contains_ONE_Fr_crud(values, size_IntFlt)) { fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRUD_IntFlt_Flag = true; }
	else { fault_flag->FR_PS_CTR_FLT.fltBit.FR_CRUD_IntFlt_Flag = false; }

	if(Contains_ONE_Fr_crud(values, size)) { res = EVNT_STATUS_ERROR; }
	else { res = EVNT_STATUS_OK; }

	return res;
}



Status Return_Evnt_Fr_SRLR_Status(t_lin_RH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag)
{
	uint8_t res;
	uint8_t values[] = {
		
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_EIDef, lin_Status->fr_srlr_status.LDATA.FR_SRLR_TSD,
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_STATE, lin_Status->fr_srlr_status.LDATA.FR_SRLR_Sensor, 
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_OV, lin_Status->fr_srlr_status.LDATA.FR_SRLR_UV, 
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_OPEN2, lin_Status->fr_srlr_status.LDATA.FR_SRLR_OPEN1, 
	lin_Status->fr_srlr_status.LDATA.FR_SRLR_OVC2, lin_Status->fr_srlr_status.LDATA.FR_SRLR_OVC1,
	fault_flag->FR_PS_SD_FLT.fltBit.FR_SRLR_ExtFlt_Flag, fault_flag->FR_PS_SD_FLT.fltBit.FR_SRLR_IntFlt_Flag, fault_flag->FR_PS_SD_FLT.fltBit.FR_SRLR_ComFlt_Flag };
	
	uint8_t size = sizeof(values) / sizeof(values[0]);
	uint8_t size_IntFlt = (sizeof(values)-3) / sizeof(values[0]);
	
	Read_LIN_Fr_SRLR_Status(&LIN_RH_FR_STATUS);

	// fr_srlr_internal Error DTC Check	
	if(Contains_ONE_Fr_srlr(values, size_IntFlt)) { fault_flag->FR_PS_SD_FLT.fltBit.FR_SRLR_IntFlt_Flag = true; }
	else { fault_flag->FR_PS_SD_FLT.fltBit.FR_SRLR_IntFlt_Flag = false; }

	if(Contains_ONE_Fr_srlr(values, size)) { res = EVNT_STATUS_ERROR; }
	else { res = EVNT_STATUS_OK; }

	return res;
}


Status Return_Evnt_Fr_SRUD_Status(t_lin_RH_FR_STATUS *lin_Status, FR_DTC_Flt_flag_t *fault_flag)
{
	uint8_t res;
	uint8_t values[] = {
		
	lin_Status->fr_srud_status.LDATA.FR_SRUD_EIDef, lin_Status->fr_srud_status.LDATA.FR_SRUD_TSD,
	lin_Status->fr_srud_status.LDATA.FR_SRUD_STATE, lin_Status->fr_srud_status.LDATA.FR_SRUD_Sensor, 
	lin_Status->fr_srud_status.LDATA.FR_SRUD_OV, lin_Status->fr_srud_status.LDATA.FR_SRUD_UV, 
	lin_Status->fr_srud_status.LDATA.FR_SRUD_OPEN2, lin_Status->fr_srud_status.LDATA.FR_SRUD_OPEN1, 
	lin_Status->fr_srud_status.LDATA.FR_SRUD_OVC2, lin_Status->fr_srud_status.LDATA.FR_SRUD_OVC1,
	fault_flag->FR_PS_SD_FLT.fltBit.FR_SRUD_ExtFlt_Flag, fault_flag->FR_PS_SD_FLT.fltBit.FR_SRUD_IntFlt_Flag, fault_flag->FR_PS_SD_FLT.fltBit.FR_SRUD_ComFlt_Flag };
	
	uint8_t size = sizeof(values) / sizeof(values[0]);
	uint8_t size_IntFlt = (sizeof(values)-3) / sizeof(values[0]);
	
	Read_LIN_Fr_SRUD_Status(&LIN_RH_FR_STATUS);

	// fr_srud_internal Error DTC Check	
	if(Contains_ONE_Fr_srud(values, size_IntFlt)) { fault_flag->FR_PS_SD_FLT.fltBit.FR_SRUD_IntFlt_Flag = true; }
	else { fault_flag->FR_PS_SD_FLT.fltBit.FR_SRUD_IntFlt_Flag = false; }

	if(Contains_ONE_Fr_srud(values, size)) { res = EVNT_STATUS_ERROR; }
	else { res = EVNT_STATUS_OK; }

	return res;
}




void Evnt_StatusError_Check(void)
{	
	Status status_Fr_Sllr = Return_Evnt_Fr_SLLR_Status(&LIN_LH_FR_STATUS, &FR_DTC_Flt_flag);
	Status status_Fr_Slud = Return_Evnt_Fr_SLUD_Status(&LIN_LH_FR_STATUS, &FR_DTC_Flt_flag);
	Status status_Fr_Cllr = Return_Evnt_Fr_CLLR_Status(&LIN_LH_FR_STATUS, &FR_DTC_Flt_flag);
	Status status_Fr_Clud = Return_Evnt_Fr_CLUD_Status(&LIN_LH_FR_STATUS, &FR_DTC_Flt_flag);

	Status status_Fr_Crlr = Return_Evnt_Fr_CRLR_Status(&LIN_RH_FR_STATUS, &FR_DTC_Flt_flag);
	Status status_Fr_Crud = Return_Evnt_Fr_CRUD_Status(&LIN_RH_FR_STATUS, &FR_DTC_Flt_flag);
	Status status_Fr_Srlr = Return_Evnt_Fr_SRLR_Status(&LIN_RH_FR_STATUS, &FR_DTC_Flt_flag);
	Status status_Fr_Srud = Return_Evnt_Fr_SRUD_Status(&LIN_RH_FR_STATUS, &FR_DTC_Flt_flag);
	
	handleStatus_FR_SLLR(status_Fr_Sllr);
	handleStatus_FR_SLUD(status_Fr_Slud);
	handleStatus_FR_CLLR(status_Fr_Cllr);
	handleStatus_FR_CLUD(status_Fr_Clud);

	handleStatus_FR_CRLR(status_Fr_Crlr);
	handleStatus_FR_CRUD(status_Fr_Crud);
	handleStatus_FR_SRLR(status_Fr_Srlr);
	handleStatus_FR_SRUD(status_Fr_Srud);

}





