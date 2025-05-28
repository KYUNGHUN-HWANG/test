#define Amo_STATUS_C_

#include "Cpu.h"
#include "Amo_main.h"
#include "Amo_timer.h"


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

void Start_Evnt_StatusCheck(void)
{
	Amo_timer_Stop(timer_22);
	Amo_timer_Start(timer_22, 50, true, Evnt_StatusError_Check);
}


void handleOk(Lvnt_3_data *L3_DataBuf)
{
	L3_DataBuf->data.L_EVNT_FrDrEvntSideFlt = E_NO_ERROR;
}

void handleError(Lvnt_3_data *L3_DataBuf)
{
	L3_DataBuf->data.L_EVNT_FrDrEvntSideFlt = E_ERROR;
}

void handleBusy(Lvnt_3_data *L3_DataBuf)
{
}

void handleTimeout(Lvnt_3_data *L3_DataBuf)
{
}

void handleStatus(Status status)
//void handleStatus(uint8_t status)
{
    switch (status)
    {
        case EVNT_STATUS_OK:
        		handleOk(&Can_Tx_Evnt_3);
        		//printf("Status: OK\n");
            break;
        case EVNT_STATUS_ERROR:
        		handleError(&Can_Tx_Evnt_3);
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


Status Return_Evnt_Status(t_lin_LH_FR_STATUS *lin_Status)
{
	uint8_t res;
	
	if(l_flg_tst_LI0_FR_SLLR_VddReset_flag())
	{
			l_flg_clr_LI0_FR_SLLR_VddReset_flag();

			lin_Status->fr_sllr_status.LDATA.FR_SLLR_VddReset = l_bool_rd_LI0_FR_SLLR_VddReset();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss = l_bool_rd_LI0_FR_SLLR_Steploss();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_EIDef = l_bool_rd_LI0_FR_SLLR_EIDef();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_TSD = l_bool_rd_LI0_FR_SLLR_TSD();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_TW = l_bool_rd_LI0_FR_SLLR_TW();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo = l_u8_rd_LI0_FR_SLLR_Tinfo();

//			lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo = EVNT_STATUS_ERROR_2;

			if(lin_Status->fr_sllr_status.LDATA.FR_SLLR_VddReset == EVNT_STATUS_ERROR || lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss == EVNT_STATUS_ERROR || \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_EIDef == EVNT_STATUS_ERROR || lin_Status->fr_sllr_status.LDATA.FR_SLLR_TSD == EVNT_STATUS_ERROR || \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_TW == EVNT_STATUS_ERROR || \
				((lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo == EVNT_STATUS_ERROR)||(lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo == EVNT_STATUS_ERROR_2))) 
			{
				res = EVNT_STATUS_ERROR;
			}

			else if ((lin_Status->fr_sllr_status.LDATA.FR_SLLR_VddReset == 0) && (lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss == 0) && \
					(lin_Status->fr_sllr_status.LDATA.FR_SLLR_EIDef == 0))
			{
				res = EVNT_STATUS_OK;
			}
	}
	return res;
}



void Evnt_StatusError_Check(void)
{	
//	PINS_DRV_SetPins(GPIO_PORTA, TEST_OUT1_MASK);

	Status status = Return_Evnt_Status(&LIN_LH_FR_STATUS);
	handleStatus(status);

//	PINS_DRV_ClearPins(GPIO_PORTA, TEST_OUT1_MASK);
}



