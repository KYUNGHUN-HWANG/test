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

bool contains_one(const uint8_t *array, uint8_t size)
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


Status Return_Evnt_Status(t_lin_LH_FR_STATUS *lin_Status)
{
	uint8_t res;
	uint8_t values[] = {
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_VddReset, lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss,
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_EIDef, lin_Status->fr_sllr_status.LDATA.FR_SLLR_TSD,
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_TW, lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo,
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_STATE, lin_Status->fr_sllr_status.LDATA.FR_SLLR_Sensor, 
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_OV, lin_Status->fr_sllr_status.LDATA.FR_SLLR_UV, 
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN2, lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN1, 
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC2, lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC1 
	};
	uint8_t size = sizeof(values) / sizeof(values[0]);
	
	if(l_flg_tst_LI0_FR_SLLR_VddReset_flag())
	{
			l_flg_clr_LI0_FR_SLLR_VddReset_flag();

			lin_Status->fr_sllr_status.LDATA.FR_SLLR_VddReset = l_bool_rd_LI0_FR_SLLR_VddReset();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss = l_bool_rd_LI0_FR_SLLR_Steploss();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_EIDef = l_bool_rd_LI0_FR_SLLR_EIDef();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_TSD = l_bool_rd_LI0_FR_SLLR_TSD();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_TW = l_bool_rd_LI0_FR_SLLR_TW();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo = l_u8_rd_LI0_FR_SLLR_Tinfo();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_STATE = l_bool_rd_LI0_FR_SLLR_STATE();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_Sensor = l_bool_rd_LI0_FR_SLLR_Sensor();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_OV = l_bool_rd_LI0_FR_SLLR_OV();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_UV = l_bool_rd_LI0_FR_SLLR_UV();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN2 = l_bool_rd_LI0_FR_SLLR_OPEN2();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN1 = l_bool_rd_LI0_FR_SLLR_OPEN1();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC2 = l_bool_rd_LI0_FR_SLLR_OVC2();
			lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC1 = l_bool_rd_LI0_FR_SLLR_OVC1();


//			lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo = EVNT_STATUS_ERROR_2;

/*
			if(
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_VddReset == EVNT_STATUS_ERROR || lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss == EVNT_STATUS_ERROR || \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_EIDef == EVNT_STATUS_ERROR || lin_Status->fr_sllr_status.LDATA.FR_SLLR_TSD == EVNT_STATUS_ERROR || \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_TW == EVNT_STATUS_ERROR || \
				((lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo == EVNT_STATUS_ERROR)||(lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo == EVNT_STATUS_ERROR_2)) || \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_STATE == EVNT_STATUS_ERROR || lin_Status->fr_sllr_status.LDATA.FR_SLLR_Sensor == EVNT_STATUS_ERROR || \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_OV == EVNT_STATUS_ERROR || lin_Status->fr_sllr_status.LDATA.FR_SLLR_UV == EVNT_STATUS_ERROR || \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN2 == EVNT_STATUS_ERROR || lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN1 == EVNT_STATUS_ERROR || \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC2 == EVNT_STATUS_ERROR || lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC1 == EVNT_STATUS_ERROR)
			{
				res = EVNT_STATUS_ERROR;
			}

			else if (
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_VddReset == EVNT_STATUS_ERROR && lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss == EVNT_STATUS_ERROR && \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_EIDef == EVNT_STATUS_ERROR && lin_Status->fr_sllr_status.LDATA.FR_SLLR_TSD == EVNT_STATUS_ERROR && \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_TW == EVNT_STATUS_ERROR && \
				((lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo == EVNT_STATUS_ERROR)&&(lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo == EVNT_STATUS_ERROR_2)) && \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_STATE == EVNT_STATUS_ERROR && lin_Status->fr_sllr_status.LDATA.FR_SLLR_Sensor == EVNT_STATUS_ERROR && \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_OV == EVNT_STATUS_ERROR && lin_Status->fr_sllr_status.LDATA.FR_SLLR_UV == EVNT_STATUS_ERROR && \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN2 == EVNT_STATUS_ERROR && lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN1 == EVNT_STATUS_ERROR && \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC2 == EVNT_STATUS_ERROR && lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC1 == EVNT_STATUS_ERROR)
			{
				res = EVNT_STATUS_OK;
			}
*/
/*
			if((lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo == EVNT_STATUS_ERROR) || (lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo == EVNT_STATUS_ERROR_2))
			{
				res = EVNT_STATUS_ERROR;
			}
*/

			
			if(contains_one(values, size)) 
			{
				res = EVNT_STATUS_ERROR;
			}
			else 
			{
				res = EVNT_STATUS_OK;
			}

/*			
			if(
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_VddReset | lin_Status->fr_sllr_status.LDATA.FR_SLLR_Steploss | \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_EIDef | lin_Status->fr_sllr_status.LDATA.FR_SLLR_TSD | \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_TW | lin_Status->fr_sllr_status.LDATA.FR_SLLR_STATE | \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_Sensor | lin_Status->fr_sllr_status.LDATA.FR_SLLR_OV | \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_UV | lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN2 | \
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_OPEN1 | lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC2 | lin_Status->fr_sllr_status.LDATA.FR_SLLR_OVC1 |\
				lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo)
			{
				res = EVNT_STATUS_ERROR;
			}
			else
			{
				res = EVNT_STATUS_OK;
			}

			if(lin_Status->fr_sllr_status.LDATA.FR_SLLR_Tinfo == EVNT_STATUS_ERROR_2) { res = EVNT_STATUS_ERROR; }
*/		
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



