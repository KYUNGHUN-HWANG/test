/* ###################################################################
**     Filename    : main.c
**     Processor   : S32K1xx
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.00
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */

/* Including necessary module. Cpu.h contains other modules needed for compiling.*/
#include "Cpu.h"
#include "clockMan1.h"
#include "dmaController1.h"
#include "pin_mux.h"
#include "watchdog1.h"
#include "pwrMan1.h"
#include "lpTmr1.h"
#include "lin_cfg.h"
#include "lin1.h"
#include "lin_common_api.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "stdlib.h"
#include "Amo_CAN_Parsing.h"
#include "Amo_Mode_Setting.h"
#include "ewl_misra_types.h"
#include "Amo_STATE_Event.h"
#include "Amo_Timer.h"
#include "Amo_main.h"
#include "Amo_Adc.h"
#include "Amo_GPIO.h"
#include "Amo_Flash.h"
#include "Amo_Nvm.h"
#include "Amo_Sleep.h"
#include "Amo_Cycle.h"
#include "Amo_System_setting.h"

/******************************************************************************
 * static variable Definitions
 ******************************************************************************/
static volatile int exit_code = 0;
/* User includes (#include below this line is not maintained by Processor Expert) */
/******************************************************************************
 * Definitions
 ******************************************************************************/

/* This example is setup to work by default with EVB. To use it with other boards
   please comment the following line
*/

/* LPIT channel used */
#define LPIT_CHANNEL        0UL
#define LPIT_Channel_IRQn   LPIT0_Ch0_IRQn

/******************************************************************************
 * Global Can value parameter 
 ******************************************************************************/

/******************************************************************************
 * Variable prototypes
 ******************************************************************************/
static uint16_t test_counter = 0;

/******************************************************************************
 * Function prototypes
 ******************************************************************************/
 #ifdef KEY_ENABLE
void buttonISR(void);
 #endif
void BoardInit(void);
void GPIOInit(void);
void WDOG_ISR(void);
void LVD_LVW_ISR(void);
void LVD_LVW_ISR(void);
void LVD_SET(void);
void Manual_dr_evt_signal(void);
void Manual_ps_evt_signal(void);
void Amo_Device_init(void);
void Amo_Com_init(void);
void Amo_Sys_init(void);

/******************************************************************************
 * Functions
 ******************************************************************************/
#ifdef FLEX_UART_ON
int __write_console(__file_handle handle, char *buffer, uint32_t *count)
{
(void)(handle);
    uint32_t bytes=*count;
    FLEXIO_UART_DRV_SendDataBlocking(&uartStateTX, (uint8_t *)buffer, bytes, TIMEOUT);
    return 0;
}
#endif
/*
* @brief WDOG_ISR function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void WDOG_ISR(void)
{
}

/*
* @brief Amo_Sys_init function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Amo_Sys_init(void)
{
	BoardInit();
	Amo_Gpio_Init();
#ifdef FLEX_UART_ON
	Flexio_Uart_Init();
#endif
#ifdef AMO_NVM_SETTING
	Amo_Flash_Init();
#endif	
	Lptmr_Init();
	Amo_Timer_Init();
}


/*
* @brief Amo_Device_init function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Amo_Device_init(void)
{
	Lin_Init();
	EDMA_DRV_Init(&dmaController1_State, &dmaController1_InitConfig0, edmaChnStateArray, edmaChnConfigArray, EDMA_CONFIGURED_CHANNELS_COUNT);
	INT_SYS_InstallHandler(WDOG_EWM_IRQn, WDOG_ISR, (isr_t *)NULL);
	INT_SYS_EnableIRQ(WDOG_EWM_IRQn);	
	Evt_queue_init();
	Start_Evnt_StatusCheck();
#ifdef AMO_NVM_SETTING
	nvmem_init();
#endif	
}

/*
* @brief Amo_Com_init function
* @param  Event: None
* @return None
* @note This function polls until conversion is complete.
*/
void Amo_Com_init(void)
{
	Amo_Can_Init();
	Amo_Adc_Init(); 
	WDOG_DRV_Init(INST_WATCHDOG1, &watchdog1_Config0);
	Adc_Value();
	LVD_SET();
}


int main(void)
{
	/*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
#ifdef PEX_RTOS_INIT
	PEX_RTOS_INIT();                   /* Initialization of the selected RTOS. Macro is defined by the RTOS component. */
#endif
	/*** End of Processor Expert internal initialization.                    ***/
	Amo_Sys_init();
	Amo_Device_init();
	/* Do the initializations required for this application */
	#ifdef DEBUG_MODE
	printf("Air Vent main start\r\n");
	#endif
	Amo_Com_init();
	while(1)
	{
		//Watch Dog Timer~~~~
		WDOG_DRV_Trigger(INST_WATCHDOG1);
		test_counter++;	
		if(test_counter == 1000)
		{
			test_counter=0;
		}
		
		if(rxFIFOdone == 1)		// if message was received into RXFIFO
		{
			/*can rx done init */
			rxFIFOdone = 0;
			FLEXCAN_DRV_RxFifo(INST_CANCOM1,&recvBuff2);
			test_counter = 0;
		}
		/* state start */
		Handle_Event_Mode();
#ifdef LIN_BUSOFF_CHECK_DELAY_50MS
		lin_BusOff();
#endif
		recovery_mode_check(slaveId_flag);
	}

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
	#if 0
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;) {
    if(exit_code != 0) {
      break;
    }
  }
  return exit_code;
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
	#endif
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.1 [05.21]
**     for the NXP S32K series of microcontrollers.
**
** ###################################################################
*/

