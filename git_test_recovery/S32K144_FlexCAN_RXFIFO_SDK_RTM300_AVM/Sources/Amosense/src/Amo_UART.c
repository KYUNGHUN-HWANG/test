#define Amo_UART_C_


#include <stdio.h>
#include <string.h>

#include "Cpu.h"
//#include "uart_amo_airVent.h"
#include "Amo_main.h"


#define TIMER_INTERVAL_20ms	(20u)
#define TIMER_INTERVAL_25ms	(25u)

#define TIMER_INTERVAL_30ms	(3u) //uesed 10m Timer
#define TIMER_INTERVAL_10ms	(1u) //uesed 10m Timer


#define TIMER_INTERVAL_100ms	(100u)


#define UART_OVERSAMPLING (8u)
#define UART_RX_UNIT_BYTE (1u)

/* Receive buffer size */
#define BUFFER_SIZE (256u)

#ifdef FLEX_UART_ON
void flexio_uart_RX_Callback0(void *driverState, uart_event_t event, void *userData);




#define WELCOME_MSG ("AirVent_Master_Nifco!!\r\n")
#define RX_BUFFER_SIZE 12 //10


uint8_t rxBuffer[BUFFER_SIZE];   /* Circular buffer for Rx */
volatile uint8_t rxWrite = 0;       /* Write location in Rx buffer */
volatile uint8_t rxRead = 0;        /* Read location in Rx buffer */
uint8_t txBuffer;                   /* 1-byte buffer for Tx */
flexio_device_state_t flexIODeviceState;
flexio_uart_state_t   uartStateTX;
flexio_uart_state_t   uartStateRX;
Flexio_Uart_flag_t			FLEXIO_UART_FLAG;

/* Write your local variable definition here */
static status_t status = STATUS_SUCCESS;

/* Declare a buffer used to store the received data */
uint32_t bytesRemaining;



void flexio_uart_RX_Callback0(void *driverState, uart_event_t event, void *userData)
{
    /* Unused parameters */
    (void)driverState;
    (void)userData;

    /* Check the event type */
    if (event == UART_EVENT_RX_FULL)
    {
        /* The reception stops when newline is received or the buffer is full */
#if defined(EOL_CRLF)
        if ((rxBuffer[rxWrite] != '\n') && (rxWrite != (BUFFER_SIZE - 2U)))
#else
        if ((rxBuffer[rxWrite] != ENDCODE) && (rxWrite != (BUFFER_SIZE - 2U)))
#endif /* EOL_CRLF */
        {
            /* Update the buffer index and the rx buffer */

        	  rxWrite++;
//            LPUART_DRV_SetRxBuffer(INST_LPUART0, &g_uartBuffer[g_bufferIdx], 1U);
            FLEXIO_UART_DRV_SetRxBuffer(&uartStateRX, &(rxBuffer[rxWrite]), 1U);
        }
    }

}

void flexio_uart_TX_Callback0(void *driverState, uart_event_t event, void *userData)
{
    (void)userData;
    (void)driverState;

    if (event == UART_EVENT_TX_EMPTY)
    {
        /* If a new char is already available, send it immediately
           This helps reduce the delays caused by stopping/restarting the transmission */
        if (rxRead != rxWrite)
        {
            /* Copy received byte to tx buffer */
            txBuffer = rxBuffer[rxRead];
            /* Advance read pointer to the next location in Rx buffer */
            rxRead++;
            if (rxRead == BUFFER_SIZE)
            {
                rxRead = 0;
            }
            /* Send received byte */
            FLEXIO_UART_DRV_SetTxBuffer(&uartStateTX, &txBuffer, 1UL);
        }
    }
}


void Flexio_Uart_Init(void)
{
  /* Init the FLEXIO device */
  FLEXIO_DRV_InitDevice(INST_FLEXIO_UART_TX, &flexIODeviceState);

  /* Init the FlexIO UART driver
   *  - 115200 baudrate
   *  - TX only
   *  - 8 bit per frame
   *  - Interrupt based
   */
   FLEXIO_UART_DRV_Init(INST_FLEXIO_UART_TX, &flexio_uart_TX_Config0, &uartStateTX);

  /* Init the FlexIO UART driver
   *  - 115200 baudrate
   *  - RX only
   *  - 8 bit per frame
   *  - Interrupt based
   */
   FLEXIO_UART_DRV_Init(INST_FLEXIO_UART_RX, &flexio_uart_RX_Config0, &uartStateRX);

   //FLEXIO_UART_DRV_SendDataBlocking(&uartStateTX, (uint8_t * )WELCOME_MSG, strlen(WELCOME_MSG), TIMEOUT);

   FLEXIO_UART_DRV_ReceiveData(&uartStateRX, rxBuffer, 1U);
//   FLEXIO_UART_DRV_ReceiveData(&uartStateRX, g_uartBuffer, 1U);

}


void Flexio_Uart_GetReceive(void)
{
	if(FLEXIO_UART_FLAG.uartStartFlag == TRUE)
  {
		/* Wait for transfer to be completed */
		while(FLEXIO_UART_DRV_GetStatus(&uartStateRX, &bytesRemaining) == STATUS_BUSY)
		{
//			if(UART_FLAG.uartStartFlag == TRUE)
//			break;
		}

		status = FLEXIO_UART_DRV_GetStatus(&uartStateRX, &bytesRemaining);

		if (status != STATUS_SUCCESS)
		{
			/* If an error occurred, send the error message and exit the loop */
//			LPUART_DRV_SendDataBlocking(INST_LPUART0, (uint8_t *)errorMsg, strlen(errorMsg), TIMEOUT);

			FLEXIO_UART_FLAG.uartErrFlag.uartComm = TRUE;
//			break;
		}
 	}
}


void Delay(volatile int cycles)
{
    /* Delay function - do nothing for a number of cycles */
    while(cycles--);
}

#endif


/* [] END OF FILE */
