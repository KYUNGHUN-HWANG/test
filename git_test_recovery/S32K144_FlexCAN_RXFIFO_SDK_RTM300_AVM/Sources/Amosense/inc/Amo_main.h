#ifndef Amo_main_H_
#define Amo_main_H_

/*
#include "bsp_master_jpc.h"
#include "timer.h"
#include "misc_amo.h"
*/

#include "Amo_UART.h"
#include "Amo_BSP_master_nifco.h"

#include "Amo_GPIO.h"
#include "Amo_LPTMR.h"
#include "Amo_LIN.h"
#include "Amo_Calculate.h"
#include "Amo_Status.h"
#include "Amo_Cycle.h"
#include "Amo_Recovery.h"

/*
#include "gpio_amo.h"
#include "system_amo.h"
#include "interrupt_amo.h"
#include "systick_amo.h"
#include "led_amo.h"
#include "pixel_calc_amo.h"
*/


//#define EOL_CRLF

//#define LIN1

//#define FOR_DEMO

//#define CHECK_IGNSTS_STAT
//#define DEBUG_ON
#define DEBUG_MODE 
#define FLEX_UART_ON
//#define SW_VERSION				1.01
//#define HW_VERSION				0x000A
#define HEADER_SIGNATURE_1	0x55AA55AAU
#define HEADER_SIGNATURE_2	0x55AA55BBU
#define HEADER_SIGNATURE_NULL	0xFFFFFFFFU

#define APP_KEY						0xA5A5A5A5U
#define FW_VERSION				0x00010001U
#define FW_START_ADDR			0x00005000U
#define FLASH_SECTOR_SIZE	0x1000U //0x1000
#define FLASH_WRITE_SIZE	0x0400U //0x1000

#define FLASH_HEADER_ADDRESS_1	0x00006000U	//0x00004000
#define FLASH_HEADER_ADDRESS_2	0x00044000U	//0x00042000	//0x00038810

#define FLASH_FW1_ADDRESS_1	0x00007000U	//0x00005000
#define FLASH_FW2_ADDRESS_2	0x00045000U	//0x00043000	//0x00039810

#define FW_CHKSUM 0x00004050U
#define FW_CHKSUM_BKUP 0x00004060U

#define EVT_QUEUE 256u

#undef FALSE
#define FALSE	            0

#undef TRUE
#define TRUE	            1

/* Definition of the TX and RX message buffers depending on the bus role */
#define TX_MAILBOX  ((uint8_t)24U)
#define CAN_RX_NUM 24
#define NVMEM_VERSION_INFO 0x03

#define AMO_NVM_SETTING
//#define AMO_GN7_PE_SETTING_NONE

#ifdef AMO_GN7_PE_SETTING_NONE
#define AMO_SETTING_BUF 58
#else
#define AMO_SETTING_BUF 38
#endif

//#define AMO_DTC_SETTING

#define MAX_DTC_COUNT 50
#define BATT_CAL_DATA 3103 //3093	//3340	//reference voltage 12.5V
//#define BATT_CAL_NONE
#define CAN_DBC_VER 0x240503u
#define LIN_LDF_VER 0x250116u 

/* Definition of power modes indexes, as configured in Power Manager Component
 *  Refer to the Reference Manual for details about the power modes.
 */
#define HSRUN (0u) /* High speed run      */
#define RUN   (1u) /* Run                 */
#define VLPR  (2u) /* Very low power run  */
#define STOP1 (3u) /* Stop option 1       */
#define STOP2 (4u) /* Stop option 2       */
#define VLPS  (5u) /* Very low power stop */

#endif /* Amo_main_H_ */

