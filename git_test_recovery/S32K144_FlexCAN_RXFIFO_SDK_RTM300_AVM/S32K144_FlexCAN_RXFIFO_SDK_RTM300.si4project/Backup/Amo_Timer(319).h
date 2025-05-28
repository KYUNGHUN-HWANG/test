
#if !defined(AMO_TMR_H)
#define AMO_TMR_H

#include <stddef.h>
#include <stdbool.h>

#include "Amo_CAN_Parsing.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**
* \addtogroup group_mw_swtmr_macros
* \{
*/

#define CY_SWTMR_MAX_TIMERS       100	//5  // must be <= 256


/** Define base time tick interval in microseconds. Default value is 1000 �s = 1 ms.
 ** This value will determine the counting resolution of the tick count returned
 ** by Cy_SwTmr_GetTickCountUs(). It will also influence the interrupt load so handle
 ** with care!
 ** Min/Max values: 20 <= CY_SWTMR_TICK_INTERVAL_US <= 1000 */
#define CY_SWTMR_TICK_INTERVAL_US          1000

/* LPIT channel used */
#define LPIT_CHANNEL        0UL
#define LPIT_Channel_IRQn   LPIT0_Ch0_IRQn

/** \} group_mw_swtmr_macros */

/**
* \addtogroup group_mw_swtmr_data_structures
* \{
*/

/** Function pointer type to void/void function for SW Timer Callbacks*/
typedef void (*amo_timer_callback_t)(void);

/** Function pointer type to void/void function that will handle the watchdog during Cy_SwTmr_Wait calls*/
typedef void (*amo_timer_wdg_handle_callout_t)(void);

/** \} group_mw_swtmr_data_structures */

/**
* \addtogroup group_mw_swtmr_enums
* \{
*/

typedef enum
{
    AMO_TIMER_SUCCESS   = 0x00u, /**< Returned successful */
    AMO_TIMER_BAD_PARAM = 0x01u, /**< Bad parameter was passed */
} amo_timer_en_result_t;

/** \} group_mw_swtmr_enums */

/***************************************
*       Function Prototypes
***************************************/

/**
* \addtogroup group_mw_swtmr_functions
* \{
*/

/*******************************************************************************
* Function Name: Cy_SwTmr_Init
****************************************************************************//**
*
* \brief  Initializes the SW Timer
*
* \param  pstcSwTmrCfg : The pointer to the init configuration struct
*
* \return None
*
*******************************************************************************/
void Amo_Timer_Init(void);

/*******************************************************************************
* Function Name: Cy_SwTmr_Disable
****************************************************************************//**
*
* \brief  Disables the SW Timer
*
* \param  None
*
* \return None
*
*******************************************************************************/
void Amo_timer_Disable(void);

amo_timer_en_result_t Amo_timer_Start(Amotimer u8TimerID, int32_t i32TimeMs, bool bReload, amo_timer_callback_t pfnCallback);
amo_timer_en_result_t Amo_timer_StartHighPrio(Amotimer u8TimerID, int32_t i32TimeMs, bool bReload, amo_timer_callback_t pfnCallback);
amo_timer_en_result_t Amo_timer_Stop(Amotimer u8TimerID);
int32_t Amo_timer_Remaining(Amotimer u8TimerID);
amo_timer_en_result_t Amo_timer_Wait(Amotimer u8TimerID, int32_t i32TimeMs, amo_timer_wdg_handle_callout_t pfnWatchdogCallout);
void Amo_timer_Main(void);
bool Amo_timer_IsElapsed(Amotimer u8TimerID, bool bClearElapsedFlag);
uint64_t Amo_timer_GetTickCountUs(void);
bool Amo_timer_IsTimeSpanElapsed(uint64_t u64TickCountUs, uint32_t u32TimeSpanUs);
amo_timer_en_result_t Amo_timer_Status(Amotimer u8TimerID);

/** \} group_mw_swtmr_functions */

#if defined(__cplusplus)
}
#endif

#endif /* CY_SWTMR_H */

/** \} group_swtmr */

/* [] END OF FILE */

