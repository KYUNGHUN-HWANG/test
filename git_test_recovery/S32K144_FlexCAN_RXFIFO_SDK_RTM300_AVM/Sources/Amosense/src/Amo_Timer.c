/***************************************************************************//**
* \file cy_sw_tmr.c
* \version 1.0
*
* \brief
* Provides an API implementation of the Software Timer (SwTmr) middleware driver
*
********************************************************************************
* \copyright
* Copyright 2016-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


/*****************************************************************************
* Include files
*****************************************************************************/
#include "Cpu.h"
#include "clockMan1.h"
#include "dmaController1.h"

#include "Amo_Timer.h"

#if defined(__cplusplus)
extern "C" {
#endif

/**
 *****************************************************************************
 ** \brief Holds configuration and status of a single SW timer
 ** 
 *****************************************************************************/
typedef struct
{
    volatile int32_t      i32Timer;
    int32_t               i32Reload;
    amo_timer_callback_t  pfnCallback;
    volatile bool         bElapsed;
    bool                  bHighPrio;
} stc_sw_tmr_cfg_status_t;


/*****************************************************************************
* Local variable definitions ('static')
*****************************************************************************/

static stc_sw_tmr_cfg_status_t m_astcTimers[CY_SWTMR_MAX_TIMERS];
static volatile uint8_t m_u8CallbackFunctionTrigger;
static volatile uint64_t m_u64TickCountUs;


/*****************************************************************************
* Local function prototypes ('static')                                                                            
*****************************************************************************/
static void StartTimer(Amotimer u8TimerID, int32_t i32TimeMs, bool bReload, amo_timer_callback_t pfnCallback);

static void Amo_timer_callback(void);

/*****************************************************************************
* Function implementation - global ('extern') and local ('static')
*****************************************************************************/


/**
 *****************************************************************************
 ** Initializes the HW timer used as SW Timer source (SysTick)
 **
 ** The global tick counter (returned by Cy_SwTmr_GetTickCountUs()) will be reset
 ** to 0.
 **
 ** \return none
 *****************************************************************************/
void Amo_Timer_Init( void )
{

	LPIT_DRV_Init(INST_LPIT1, &lpit1_InitConfig);
	/* Initialize LPIT channel 0 and configure it as a periodic counter
	* which is used to generate an interrupt every second.
	*/
	LPIT_DRV_InitChannel(INST_LPIT1, LPIT_CHANNEL, &lpit1_ChnConfig0);

	/* Start LPIT0 channel 0 counter */
	LPIT_DRV_StartTimerChannels(INST_LPIT1, (1U << LPIT_CHANNEL));

	/* Install LPIT_ISR as LPIT interrupt handler */
	INT_SYS_InstallHandler(LPIT_Channel_IRQn, &Amo_timer_callback, (isr_t *)NULL);

	// Reset tick counter
	m_u64TickCountUs = 0;
    
}

/**
 *****************************************************************************
 ** Disables the HW timer used as SW Timer source (SysTick)
 **
 ** \return none
 *****************************************************************************/
 void Amo_timer_Disable( void )
{
    LPIT_DRV_Deinit(INST_LPIT1);
}

/**
 *****************************************************************************
 ** Starts a SW timer, indexed by an ID.
 ** To use the callback function, it is mandatory to
 ** cyclically call the Cy_SwTmr_Main() function. The callback
 ** functions are not called directly out of the ISR!
 **
 ** \param u8TimerID   0 to (CY_SWTMR_MAX_TIMERS - 1)
 ** \param i32TimeMs   Duration in ms (must be greater than or equal 0)
 ** \param bReload     Reload the timer automatically
 ** \param pfnCallback Callback function (when timer is elapsed)
 **
 ** \retval AMO_TIMER_SUCCESS    Timer started
 ** \retval AMO_TIMER_BAD_PARAM  u8TimerID invalid or i32TimeMs invalid
 *****************************************************************************/
amo_timer_en_result_t Amo_timer_Start(Amotimer u8TimerID, int32_t i32TimeMs, bool bReload, amo_timer_callback_t pfnCallback)
{
		amo_timer_en_result_t enResult = AMO_TIMER_SUCCESS;
		
    if (u8TimerID >= TIMER_MAX)
    {
        enResult = AMO_TIMER_BAD_PARAM;
    }
    if (i32TimeMs < 0)
    {
        enResult = AMO_TIMER_BAD_PARAM;        
    }
    
    m_astcTimers[u8TimerID].bHighPrio = false;
    StartTimer(u8TimerID, i32TimeMs, bReload, pfnCallback);
    
    return enResult;
}

/**
 *****************************************************************************
 ** Starts a (high-prio) SW timer, indexed by an ID.
 ** The callback functions will be called directly out of the ISR!
 ** For this timer type it is NOT necessary to cyclically call Cy_SwTmr_Main()
 **
 ** \param u8TimerID   0 to (CY_SWTMR_MAX_TIMERS - 1)
 ** \param u32TimeMs   Duration in ms
 ** \param bReload     Reload the timer automatically
 ** \param pfnCallback Callback function (when timer is elapsed)
 **
 ** \retval AMO_TIMER_SUCCESS    Timer started
 ** \retval AMO_TIMER_BAD_PARAM  u8TimerID invalid or i32TimeMs invalid
 *****************************************************************************/
amo_timer_en_result_t Amo_timer_StartHighPrio(Amotimer u8TimerID, int32_t i32TimeMs, bool bReload, amo_timer_callback_t pfnCallback)
{
		amo_timer_en_result_t enResult = AMO_TIMER_SUCCESS;
		
    if (u8TimerID >= TIMER_MAX)
    {
        enResult = AMO_TIMER_BAD_PARAM;
    }
    if (i32TimeMs < 0)
    {
        enResult = AMO_TIMER_BAD_PARAM;        
    }    

    m_astcTimers[u8TimerID].bHighPrio = true;
    StartTimer(u8TimerID, i32TimeMs, bReload, pfnCallback);
		
	return enResult;    
}

/**
 *****************************************************************************
 ** Starts a SW timer, indexed by an ID.
 ** (common part for Cy_SwTmr_Start and Cy_SwTmr_StartHighPrio)
 **
 ** \param u8TimerID   0 to (CY_SWTMR_MAX_TIMERS - 1)
 ** \param i32TimeMs   Duration in ms (must be greater than or equal 0)
 ** \param bReload     Reload the timer automatically
 ** \param pfnCallback Callback function (when timer is elapsed)
 **
 *****************************************************************************/
static void StartTimer(Amotimer u8TimerID, int32_t i32TimeMs, bool bReload, amo_timer_callback_t pfnCallback)
{
    // Re-calculate timer value for current tick interval
    int32_t i32ConvertedTime = (i32TimeMs * 1000) / (CY_SWTMR_TICK_INTERVAL_US);
    
    m_astcTimers[u8TimerID].i32Timer    = i32ConvertedTime;
    m_astcTimers[u8TimerID].i32Reload   = (bReload == true) ? i32ConvertedTime : -1;
    m_astcTimers[u8TimerID].bElapsed    = false;
    m_astcTimers[u8TimerID].pfnCallback = pfnCallback;
}

/**
 *****************************************************************************
 ** Stops a SW timer, indexed by an ID.
 **
 ** \param u8TimerID   0 to (CY_SWTMR_MAX_TIMERS - 1)
 **
 ** \retval AMO_TIMER_SUCCESS    Timer stopped
 ** \retval AMO_TIMER_BAD_PARAM  u8TimerID invalid
 *****************************************************************************/
amo_timer_en_result_t Amo_timer_Stop(Amotimer u8TimerID)
{
		amo_timer_en_result_t enResult = AMO_TIMER_SUCCESS;
		
    if (u8TimerID >= TIMER_MAX)
    {
        enResult = AMO_TIMER_BAD_PARAM;
    }
    
    m_astcTimers[u8TimerID].i32Timer    = -1;
    m_astcTimers[u8TimerID].i32Reload   = -1;
    m_astcTimers[u8TimerID].bElapsed    = false;
    m_astcTimers[u8TimerID].pfnCallback = NULL;

	m_u8CallbackFunctionTrigger = 0;
	
    return enResult;
}

/**
 *****************************************************************************
 ** Status a SW timer, indexed by an ID.
 **
 ** \param u8TimerID   0 to (CY_SWTMR_MAX_TIMERS - 1)
 **
 ** \retval AMO_TIMER_SUCCESS    Timer active
 ** \retval AMO_TIMER_BAD_PARAM  u8TimerID invalid
 *****************************************************************************/
amo_timer_en_result_t Amo_timer_Status(Amotimer u8TimerID)
{
	amo_timer_en_result_t enResult = AMO_TIMER_SUCCESS;
	
	if (u8TimerID >= TIMER_MAX)
	{
		enResult = AMO_TIMER_BAD_PARAM;
	}

	if(m_astcTimers[u8TimerID].pfnCallback == NULL)
	{
		enResult = AMO_TIMER_BAD_PARAM;	
	}

	return enResult;
}

/**
 *****************************************************************************
 ** Function must be called cyclically to use callback functions of
 ** non-high-prio SW timers. If there are no callback functions in use, or only
 ** high-prio timers are used, there is no need to call Cy_SwTmr_Main().
 **
 ** \return none
 *****************************************************************************/
void Amo_timer_Main( void )
{
    static uint8_t u8PrevTrigger = 0;
    uint8_t u8Trigger = m_u8CallbackFunctionTrigger;
		bool bProcessRequired = false;
		
		if (u8PrevTrigger != u8Trigger)
		{
			u8PrevTrigger = u8Trigger;
			bProcessRequired = true;
		}
		
    // Look for callback functions to call
    if (bProcessRequired == true)
    {
	    for (uint8_t u8TimerId = 0; u8TimerId < (uint8_t)CY_SWTMR_MAX_TIMERS; u8TimerId++)
	    {
	        // If this timer is elapsed and it is a "standard" timer
	        if ((m_astcTimers[u8TimerId].bElapsed == true) && (m_astcTimers[u8TimerId].bHighPrio == false))
	        {
	            // Call function
	            if (m_astcTimers[u8TimerId].pfnCallback != NULL)
	            {
	                // Clear "Elapsed" flag before a possible long function call
	                // (so it may be set again while calling)
	                m_astcTimers[u8TimerId].bElapsed = false;
	                // Call function
	                m_astcTimers[u8TimerId].pfnCallback() ;
	            }
	        }
	    }
    }
}


/**
 *****************************************************************************
 ** Returns the remaining time of a timer.
 **
 ** \param u8TimerID 0 to (CY_SWTMR_MAX_TIMERS - 1)
 **
 ** \return int32_t remaining time in milliseconds or "-1" for an invalid timer ID
 *****************************************************************************/
int32_t Amo_timer_Remaining(Amotimer u8TimerID)
{
		int32_t ret = -1;
		
    if (u8TimerID < TIMER_MAX)
    {
        // Re-calculate from current tick interval to milliseconds
        ret = m_astcTimers[u8TimerID].i32Timer / (1000 / (CY_SWTMR_TICK_INTERVAL_US));
    }

    return ret;
}

/**
 *****************************************************************************
 ** Checks, whether a timer is elapsed or not.
 **
 ** \param u8TimerID          0 to (CY_SWTMR_MAX_TIMERS - 1)
 ** \param bClearElapsedFlag  clear elapsed flag yes/no <=> true/false
 **
 ** \return bool     timer is elapsed yes/no <=> true/false (or false for an invalid timer ID)
 *****************************************************************************/
bool Amo_timer_IsElapsed(Amotimer u8TimerID, bool bClearElapsedFlag)
{
    bool bElapsed = false;

    if (u8TimerID < TIMER_MAX)
    {
        bElapsed = m_astcTimers[u8TimerID].bElapsed;
        // Before clearing, check the mirrored bElapsed, to avoid
        // clearing the real flag just in the case it was set in the meantime
        if ((bClearElapsedFlag == true) && (bElapsed == true))
        {
            m_astcTimers[u8TimerID].bElapsed = false;
        }
    }
    return bElapsed;
}

/**
 *****************************************************************************
 ** Starts a timer and waits synchronously until it is elapsed.
 **
 ** \param u8TimerID               0 to (CY_SWTMR_MAX_TIMERS - 1)
 ** \param i32TimeMs               Duration in ms (must be greater than or equal 0)
 ** \param pfnWatchdogCallout      Function to handle the watchdog, can be NULL
 **
 ** \retval AMO_TIMER_SUCCESS    Timer started
 ** \retval AMO_TIMER_BAD_PARAM  u8TimerID invalid or i32TimeMs invalid
 *****************************************************************************/
amo_timer_en_result_t Amo_timer_Wait(Amotimer u8TimerID, int32_t i32TimeMs, amo_timer_wdg_handle_callout_t pfnWatchdogCallout)
{
    amo_timer_en_result_t enResult;
    
    enResult = Amo_timer_Start(u8TimerID, i32TimeMs, false, NULL);
    
    if(enResult == AMO_TIMER_SUCCESS)
    {    
	    while(Amo_timer_IsElapsed(u8TimerID, true) == false)
	    {
	        // Call watchdog handle function if configured
	        if (pfnWatchdogCallout != NULL)
	        {
	            pfnWatchdogCallout();
	        }
	    }
			enResult = AMO_TIMER_SUCCESS;
    }
    
    return enResult;
}

/**
 *****************************************************************************
 ** \brief Get the global tick count value.
 **
 ** The global tick count value will be reset by calling Cy_SwTmr_Init() and
 ** will go on counting infinitely.
 ** The resolution of this counter is CY_SWTMR_TICK_INTERVAL_US.
 **
 ** \return 64-bit tick count value in microseconds
 *****************************************************************************/
uint64_t Amo_timer_GetTickCountUs(void)
{
    return m_u64TickCountUs;
}

/**
 *****************************************************************************
 ** \brief Check whether a certain time span is elapsed.
 **
 ** Call this function with a previously stored tick count value to determine
 ** whether a time span is elapsed or not.
 **
 ** This function can be used to realize easy timeout checks without using
 ** a dedicated timer ID.
 **
 ** The resolution of the time span is based on CY_SWTMR_TICK_INTERVAL_US.
 **
 ** \param u64TickCountUs   Tick count value previously determined by Cy_SwTmr_GetTickCountUs()
 ** \param u32TimeSpanUs    Time span in microseconds
 **
 ** \retval true    u32TimeSpanUs is elapsed since time stamp u64TickCountUs
 ** \retval false   u32TimeSpanUs is not elapsed since time stamp u64TickCountUs
 *****************************************************************************/
bool Amo_timer_IsTimeSpanElapsed(uint64_t u64TickCountUs, uint32_t u32TimeSpanUs)
{
    bool bElapsed = false;
    uint64_t u64CurrentTickCount = Amo_timer_GetTickCountUs();
    
    // If current time stamp is older or equal than u64TickCountUs
    if (u64CurrentTickCount >= u64TickCountUs)
    {
        // Determine elapsed time
        uint32_t u32ElapsedTimeUs = (uint32_t)((uint64_t)u64CurrentTickCount - (uint64_t)u64TickCountUs);
        // If elapsed time is greater or equal than time span, return true
        if (u32ElapsedTimeUs >= u32TimeSpanUs)
        {
            bElapsed = true;
        }
    }
    return bElapsed;
}

/**
 *****************************************************************************
 ** Callback function from SysTick driver
 **
 ** \return none
 *****************************************************************************/
static void Amo_timer_callback(void)
{
	LPIT_DRV_ClearInterruptFlagTimerChannels(INST_LPIT1, (1U << LPIT_CHANNEL));

  // Increment global tick counter
  m_u64TickCountUs += CY_SWTMR_TICK_INTERVAL_US;

	// Countdown all running timers
	for (uint8_t u8TimerId = 0; u8TimerId < (uint8_t)CY_SWTMR_MAX_TIMERS; u8TimerId++)
	{
	    // If timer is disabled, continue
	    if (m_astcTimers[u8TimerId].i32Timer < 0)
	    {
	        continue;
	    }
	        
	    if (m_astcTimers[u8TimerId].i32Timer > 0)
	    {
	        m_astcTimers[u8TimerId].i32Timer--;
	    }
	    
	    if (m_astcTimers[u8TimerId].i32Timer == 0)
	    {
	        // Set marker for processing in Cy_SwTmr_Main()
	        if(m_astcTimers[u8TimerId].bHighPrio == false)
	        {
	            m_astcTimers[u8TimerId].bElapsed = true;
	            m_u8CallbackFunctionTrigger++;
	        }
	        else
	        {
	            // Call user callback directly from IRQ context
	            if (m_astcTimers[u8TimerId].pfnCallback != NULL)
	            {
	                m_astcTimers[u8TimerId].pfnCallback();
	            }
	            m_astcTimers[u8TimerId].bElapsed = false;
	        }

	        // Check, whether to reload the timer or not
	        if (m_astcTimers[u8TimerId].i32Reload > 0)
	        {
	            m_astcTimers[u8TimerId].i32Timer = m_astcTimers[u8TimerId].i32Reload;
	        }
	        else
	        {
	            m_astcTimers[u8TimerId].i32Timer = -1;    // disable timer (single-shot mode)
	        }
	    }
	}
	Amo_timer_Main();
}


#if defined(__cplusplus)
}
#endif


/* [] END OF FILE */

