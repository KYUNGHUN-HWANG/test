#define Amo_LPTMR_C_

#include "Cpu.h"
#include "Amo_LPTMR.h"

/* (CLK (MHz)* timer period (us) / Prescaler) */
#define TIMER_COMPARE_VAL (uint16_t)(2000U)
#define TIMER_TICKS_1US   (uint16_t)(4U)

void LPTMR_ISR(void);
uint32_t timerGetTimeIntervalCallback0(uint32_t *ns);
void Lptmr_Init(void);

uint16_t timerOverflowInterruptCount = 0U;
uint16_t timerOverflowInterruptCount_1 = 0U;
uint16_t timerOverflowInterruptCount_2 = 0U;


uint32_t NULL__ZERO(uint32_t *ns) { return 0; }
uint32_t NULL_ZERO(uint32_t *ns) { return 0; }

/*!
 * @brief LPTMR Interrupt Service Routine
 * The ISR will call LIN timeout service every 500us
 * and will provide the required tick for LIN (every 5 ms)
 */
void LPTMR_ISR(void)
{
	/* Static variable, used to count if the timeout has passed to
	 * provide the LIN scheme tick.
	 */
	static uint32_t interruptCount = 0UL;
	static uint32_t interruptCount_1 = 0UL;

    /* Timer Interrupt Handler */
    lin_lld_timeout_service(LI0);
    lin_lld_timeout_service(LI1);

    /* If 5 ms have passed, provide the required tick */
		++interruptCount;
    if(interruptCount == 10UL)
    {
    	l_sch_tick(LI0);
    	interruptCount = 0UL;
    }
		++interruptCount_1;
    if(interruptCount_1 == 10UL)
    {
    	l_sch_tick(LI1);
    	interruptCount_1 = 0UL;
    }

    /* Increment overflow count */
//    timerOverflowInterruptCount++;
//    timerOverflowInterruptCount_1++;
//    timerOverflowInterruptCount_2++;

	/* Clear compare flag */
	LPTMR_DRV_ClearCompareFlag(INST_LPTMR1);

}


/*!
 * @brief Callback function to get time interval in nano seconds
 * @param[out] ns - number of nanoseconds passed since the last call of the function
 * @return dummy value
 */
uint32_t timerGetTimeIntervalCallback0(uint32_t *ns)
{
	static uint32_t previousCountValue = 0UL;
	uint32_t counterValue;

	counterValue = LPTMR_DRV_GetCounterValueByCount(INST_LPTMR1);
	*ns = (((((uint32_t)counterValue + (uint32_t)timerOverflowInterruptCount) * ((uint32_t)TIMER_COMPARE_VAL - (uint32_t)previousCountValue)) * 1000) / TIMER_TICKS_1US);
	timerOverflowInterruptCount = (uint16_t)0UL;
	previousCountValue = counterValue;
	return 0UL;
}


void Lptmr_Init(void)
{
	/* Initialize LPTMR */
	LPTMR_DRV_Init(INST_LPTMR1, &lpTmr1_config0, false);
	INT_SYS_InstallHandler(LPTMR0_IRQn, LPTMR_ISR, (isr_t *)NULL);
	INT_SYS_EnableIRQ(LPTMR0_IRQn);
	LPTMR_DRV_StartCounter(INST_LPTMR1);
}


