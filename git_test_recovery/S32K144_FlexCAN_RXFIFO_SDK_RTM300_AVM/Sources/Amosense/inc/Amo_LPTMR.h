#ifndef Amo_LPTMR_H_
#define Amo_LPTMR_H_

#ifdef	Amo_LPTMR_C_
uint32_t NULL_ZERO(uint32_t *ns);
uint32_t NULL__ZERO(uint32_t *ns);
void Lptmr_Init(void);
void LPTMR_ISR(void);
#else
extern uint32_t NULL_ZERO(uint32_t *ns);
extern uint32_t NULL__ZERO(uint32_t *ns);
extern void Lptmr_Init(void);
extern void LPTMR_ISR(void);
#endif /* Amo_LPTMR_C_ */

extern uint16_t timerOverflowInterruptCount;
extern uint16_t timerOverflowInterruptCount_1;
extern uint16_t timerOverflowInterruptCount_2;

#endif /* Amo_LPTMR_H_ */

