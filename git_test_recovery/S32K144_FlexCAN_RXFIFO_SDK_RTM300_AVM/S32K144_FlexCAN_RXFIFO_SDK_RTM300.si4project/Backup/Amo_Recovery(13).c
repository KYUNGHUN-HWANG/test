#define Amo_RECOVERY_C_

#include "Cpu.h"
#include "Amo_main.h"
#include "Amo_Timer.h"

/*
#include "Amo_LIN.h"
#include "Amo_Timer.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
*/


#define MAX_RECOVERY_ATTEMPTS (5U)
#define NUM_SLAVES (8U)

static SlaveState_t slave_states[NUM_SLAVES];

//SlaveId_Flag_t slaveId_flag = SLAVE_NONE_FLAG;


/* Helper: get OpDone timeout based on moved distance and RPM */
static uint32_t get_opdone_timeout(uint16_t moved, uint8_t rpm)
{
    uint32_t timeout = 3500U;
    if (rpm == 10U)
    {
        if (moved <= 300U)
        {
            timeout = 1500U;
        }
        else if (moved <= 600U)
        {
            timeout = 2000U;
        }
        else if (moved <= 1600U)
        {
            timeout = 3500U;
        }
    }
    else if (rpm == 3U)
    {
        if (moved <= 300U)
        {
            timeout = 3000U;
        }
        else if (moved <= 600U)
        {
            timeout = 4000U;
        }
        else if (moved <= 1600U)
        {
            timeout = 7000U;
        }
    }
		else if (rpm == 2U)
    {
        if (moved <= 300U)
        {
            timeout = 4000U;
        }
        else if (moved <= 600U)
        {
            timeout = 5000U;
        }
        else if (moved <= 1600U)
        {
            timeout = 8000U;
        }
    }
    else
    {
        /* Default: already set to 3500U */
    }
    return timeout;
}

/* Helper: get OpDone signal for each slave */
static bool get_opdone(SlaveId_t id)
{
    bool opdone = false;
    switch (id)
    {
        case SLAVE_SLLR:
            opdone = l_bool_rd_LI0_FR_SLLR_OpDone();
            break;
        case SLAVE_SLUD:
            opdone = l_bool_rd_LI0_FR_SLUD_OpDone();
            break;
        case SLAVE_CLLR:
            opdone = l_bool_rd_LI0_FR_CLLR_OpDone();
            break;
        case SLAVE_CLUD:
            opdone = l_bool_rd_LI0_FR_CLUD_OpDone();
            break;
        case SLAVE_CRLR:
            opdone = l_bool_rd_LI1_FR_CRLR_OpDone();
            break;
        case SLAVE_CRUD:
            opdone = l_bool_rd_LI1_FR_CRUD_OpDone();
            break;
        case SLAVE_SRLR:
            opdone = l_bool_rd_LI1_FR_SRLR_OpDone();
            break;
        case SLAVE_SRUD:
            opdone = l_bool_rd_LI1_FR_SRUD_OpDone();
            break;
        default:
            /* Defensive: do nothing */
            break;
    }
    return opdone;
}

/* Helper: get actuator_state signal for each slave */
static uint16_t get_act_state(SlaveId_t id)
{
    uint16_t act_state = 0;
    switch (id)
    {
        case SLAVE_SLLR:
            act_state = l_u16_rd_LI0_FR_SLLR_ActuatorState();
            break;
        case SLAVE_SLUD:
            act_state = l_u16_rd_LI0_FR_SLUD_ActuatorState();
            break;
        case SLAVE_CLLR:
            act_state = l_u16_rd_LI0_FR_CLLR_ActuatorState();
            break;
        case SLAVE_CLUD:
            act_state = l_u16_rd_LI0_FR_CLUD_ActuatorState();
            break;
        case SLAVE_CRLR:
            act_state = l_u16_rd_LI1_FR_CRLR_ActuatorState();
            break;
        case SLAVE_CRUD:
            act_state = l_u16_rd_LI1_FR_CRUD_ActuatorState();
            break;
        case SLAVE_SRLR:
            act_state = l_u16_rd_LI1_FR_SRLR_ActuatorState();
            break;
        case SLAVE_SRUD:
            act_state = l_u16_rd_LI1_FR_SRUD_ActuatorState();
            break;
        default:
            /* Defensive: do nothing */
            break;
    }
    return act_state;
}


/* Helper: send move command for each slave */
static void send_move_command(SlaveId_t id, uint16_t target, lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl_lh, lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl_rh)
{
//    (void)target; /* If not used in your LIN command, suppress unused warning */
		
    switch (id)
    {
        case SLAVE_SLLR:
        case SLAVE_SLUD:
        case SLAVE_CLLR:
        case SLAVE_CLUD:
						/* LIN0 */
						if(id == SLAVE_SLLR)
						{
							lin_Ctrl_lh->LDATA.EVNT_Front_Side_LH_LeftRight_TargetPosition = target;
							slaveId_flag |= SLAVE_SLLR_FLAG;
						}
						else if(id == SLAVE_SLUD)
						{
							lin_Ctrl_lh->LDATA.EVNT_Front_Side_LH_UpDown_TargetPosition = target;
							slaveId_flag |= SLAVE_SLUD_FLAG;
						}
						else if(id == SLAVE_CLLR)
						{
							lin_Ctrl_lh->LDATA.EVNT_Front_Center_LH_LeftRight_TargetPosition = target;
						}
						else //SLAVE_CLUD
						{
							lin_Ctrl_lh->LDATA.EVNT_Front_Center_LH_UpDown_TargetPosition = target;
						}    
            lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
            break;
						
        case SLAVE_CRLR:
        case SLAVE_CRUD:
        case SLAVE_SRLR:
        case SLAVE_SRUD:
            /* LIN1 */
						if(id == SLAVE_CRLR)
						{
							lin_Ctrl_rh->LDATA.EVNT_Front_Center_RH_LeftRight_TargetPosition = target;
						}
						else if(id == SLAVE_CRUD)
						{
							lin_Ctrl_rh->LDATA.EVNT_Front_Center_RH_UpDown_TargetPosition = target;
						}
						else if(id == SLAVE_SRLR)
						{
							lin_Ctrl_rh->LDATA.EVNT_Front_Side_RH_LeftRight_TargetPosition = target;
						}
						else //SLAVE_SRUD
						{
							lin_Ctrl_rh->LDATA.EVNT_Front_Side_RH_UpDown_TargetPosition = target;
						}
            lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
            break;
						
        default:
            /* Defensive: do nothing */
            break;
    }
		
}


void handle_slave_recovery_sllr(void)
{
	handle_slave_recovery(SLAVE_SLLR);
}

void handle_slave_recovery_slud(void)
{
	handle_slave_recovery(SLAVE_SLUD);
}

/* Timer callback for each slave (pass slave id as argument if your timer supports it) */
void handle_slave_recovery(SlaveId_t id)
{
    SlaveState_t *state;
    bool opdone;

    if (id < NUM_SLAVES)
    {
        state = &slave_states[id];
        opdone = get_opdone(id);

        if (!opdone)
        {
            /* Not arrived at target within timeout: enter Recovery Mode */
            state->in_recovery = true;
            state->recovery_attempts++;
//            state->target_position = 0U;

						SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_2);
            send_move_command(id, 0U, &LIN_LH_EVNT_MASTER_CMD, &LIN_RH_EVNT_MASTER_CMD);
        }
        else
        {
            /* Arrived at target: exit Recovery Mode */
            state->in_recovery = false;
            state->recovery_attempts = 0U;
        }
    }
}


void timer_start_recovery(SlaveId_t id,  uint32_t timeout)
{
		switch (id)
    {
        case SLAVE_SLLR:
						Amo_timer_Stop(timer_91);
						Amo_timer_Start(timer_91, timeout, false, handle_slave_recovery_sllr);
						break;
        case SLAVE_SLUD:
						Amo_timer_Stop(timer_92);
						Amo_timer_Start(timer_92, timeout, false, handle_slave_recovery_slud);
						break;
        case SLAVE_CLLR:
						break;
        case SLAVE_CLUD:
            break;
        case SLAVE_CRLR:
						break;
        case SLAVE_CRUD:
						break;
        case SLAVE_SRLR:
						break;
        case SLAVE_SRUD:
            break;
        default:
            /* Defensive: do nothing */
            break;
    }
}

uint16_t absValue(int16_t num)
{
	return (num < 0) ? -num : num;
}

/* Call this when a move command is issued for a slave */

//void process_slave_command(SlaveId_t id, uint16_t target, uint8_t rpm, uint16_t curr_position)
void process_slave_command(SlaveId_t id, Lin_EVntPt_t *lin_Pt, uint8_t rpm)
{
    SlaveState_t *state;
    int16_t moved;
		uint16_t abs_moved;
    uint32_t timeout;
		uint16_t act_state;

    if (id < NUM_SLAVES)
    {
				act_state = get_act_state(id);
			
        state = &slave_states[id];
				state->curr_position = act_state;
        state->prev_position = state->curr_position;
//        state->curr_position = act_state;
        state->target_position = lin_Pt->FrDrPt.targetPt;

        send_move_command(id, state->target_position, &LIN_LH_EVNT_MASTER_CMD, &LIN_RH_EVNT_MASTER_CMD);

//        moved = (uint16_t)abs((int32_t)target - (int32_t)state->prev_position);

				moved = (state->target_position) - (state->prev_position);
				abs_moved = absValue(moved);
				
				if(abs_moved > 3000)
				{
					if(state->target_position > 3000)
					{
						state->target_position = 4096-(state->target_position);
					}
					if(state->prev_position > 3000)
					{
						state->prev_position = 4096-(state->prev_position);
					}
					
					abs_moved = (state->target_position) + (state->prev_position);
				}

        timeout = get_opdone_timeout(abs_moved, rpm);

        /* Start timer for this slave (use a unique timer/channel per slave if possible) */
        /* If your timer API supports passing an argument, pass 'id' to the callback */

				timer_start_recovery(id, timeout);
				
//        Amo_timer_Start(/*timer_id_for_slave[id]*/, timeout, false, /*callback for this slave*/);
    }
}


void recovery_mode_check(SlaveId_Flag_t slaveId_flag)
{
	if(slaveId_flag & SLAVE_SLLR_FLAG)
	{
		recovery_mode_step(SLAVE_SLLR, 2);
	}
	if(slaveId_flag & SLAVE_SLUD_FLAG)
	{
		recovery_mode_step(SLAVE_SLUD, 2);
	}
}

/* Call this periodically (e.g., in main loop or timer interrupt) for each slave */
void recovery_mode_step(SlaveId_t id, uint8_t rpm)
{
    SlaveState_t *state;
    int16_t moved;
		uint16_t abs_moved;
    uint32_t timeout;
		uint16_t act_state;
		bool opdone;

    if (id < NUM_SLAVES)
    {
				act_state = get_act_state(id);
				opdone = get_opdone(id);
				
        state = &slave_states[id];
        state->curr_position = act_state;

        if (state->in_recovery)
        {
            if ((opdone == true) && ((state->curr_position >= 0 && state->curr_position < 30) || (state->curr_position >= 4096 && state->curr_position < 4066)))
            {
                /* Reached zero, try to move to target again */
                if (state->recovery_attempts < MAX_RECOVERY_ATTEMPTS)
                {
                    send_move_command(id, state->target_position, &LIN_LH_EVNT_MASTER_CMD, &LIN_RH_EVNT_MASTER_CMD);

										moved = (state->target_position) - (state->curr_position);
										abs_moved = absValue(moved);
										
										if(abs_moved > 3000)
										{
											if(state->target_position > 3000)
											{
												state->target_position = 4096-(state->target_position);
											}
											if(state->curr_position > 3000)
											{
												state->curr_position = 4096-(state->curr_position);
											}
											
											abs_moved = (state->target_position) + (state->curr_position);
										}

						        timeout = get_opdone_timeout(abs_moved, rpm);

						        /* Start timer for this slave (use a unique timer/channel per slave if possible) */
						        /* If your timer API supports passing an argument, pass 'id' to the callback */

										timer_start_recovery(id, timeout);

										
//                    moved = (uint16_t)abs((int32_t)state->target_position - 0);
//                    timeout = get_opdone_timeout(moved, rpm);
//                    Amo_timer_Start(/*timer_id_for_slave[id]*/, timeout, true, /*callback for this slave*/);
                }
                else
                {
                    /* Failed 5 times: issue Hard Stop command */
                    /* lin_Write_SpecialCommand_HardStop(id); */
                    state->in_recovery = false;
                }
            }
            else if (state->curr_position == state->target_position)
            {
                /* Arrived at target: exit Recovery Mode */
                state->in_recovery = false;
                state->recovery_attempts = 0U;
            }
            else
            {
                /* Still moving: do nothing */
            }
        }
    }
}
