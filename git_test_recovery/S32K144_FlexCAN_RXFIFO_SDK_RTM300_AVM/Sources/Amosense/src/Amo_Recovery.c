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


#define MAX_RECOVERY_ATTEMPTS (2U) //(5U)
#define NUM_SLAVES (8U)

static SlaveState_t slave_states[NUM_SLAVES];

/* Function Prototypes */
static uint32_t get_opdone_timeout(uint16_t moved, uint8_t rpm);
static bool get_opdone(SlaveId_t id);
static uint16_t get_act_state(SlaveId_t id);
static void send_move_command(SlaveId_t id, uint16_t target, lin_LH_EVNT_MASTER_CMD_11 *lin_Ctrl_lh, lin_RH_EVNT_MASTER_CMD_21 *lin_Ctrl_rh);
static uint16_t calc_abs_moved(uint16_t target, uint16_t prev);
static void start_slave_timer(SlaveId_t id, uint16_t target, uint16_t prev, uint8_t rpm);

/* Helper: get OpDone timeout based on moved distance and RPM */
static uint32_t get_opdone_timeout(uint16_t moved, uint8_t rpm)
{
    uint32_t timeout = 13833U;
    if (rpm == 10U)
    {
        if (moved <= 350U)
        {
            timeout = 1500U;
        }
        else if (moved <= 450U)
        {
            timeout = 1700U;
        }
        else if (moved <= 700U)
        {
            timeout = 2100U;
        }
        else if (moved <= 900U)
        {
            timeout = 2500U;
        }
        else if (moved <= 1600U)
        {
            timeout = 3500U;
        }
        else
        {
            /* Default: already set to 13833U */
        }
    }
    else if (rpm == 3U)
    {
        if (moved <= 350U)
        {
            timeout = 2444U;
        }
        else if (moved <= 450U)
        {
            timeout = 3000U;
        }
        else if (moved <= 700U)
        {
            timeout = 4389U;
        }
        else if (moved <= 900U)
        {
            timeout = 5500U;
        }
        else if (moved <= 1600U)
        {
            timeout = 9389U;
        }
        else
        {
            /* Default: already set to 13833U */
        }
    }
    else if (rpm == 2U)
    {
        if (moved <= 350U)
        {
            timeout = 3900U;
        }
        else if (moved <= 450U)
        {
            timeout = 4700U;
        }
        else if (moved <= 700U)
        {
            timeout = 6800U;
        }
        else if (moved <= 900U)
        {
            timeout = 8500U;
        }
        else if (moved <= 1600U)
        {
            timeout = 14300U;
        }
        else
        {
            /* Default: already set to 13833U */
        }
    }
    else
    {
        /* Default: already set to 13833U */
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
    uint16_t act_state = 0U;
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
    /* (void)target; If not used in your LIN command, suppress unused warning */
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
            else
            {
                /* SLAVE_CLUD */
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
            else
            {
                /* SLAVE_SRUD */
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
    SlaveState_t *state = NULL;
    bool opdone = false;
    uint16_t act_state = 0U;
    if (id < NUM_SLAVES)
    {
        state = &slave_states[id];
        opdone = get_opdone(id);
        act_state = get_act_state(id);
        if (!opdone)
        {
            if(state->recovery_attempts < MAX_RECOVERY_ATTEMPTS)
            {
                /* Not arrived at target within timeout: enter Recovery Mode */
                state->in_recovery = true;
                state->recovery_attempts++;
                SetLIN_EVNT_FrDrSPEED(&LIN_LH_EVNT_MASTER_CMD, ACT_RPM_2);
                send_move_command(id, 0U, &LIN_LH_EVNT_MASTER_CMD, &LIN_RH_EVNT_MASTER_CMD);
                if(state->target_position < 3000U)
                {
                    if(state->prev_position < 3000U)
                    {
                        if(act_state < state->prev_position)
                        {
                            state->cant_reach_zeroPt = true;
                        }
												else
												{
														state->reach_zeroPt = true;
												}
                    }
                    else
                    {
                        if(act_state > state->prev_position)
                        {
                            state->cant_reach_zeroPt = true;
                        }
												else
												{
														state->reach_zeroPt = true;
												}
                    }
                }
                else if(state->target_position > 3000U)
                {
                    if(state->prev_position > 3000U)
                    {
                        if(act_state > state->prev_position)
                        {
                            state->cant_reach_zeroPt = true;
                        }
												else
												{
														state->reach_zeroPt = true;
												}
                    }
                    else
                    {
                        if(act_state < state->prev_position)
                        {
                            state->cant_reach_zeroPt = true;
                        }
												else
												{
														state->reach_zeroPt = true;
												}
                    }
                }
                else
                {
                    /* No action needed */
                }
            }
            else
            {
                /* Failed maximum times: issue Hard Stop command */
                /* lin_Write_SpecialCommand_HardStop(id); */
                state->in_recovery = false;
                state->recovery_attempts = 0U;
            }
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
            /* Not implemented */
            break;
        case SLAVE_CLUD:
            /* Not implemented */
            break;
        case SLAVE_CRLR:
            /* Not implemented */
            break;
        case SLAVE_CRUD:
            /* Not implemented */
            break;
        case SLAVE_SRLR:
            /* Not implemented */
            break;
        case SLAVE_SRUD:
            /* Not implemented */
            break;
        default:
            /* Defensive: do nothing */
            break;
    }
}

uint16_t absValue(int16_t num)
{
    uint16_t result = 0U;
    if (num < 0)
    {
        result = (uint16_t)(-num);
    }
    else
    {
        result = (uint16_t)num;
    }
    return result;
}

/* Calculate absolute moved distance, considering wrap-around */
static uint16_t calc_abs_moved(uint16_t target, uint16_t prev)
{
    int16_t moved = (int16_t)((int32_t)target - (int32_t)prev);
    uint16_t abs_moved = absValue(moved);
    uint16_t cal_target_pt = 0U;
    uint16_t cal_prev_pt = 0U;
    if(abs_moved > 3000U)
    {
        if(target > 3000U)
        {
            cal_target_pt = (uint16_t)(4096U - target);
        }
        else
        {
            cal_target_pt = target;
        }
        if(prev > 3000U)
        {
            cal_prev_pt = (uint16_t)(4096U - prev);
        }
        else
        {
            cal_prev_pt = prev;
        }
        abs_moved = (uint16_t)(cal_target_pt + cal_prev_pt);
    }
    return abs_moved;
}

/* Start timer for slave with calculated timeout */
static void start_slave_timer(SlaveId_t id, uint16_t target, uint16_t prev, uint8_t rpm)
{
    uint16_t abs_moved = calc_abs_moved(target, prev);
    uint32_t timeout = get_opdone_timeout(abs_moved, rpm);
    timer_start_recovery(id, timeout);
}

/* Call this when a move command is issued for a slave */
void process_slave_command(SlaveId_t id, Lin_EVntPt_t *lin_Pt, uint8_t rpm)
{
    SlaveState_t *state = NULL;
    uint16_t act_state = 0U;
    if (id < NUM_SLAVES)
    {
        act_state = get_act_state(id);
        state = &slave_states[id];
        state->curr_position = act_state;
        state->prev_position = state->curr_position;
        state->target_position = lin_Pt->FrDrPt.targetPt;
        send_move_command(id, state->target_position, &LIN_LH_EVNT_MASTER_CMD, &LIN_RH_EVNT_MASTER_CMD);
        start_slave_timer(id, state->target_position, state->prev_position, rpm);
    }
}

void recovery_mode_check(SlaveId_Flag_t slaveId_flag)
{
    if((slaveId_flag & SLAVE_SLLR_FLAG) != 0U)
    {
        recovery_mode_step(SLAVE_SLLR, 2U);
    }
    if((slaveId_flag & SLAVE_SLUD_FLAG) != 0U)
    {
        recovery_mode_step(SLAVE_SLUD, 2U);
    }
		if((slaveId_flag & SLAVE_CLLR_FLAG) != 0U)
    {
        recovery_mode_step(SLAVE_CLLR, 2U);
    }
}

/* Call this periodically (e.g., in main loop or timer interrupt) for each slave */
void recovery_mode_step(SlaveId_t id, uint8_t rpm)
{
    SlaveState_t *state = NULL;
    uint16_t act_state = 0U;
    bool opdone = false;
    if (id < NUM_SLAVES)
    {
        act_state = get_act_state(id);
        opdone = get_opdone(id);
        state = &slave_states[id];
        state->curr_position = act_state;
        if (state->in_recovery)
        {
            if(state->cant_reach_zeroPt == true)
            {
                state->cant_reach_zeroPt = false;
                send_move_command(id, state->target_position, &LIN_LH_EVNT_MASTER_CMD, &LIN_RH_EVNT_MASTER_CMD);
                start_slave_timer(id, state->target_position, state->curr_position, rpm);
            }
            else
            {
                if ((opdone == true) && (((state->curr_position >= 0U) && (state->curr_position < 20U)) || ((state->curr_position > 4076U) && (state->curr_position <= 4096U))))
                {
										if(state->reach_zeroPt == true)
										{
												state->reach_zeroPt = false;
		                    send_move_command(id, state->target_position, &LIN_LH_EVNT_MASTER_CMD, &LIN_RH_EVNT_MASTER_CMD);
		                    start_slave_timer(id, state->target_position, state->curr_position, rpm);
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
}

