#include "Amo_LIN.h"
#include "Amo_Timer.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define MAX_RECOVERY_ATTEMPTS (5U)
#define NUM_SLAVES (8U)

typedef enum
{
    SLAVE_SLLR = 0U, /* LIN0 */
    SLAVE_SLUD,
    SLAVE_CLLR,
    SLAVE_CLUD,
    SLAVE_CRLR,      /* LIN1 */
    SLAVE_CRUD,
    SLAVE_SRLR,
    SLAVE_SRUD
} SlaveId_t;

typedef struct
{
    uint16_t prev_position;
    uint16_t curr_position;
    uint16_t target_position;
    uint8_t recovery_attempts;
    bool in_recovery;
} SlaveState_t;

static SlaveState_t slave_states[NUM_SLAVES];

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

/* Helper: send move command for each slave */
static void send_move_command(SlaveId_t id, uint16_t target)
{
    (void)target; /* If not used in your LIN command, suppress unused warning */
    switch (id)
    {
        case SLAVE_SLLR:
        case SLAVE_SLUD:
        case SLAVE_CLLR:
        case SLAVE_CLUD:
            /* LIN0 */
            lin_Write_FrDrEVntPt_XY(&LIN_LH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
            break;
        case SLAVE_CRLR:
        case SLAVE_CRUD:
        case SLAVE_SRLR:
        case SLAVE_SRUD:
            /* LIN1 */
            lin_Write_FrPsEVntPt_XY(&LIN_RH_EVNT_MASTER_CMD, &Can_Tx_Evnt_3);
            break;
        default:
            /* Defensive: do nothing */
            break;
    }
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
            state->target_position = 0U;
            send_move_command(id, 0U);
        }
        else
        {
            /* Arrived at target: exit Recovery Mode */
            state->in_recovery = false;
            state->recovery_attempts = 0U;
        }
    }
}

/* Call this when a move command is issued for a slave */
void process_slave_command(SlaveId_t id, uint16_t target, uint8_t rpm, uint16_t curr_position)
{
    SlaveState_t *state;
    uint16_t moved;
    uint32_t timeout;

    if (id < NUM_SLAVES)
    {
        state = &slave_states[id];
        state->prev_position = state->curr_position;
        state->curr_position = curr_position;
        state->target_position = target;

        send_move_command(id, target);

        moved = (uint16_t)abs((int32_t)target - (int32_t)state->prev_position);
        timeout = get_opdone_timeout(moved, rpm);

        /* Start timer for this slave (use a unique timer/channel per slave if possible) */
        /* If your timer API supports passing an argument, pass 'id' to the callback */
        Amo_timer_Start(/*timer_id_for_slave[id]*/, timeout, true, /*callback for this slave*/);
    }
}

/* Call this periodically (e.g., in main loop or timer interrupt) for each slave */
void recovery_mode_step(SlaveId_t id, uint16_t curr_position, uint8_t rpm)
{
    SlaveState_t *state;
    uint16_t moved;
    uint32_t timeout;

    if (id < NUM_SLAVES)
    {
        state = &slave_states[id];
        state->curr_position = curr_position;

        if (state->in_recovery)
        {
            if (state->curr_position == 0U)
            {
                /* Reached zero, try to move to target again */
                if (state->recovery_attempts < MAX_RECOVERY_ATTEMPTS)
                {
                    send_move_command(id, state->target_position);
                    moved = (uint16_t)abs((int32_t)state->target_position - 0);
                    timeout = get_opdone_timeout(moved, rpm);
                    Amo_timer_Start(/*timer_id_for_slave[id]*/, timeout, true, /*callback for this slave*/);
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