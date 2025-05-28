#ifndef Amo_RECOVERY_H_ /* to interprete header file only once */
#define Amo_RECOVERY_H_

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

#ifdef	Amo_RECOVERY_C_
void process_slave_command(SlaveId_t id, Lin_EVntPt_t *lin_Pt, uint8_t rpm);
void handle_slave_recovery(SlaveId_t id);

#else
extern void process_slave_command(SlaveId_t id, Lin_EVntPt_t *lin_Pt, uint8_t rpm);
extern void handle_slave_recovery(SlaveId_t id);

#endif /* Amo_RECOVERY_C_ */


#endif /* Amo_RECOVERY_H_ */



