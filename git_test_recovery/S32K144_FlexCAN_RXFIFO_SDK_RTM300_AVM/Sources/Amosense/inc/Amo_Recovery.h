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

typedef enum
{
		SLAVE_NONE_FLAG = 0,
		SLAVE_SLLR_FLAG = 1 << 0,	
    SLAVE_SLUD_FLAG = 1 << 1,
    SLAVE_CLLR_FLAG = 1 << 2,
    SLAVE_CLUD_FLAG = 1 << 3,
    SLAVE_CRLR_FLAG = 1 << 4,
    SLAVE_CRUD_FLAG = 1 << 5,
    SLAVE_SRLR_FLAG = 1 << 6,
    SLAVE_SRUD_FLAG = 1 << 7
} SlaveId_Flag_t;


typedef struct
{
    uint16_t prev_position;
    uint16_t curr_position;
    uint16_t target_position;
    uint8_t recovery_attempts;
    bool in_recovery;
		bool cant_reach_zeroPt;
		bool reach_zeroPt;
} SlaveState_t;

#ifdef	Amo_RECOVERY_C_

SlaveId_Flag_t slaveId_flag = SLAVE_NONE_FLAG;

void process_slave_command(SlaveId_t id, Lin_EVntPt_t *lin_Pt, uint8_t rpm);
void handle_slave_recovery(SlaveId_t id);
void recovery_mode_check(SlaveId_Flag_t slaveId_flag);
void recovery_mode_step(SlaveId_t id, uint8_t rpm);





#else

extern SlaveId_Flag_t slaveId_flag;

extern void process_slave_command(SlaveId_t id, Lin_EVntPt_t *lin_Pt, uint8_t rpm);
extern void handle_slave_recovery(SlaveId_t id);
extern void recovery_mode_check(SlaveId_Flag_t slaveId_flag);
extern void recovery_mode_step(SlaveId_t id, uint8_t rpm);

#endif /* Amo_RECOVERY_C_ */


#endif /* Amo_RECOVERY_H_ */



