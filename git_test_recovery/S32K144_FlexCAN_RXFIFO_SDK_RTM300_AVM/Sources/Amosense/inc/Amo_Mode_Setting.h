/*
 * Amo_Mode_Setting.h
 *
 *  Created on: 2025. 1. 7.
 *      Author: S2402004
 */

#ifndef AMOSENSE_INC_AMO_MODE_SETTING_H_
#define AMOSENSE_INC_AMO_MODE_SETTING_H_

//#include "Amo_STATE_Event.h"

void Hu_Front_Focus_mode_setting(void);
void Hu_Front_Spread_mode_setting(void);
void Hu_Front_Cycle_mode_setting(void);
void Hu_Front_mode_FullClose_setting(void);
void HU_Rear_mode_setting(void);
void Hu_Front_mode_manual_key_setting(void);
void RHD_Hu_Front_Focus_mode_setting(void);
void RHD_Hu_Front_Spread_mode_setting(void);
void RHD_Hu_Front_Cycle_mode_setting(void);
void RHD_Hu_Front_mode_FullClose_setting(void);

extern uint8_t key_full_left_close;
extern uint8_t key_full_right_close;
#endif /* AMOSENSE_INC_AMO_MODE_SETTING_H_ */
