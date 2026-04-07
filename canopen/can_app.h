#ifndef __CAN_APP_H
#define __CAN_APP_H

#include <stdint.h>

extern volatile uint8_t g_g4_comm_timeout;
extern volatile uint8_t g_g4_feedback_seen;
extern volatile uint8_t g_g4_feedback_online;

void User_Slave_Init(void);
void CAN_App_OnG4FeedbackReceived(void);
void CAN_App_Check_G4_Watchdog(void);

#endif
