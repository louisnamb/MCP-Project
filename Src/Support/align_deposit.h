#ifndef ALIGN_AND_DEPOSIT_H
#define ALIGN_AND_DEPOSIT_H

#include <stdint.h>
#include <stdbool.h>
#include "kobuki.h"        /* also declares the global 'kobuki' used by the sub-FSM */
#include "ultrasonic.h"    /* Ultrasonic_Read() */

void Align_and_Deposit_Init(void);
bool Align_and_Deposit_SubFSM(void);

#endif