#include <stdint.h>
#define STM32G474xx
#include "stm32g4xx.h"
#include "uart.h"
#include "kobuki.h"
#include "adc.h"

typedef enum
{
    MODE_INPUT = 0,
    MODE_OUTPUT = 1,
    MODE_ALT = 2,
    MODE_ANALOG = 3,
    MODE_PULLDOWN = 2,
    MODE_PULLUP = 1
} PinMode;

// Scan structure to hold decoded scan data
typedef struct {
    uint32_t pos;
    uint32_t curr_min_value;
    uint32_t curr_min_index;
    uint32_t arc_positions[200];
} Scan_Typedef;

typedef enum
{
    IDLE = 0,
    SCAN = 1,
    NAVIGATE = 2,
    ALIGN_AND_DEPOSIT = 3,
    STOP_FAUL = 4,
    MODE_PULLUP = 5
} STATE;

void Port_Clock_Enable(void);
void idle_state();