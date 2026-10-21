#include <stdint.h>
#define STM32G474xx
#include "stm32g4xx.h"
#include "uart.h"
#include "kobuki.h"
#include "adc.h"
#include <stdbool.h>

typedef enum
{
    MODE_INPUT_ = 0,
    MODE_OUTPUT = 1,
    MODE_ALT = 2,
    MODE_ANALOG = 3,
    MODE_PULLDOWN = 2,
    MODE_PULLUP = 1
} PinMode;

// Define master struct
typedef struct {
    uint16_t arc_positions[100];
    uint16_t index_pos; //array index
    uint16_t curr_min_value; //min array value
    uint16_t curr_min_index; //min array index
    uint8_t curr_state; //current state
} Master_t;

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
uint8_t scan_state(Master_t rover);
void Timer_Init(TIM_TypeDef *timer, IRQn_Type irq_type, uint16_t psc, uint16_t arr, bool enable_interrupt);