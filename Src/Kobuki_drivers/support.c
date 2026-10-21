#include <stdint.h>
#define STM32G474xx
#include "stm32g4xx.h"
#include "uart.h"
#include "kobuki.h"
#include <stdbool.h>

#define MAX_SAMPLES 200
uint32_t pos = 0;
uint32_t curr_min_value = ULTRASONIC_MAX_RANGE_MM;
uint32_t curr_min_index = 0;
uint16_t arc_positions[200];

void Timer_Init(TIM_TypeDef *timer, IRQn_Type irq_type, uint16_t psc, uint16_t arr, uint8_t enable_interrupt)
{
    timer->PSC = psc;
    timer->ARR = arr;
    if (enable_interrupt) {
        timer->DIER |= 1<<0; // Turn on interrupts for the Timer
        NVIC_EnableIRQ(irq_type); // Allow timer to interrupt Cortex processor
    }
    // Enable Timer/Counter
    timer->CR1 |= 1<<0;
}

void Port_Clock_Enable(void)
{
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN | RCC_IOPENR_GPIOBEN | RCC_IOPENR_GPIOCEN;
    RCC->APBENR1 |= RCC_APBENR1_TIM3EN;
    RCC->APBENR2  |= RCC_APBENR2_TIM1EN;
}

uint8_t scan_state(Master_t *rover)
{
    uint16_t d = Ultrasonic_Read();            // raw distance, no threshold gate
    rover->arc_positions[rover->index_pos] = d;
    if (d < rover->curr_min_value) {
        rover->curr_min_value = d;
        rover->curr_min_index = rover->index_pos;
    }
    rover->index_pos++;
    return (rover->index_pos >= MAX_SAMPLES);
}