/*******************************************************
 * timers.h - functions for timers configuration and usage,
 * including SysTick and PWM outputs.
 *******************************************************/

#ifndef INC_TIMERS_H_
#define INC_TIMERS_H_

void SysTick_init(uint32_t ticks);
void SysTick_Handler(void);
void delay_ms(uint32_t ms);
void TIM1_init_and_start(void);
void TIM4_init_and_start(void);

#endif /* INC_TIMERS_H_ */
