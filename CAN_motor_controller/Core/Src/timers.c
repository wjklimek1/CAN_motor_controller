#include <stdint.h>
#include "stm32f103xb.h"

#include "timers.h"

volatile uint32_t ms_counter = 0;

void SysTick_init(uint32_t ticks)
{
  //set SysTick overflow value to system clock -1
  SysTick->LOAD = (uint32_t) (ticks - 1);
  //set SysTick interrupt priority
  NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
  //rest current counter value
  SysTick->VAL = 0UL;
  //select main system clock as source, enable exception when counted to 0, start SysTick timer
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
{
  if (ms_counter > 0)
    ms_counter--;
}

void delay_ms(uint32_t ms)
{
  ms_counter = ms;
  while(ms_counter);
}

void TIM1_init_and_start(void)
{
  //init PWM GPIOs - PA8 and PA9
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                         //enable alternate function clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                         //enable GPIOA clock
  GPIOA->CRH &= ~(GPIO_CRH_MODE8_Msk | GPIO_CRH_MODE9_Msk);   //clear mode bits
  GPIOA->CRH &= ~(GPIO_CRH_CNF8_Msk | GPIO_CRH_CNF9_Msk);     //clear cnf bits
  GPIOA->CRH |= (GPIO_CRH_MODE8 | GPIO_CRH_MODE9);            //set mode to 50MHz output
  GPIOA->CRH |= (GPIO_CRH_CNF8_1 | GPIO_CRH_CNF9_1);          //set cnf to AF push-pull

  //timer configuration
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;     //enable timer clock
  TIM1->CR1 = 0UL;                        //reset all values in timer register
  TIM1->PSC = (3 - 1);                    //setup prescaler
  TIM1->ARR = (1000 - 1);                 //setup counter overflow value

  TIM1->CCMR1 |= (0b110UL << TIM_CCMR1_OC1M_Pos) | (0b110UL << TIM_CCMR1_OC2M_Pos);  //set outputs as PWM mode 1
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;                                  //enable preload register
  TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_ARPE;                                          //enable auto reload preload
  TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E;                                       //enable channel 1 output
  TIM1->BDTR |= TIM_BDTR_MOE;                                                        //all outputs enable
  TIM1->BDTR |= TIM_BDTR_BKP;                                                        //break polarity high

  TIM1->EGR |= TIM_EGR_UG;    //send an update event to reset the timer and apply settings
  TIM1->CR1 |= TIM_CR1_CEN;   //enable timer
}

void TIM4_init_and_start(void)
{
  //init input capture GPIO - PB7
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;   //enable alternate function clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;   //enable GPIOA clock
  GPIOB->CRL &= ~GPIO_CRL_MODE7_Msk;    //clear mode bits - set GPIO as input
  GPIOB->CRL &= ~GPIO_CRL_CNF7_Msk;     //clear cnf bits
  GPIOB->CRL |= GPIO_CRL_CNF7_1;        //set cnf to pull-up/pull-down input
  GPIOB->ODR &= ~GPIO_ODR_ODR7;         //enable pull-down resistor, setting pin low

  //timer configuration
  RCC->APB2ENR |= RCC_APB1ENR_TIM4EN;   //enable timer clock
  TIM4->CR1 = 0UL;                      //reset all values in timer register
  TIM1->PSC = (64 - 1);                 //setup prescaler
  TIM1->ARR = 65535;                    //setup counter overflow value

  //unfinished
}
