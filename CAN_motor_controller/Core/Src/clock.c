#include <stdint.h>
#include "stm32f103xb.h"

#include "clock.h"

void set_system_clock_to_64Mhz(void)
{
  //set desired flash latency for high clock speeds and enable prefetch buffer
  FLASH->ACR &= ~(FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE);
  FLASH->ACR |= (FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE);

  //PLL loop configuration
  RCC->CFGR &= ~RCC_CFGR_PLLSRC;    //select HSI/2 as PLL source
  RCC->CFGR |= RCC_CFGR_PLLMULL16;  //set PLL multiplier to x16

  //set APB1 prescaler to 2
  RCC->CFGR &= ~RCC_CFGR_PPRE1_Msk;
  RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
  //set AHPB prescaler to 1 - not divided
  RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;

  RCC->CR |= RCC_CR_PLLON;                //enable PLL loop
  while(!(RCC->CR & RCC_CR_PLLRDY));      //wait for PLL stabilization

  RCC->CFGR |= RCC_CFGR_SW_PLL;           //set PLL as system clock source
  while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)); //wait to confirm that PLL loop is a new clock source
}
