#include <stdint.h>
#include "stm32f103xb.h"

volatile uint32_t ms_counter = 0;

void set_system_clock_to_64Mhz(void)
{
	//set desired flash latency for high clock speeds and enable prefetch buffer
	FLASH->ACR &= ~(FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE);
	FLASH->ACR |=  (FLASH_ACR_LATENCY_1 | FLASH_ACR_PRFTBE);

	//PLL loop configuration
	//select HSI/2 as PLL source
	RCC->CFGR &= ~RCC_CFGR_PLLSRC;
	//set PLL multiplier to x16
	RCC->CFGR |= RCC_CFGR_PLLMULL16;

	//set APB1 prescaler to 2
	RCC->CFGR &= ~RCC_CFGR_PPRE1_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;
	//set AHPB prescaler to 1 - not divided
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;

	//enable PLL loop
	RCC->CR |= RCC_CR_PLLON;
	//wait for PLL stabilization
	while(!(RCC->CR & RCC_CR_PLLRDY)){}

	//set PLL as system clock source
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	//wait to confirm that PLL loop is a new clock source
	while(!(RCC->CFGR & RCC_CFGR_SWS_PLL)){}
}

void SysTick_init(uint32_t ticks)
{
	//set SysTick overflow value to system clock -1
	SysTick->LOAD = (uint32_t) (ticks - 1);
	//set systick interrupt priority
	NVIC_SetPriority(SysTick_IRQn, (1UL << __NVIC_PRIO_BITS) - 1UL);
	//rest current counter value
	SysTick->VAL = 0UL;
	//select main system clock as source, enable exeption when counted to 0, start SysTick timer
	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk |	SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void)
{
	if (ms_counter > 0)
		ms_counter--;
}

void delay_ms(uint32_t ms)
{
	ms_counter = ms;
	while (ms_counter){};
}

void TIM1_init_and_start(void)
{
	//init PWM GPIOs
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; //enable alternate function
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; //enable GPIOA clock
	GPIOA->CRH &= ~(GPIO_CRH_MODE8_Msk | GPIO_CRH_MODE9_Msk); //clear mode bits
	GPIOA->CRH &= ~(GPIO_CRH_CNF8_Msk | GPIO_CRH_CNF9_Msk);   //clear cnf bits
	GPIOA->CRH |=  (GPIO_CRH_MODE8 | GPIO_CRH_MODE9); //set mode to 50MHz output
	GPIOA->CRH |=  (GPIO_CRH_CNF8_1 | GPIO_CRH_CNF9_1); //set cnf to AF push-pull

	//timer configuration
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; //enable timer clock
	TIM1->CR1 = 0UL; //reset all values in timer register

	TIM1->PSC = (3-1);  //setup prescaler
	TIM1->ARR = (1000 - 1);  //setup counter overflow value

	TIM1->CCMR1 |=  (0b110UL << TIM_CCMR1_OC1M_Pos) | (0b110UL << TIM_CCMR1_OC2M_Pos); //set outputs as PWM mode 1
	TIM1->CCMR1 |=  TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE; //enable preload register
	TIM1->CR1 |= TIM_CR1_ARPE | TIM_CR1_ARPE;
	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E; //enable channel 1 output
	TIM1->BDTR |= TIM_BDTR_MOE; //all outputs enable
	TIM1->BDTR |= TIM_BDTR_BKP; //break polarity high

	TIM1->EGR |= TIM_EGR_UG;  //send an update event to reset the timer and apply settings.
	TIM1->CR1 |= TIM_CR1_CEN; //enable timer

}

int main()
{
	RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;  //enable gpio port C clock

	GPIOC->CRH &= ~(GPIO_CRH_MODE13_Msk);
	GPIOC->CRH |=  (GPIO_CRH_MODE13_1);
	GPIOC->CRH &= ~(GPIO_CRH_CNF13_Msk);
	GPIOC->ODR |= GPIO_ODR_ODR13;

	set_system_clock_to_64Mhz();
	SysTick_init(64000);

	TIM1_init_and_start();
	TIM1->CCR1 = 500;
	TIM1->CCR2 = 300;


	int x = 0;
	while(1)
	{
		x++;

		GPIOC->ODR |= GPIO_ODR_ODR13;
		delay_ms(100);
		GPIOC->ODR &= ~GPIO_ODR_ODR13;
		delay_ms(100);
	}
}
