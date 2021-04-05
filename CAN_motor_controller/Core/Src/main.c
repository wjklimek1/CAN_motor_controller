#include <stdint.h>
#include "stm32f103xb.h"

#include "timers.h"
#include "clock.h"
#include "adc.h"

uint16_t ADC_raw_values[6];

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

	ADC_init();
	DMA_init();
	ADC_start_with_DMA(ADC_raw_values);

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
