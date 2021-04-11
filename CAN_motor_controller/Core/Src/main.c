#include <stdint.h>
#include "stm32f103xb.h"

#include "timers.h"
#include "clock.h"
#include "adc.h"
#include "uart.h"
#include "printf.h"
#include "canbus.h"

uint16_t ADC_raw_values[6];

int main()
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;  //enable gpio port C clock

  GPIOC->CRH &= ~(GPIO_CRH_MODE13_Msk);
  GPIOC->CRH |= (GPIO_CRH_MODE13_1);
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

  UART3_init(32000000, 115200);

  CAN1_init(250000);

  GPIOC->ODR |= GPIO_ODR_ODR13;

  struct CANbus_msg_t msg;
  msg.stdID = 0x444;
  msg.DLC = 8;
  msg.RTR = 0;
  msg.data[0] = 0;
  msg.data[1] = 1;
  msg.data[2] = 2;
  msg.data[3] = 3;
  msg.data[4] = 4;
  msg.data[5] = 5;
  msg.data[6] = 6;
  msg.data[7] = 7;

  while(1)
  {
    delay_ms(1000);
    CAN1_transmit_message(msg);

    //int c = CAN1_get_message(&msg);
    int a = CAN1_messages_pending();
    int b = CAN1_messages_pending_FIFO0();

    delay_ms(1);
  }
}
