#include <stdint.h>
#include "stm32f103xb.h"

#include "timers.h"
#include "clock.h"
#include "adc.h"
#include "uart.h"
#include "printf.h"
#include "canbus.h"
#include "ringbuffer.h"
#include "thermistor.h"
#include "command_interpreter.h"

volatile uint16_t ADC_raw_values[6];
volatile CANbus_RX_buffer_t rx_buffer;

int main()
{
  set_system_clock_to_64Mhz();

  SysTick_init(64000);
  TIM1_init_and_start();
  ADC_init();
  ADC_start_with_DMA((uint16_t*)ADC_raw_values);
  UART3_init(64000000, 115200);

  CAN1_init(250000);
  CAN1_config_filter(0, COMMAND_MSG_ID, 0);
  CAN1_config_filter(1, DATARQ_MSG_ID, 1);

  ringbuffer_init(&rx_buffer);

  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;  //enable gpio port C clock
  GPIOC->CRH &= ~(GPIO_CRH_MODE13_Msk);
  GPIOC->CRH |= (GPIO_CRH_MODE13_1);
  GPIOC->CRH &= ~(GPIO_CRH_CNF13_Msk); //turn on LED

  while(1)
  {
    command_interpreter(&rx_buffer);
    follow_target_speed();
    transmit_heartbeat();
  }
}
