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

  ringbuffer_init(&rx_buffer);

  while(1)
  {
    int x = command_interpreter(&rx_buffer);

    if(x != 0)
    {
      delay_ms(1);
    }

    delay_ms(10);

    CANbus_msg_t msg;
    msg.DLC = 5;
    msg.stdID = 0x400;
    x = CAN1_transmit_message(msg);

    if(x != 2)
    {
      delay_ms(1);
    }

  }
}
