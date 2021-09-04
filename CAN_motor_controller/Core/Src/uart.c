#include <stdint.h>
#include "stm32f103xb.h"

#include "uart.h"

//====================== initialization ========================//
/**
 *  @brief Initializes UART3 peripherial with PB10 as TX and PB11 as RX.
 *
 *  Available baudrates are: 250kb/s, 500kb/s 1Mb/s
 *
 *  @param[in] sys_core_clk value of system clock in MCU
 *  @param[in] baudrate baudrate of UART transmission
 */

void UART3_init(uint32_t sys_core_clk, uint32_t baudrate)
{
  //configure UART3 GPIOs: PB10 - TX, PB11 - RX
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;  //enable GPIOB clock
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;  //enable alternate function clock

  GPIOB->CRH &= ~(GPIO_CRH_MODE10_Msk | GPIO_CRH_CNF10_Msk | GPIO_CRH_MODE11_Msk | GPIO_CRH_CNF11_Msk);   //reset MODE and CNF bits
  GPIOB->CRH |= (0b11 << GPIO_CRH_MODE10_Pos | 0b10 << GPIO_CRH_CNF10_Pos);                               //set PB10 TX pin as AF push-pull output
  GPIOB->CRH |= (0b00 << GPIO_CRH_MODE11_Pos | 0b01 << GPIO_CRH_CNF11_Pos);                               //set PB11 RX pin as floating input

  //configure USART3 peripherial
  RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  //enable peripherial clock
  USART3->CR1 |= USART_CR1_UE;           //enable peripherial

  //configure baudrate
  uint32_t uartdiv = (sys_core_clk / 2) / baudrate;
  USART3->BRR = (((uartdiv / 16) << USART_BRR_DIV_Mantissa_Pos) | ((uartdiv % 16) << USART_BRR_DIV_Fraction_Pos));  //set baudrate configuration
  USART3->CR1 |= USART_CR1_TE;                                                                                      //enable transmission - send idle frame at the beggining
}

//====================== send byte ========================//
/**
 *  @brief Transmits single byte through UART3.
 *
 *  @param[in] tx_byte single byte to transmit through UART3
 */

void UART3_send_byte(uint8_t tx_byte)
{
  while(!(USART3->SR & USART_SR_TXE));  //wait until last transmission is completed
  USART3->DR = tx_byte;                 //write data to transmission register
}

//====================== send string ========================//
/**
 *  @brief Transmits string through UART3.
 *
 *  @param[in] string pointer to char array terminated with 0.
 */

void UART3_print_string(char *string)
{
  int i = 0;
  while(*(string + i) != 0)
    {
      UART3_send_byte(*(string + i));
      i++;
    }
}

//====================== _putchar ========================//
/**
 *  @brief _putchar implementation for printf() support.
 *
 *  @param[in] character single byte to transmit through UART3
 */

void _putchar(char character)
{
  while(!(USART3->SR & USART_SR_TXE));  //wait until last transmission is completed
  USART3->DR = character;               //write data to transmission register
}
