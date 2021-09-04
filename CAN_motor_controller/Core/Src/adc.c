#include <stdint.h>
#include "stm32f103xb.h"

#include "adc.h"
#include "timers.h"

void ADC_init(void)
{
  //init ADC GPIOs - PA0, PA1, PA2, PA3, PA4
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                                                                  //enable alternate function clock
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;                                                                  //enable GPIOA clock
  GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_MODE1 | GPIO_CRL_MODE2 | GPIO_CRL_MODE3 | GPIO_CRL_MODE4); //set GPIOs to analog mode
  GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_CNF1 | GPIO_CRL_CNF2 | GPIO_CRL_CNF4 | GPIO_CRL_CNF4);      //set GPIOs as inputs

  //configure ADC clock source
  RCC->CFGR &= ~RCC_CFGR_ADCPRE;       //clear ADC clock prescaler bits
  RCC->CFGR |= RCC_CFGR_ADCPRE_DIV8;   //set ADC prescaler to 8 to get 8HMz ADC clock
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;  //enable ADC clock
  //init ADC1
  ADC1->CR2 |= ADC_CR2_ADON;           //wake up ADC form power-down mode
  delay_ms(1);                         //wait some time between power-up and calibration
  ADC1->CR2 |= ADC_CR2_CAL;            //start ADC calibration
  while(ADC1->CR2 & ADC_CR2_CAL)
    {
    };  //wait until calibration is completed

  ADC1->CR2 |= (ADC_CR2_CONT | ADC_CR2_DMA | ADC_CR2_EXTSEL);  //enable continuous mode, enable DMA, set software as conversion trigger
  ADC1->CR1 |= ADC_CR1_SCAN;                                   //enable scan mode
  ADC1->CR2 |= ADC_CR2_TSVREFE;                                //enable internal temperature sensor
  ADC1->CR2 |= ADC_CR2_EXTTRIG;                                //enable conversion on external trigger

  ADC1->SQR1 = 0; //reset register
  ADC1->SQR1 |= ((6 - 1) << ADC_SQR1_L_Pos);  //set number of conversions to 6
  ADC1->SQR3 |= (0 << ADC_SQR3_SQ1_Pos);      //PA0 as 1st conversion
  ADC1->SQR3 |= (1 << ADC_SQR3_SQ2_Pos);      //PA1 as 2nd conversion
  ADC1->SQR3 |= (2 << ADC_SQR3_SQ3_Pos);      //PA2 as 3rd conversion
  ADC1->SQR3 |= (3 << ADC_SQR3_SQ4_Pos);      //PA3 as 4th conversion
  ADC1->SQR3 |= (4 << ADC_SQR3_SQ5_Pos);      //PA4 as 5th conversion
  ADC1->SQR3 |= (16 << ADC_SQR3_SQ6_Pos);     //TempSensor as 6th conversion

  //set sampling time of all conversions to 239.5 cycles for best stability
  ADC1->SMPR2 |= ADC_SMPR2_SMP0;
  ADC1->SMPR2 |= ADC_SMPR2_SMP1;
  ADC1->SMPR2 |= ADC_SMPR2_SMP2;
  ADC1->SMPR2 |= ADC_SMPR2_SMP3;
  ADC1->SMPR2 |= ADC_SMPR2_SMP4;
  ADC1->SMPR1 |= ADC_SMPR1_SMP16;
}

void ADC_start_with_DMA(uint16_t *results_array)
{
  // DMA initialization and start
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;               //enable DMA1 clock
  DMA1_Channel1->CCR &= ~(DMA_CCR_EN);            //disable DMA channel associated with ADC1
  DMA1_Channel1->CPAR = (uint32_t) &ADC1->DR;     //set peripherial address to read from ADC's data register
  DMA1_Channel1->CMAR = (uint32_t) results_array; //set memory address to write to
  DMA1_Channel1->CNDTR = 6;                       //set number of data transmissions
  DMA1_Channel1->CCR |= DMA_CCR_CIRC;             //enable circular mode
  DMA1_Channel1->CCR |= DMA_CCR_MSIZE_0;          //set memory data size to 16bit word
  DMA1_Channel1->CCR |= DMA_CCR_PSIZE_0;          //set peripherial data size to 16bit word
  DMA1_Channel1->CCR |= DMA_CCR_MINC;             //enable memory address incrementation
  DMA1_Channel1->CCR &= ~(DMA_CCR_PL);            //clear DMA channel priority bits to set lowest priority
  DMA1_Channel1->CCR |= DMA_CCR_EN;               //enable DMA1 channel 1

  // start ADC conversions
  ADC1->CR2 |= ADC_CR2_ADON;    //final ADC power up
  ADC1->CR2 |= ADC_CR2_SWSTART; //ADC start
}
