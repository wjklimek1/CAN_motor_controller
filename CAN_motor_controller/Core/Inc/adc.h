#ifndef INC_ADC_H_
#define INC_ADC_H_

void ADC_init(void);
void DMA_init(void);
void ADC_start_with_DMA(uint16_t *results_array);

#endif /* INC_ADC_H_ */
