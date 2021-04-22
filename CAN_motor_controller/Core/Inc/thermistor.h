#ifndef THERMISTOR_H
#define THERMISTOR_H

float thermistor_getTemperature_steinhart(uint32_t resistance, uint32_t nominal_resistance, uint32_t beta, uint32_t ref_temp);

#endif /* THERMISTOR_H */
