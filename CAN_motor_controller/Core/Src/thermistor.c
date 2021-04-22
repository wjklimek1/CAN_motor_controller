#include <stdint.h>
#include <math.h>
#include "thermistor.h"

float thermistor_getTemperature_steinhart(uint32_t resistance, uint32_t nominal_resistance, uint32_t beta, uint32_t ref_temp)
{
  float steinhart;
  steinhart = resistance / nominal_resistance;      // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= beta;                                // 1/B * ln(R/Ro)
  steinhart += 1.0 / (ref_temp + 273.15);           // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to C
  return steinhart;
}
