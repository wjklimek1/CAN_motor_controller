#include <stdint.h>
#include <math.h>
#include "thermistor.h"

//====================== read tempeatature ========================//
/**
 *  @brief Converts thermistor resistance to temperature.
 *
 *  Using Steinhart equation, this function converts resistance of thermistor to temperature in Celcius.
 *
 *  @param[in] resistance current resistance of thermistor in Ohms
 *  @param[in] nominal_resistance nominal resistance of thermistor in Ohms (get from datasheet)
 *  @param[in] beta beta value of thermistor (get from datasheet)
 *  @param[in] ref_temp temperatue, when thermistor has nominal resistance. Almost always = 25 degree C, but check in datasheet to be sure.
 *
 *  @return value of temperature in Celcius degree
 */

float thermistor_getTemperature_steinhart(uint32_t resistance, uint32_t nominal_resistance, uint32_t beta, uint32_t ref_temp)
{
  float steinhart;
  steinhart = (float)resistance / (float)nominal_resistance;      // (R/Ro)
  steinhart = log(steinhart);                       // ln(R/Ro)
  steinhart /= beta;                                // 1/B * ln(R/Ro)
  steinhart += 1.0 / (ref_temp + 273.15);           // + (1/To)
  steinhart = 1.0 / steinhart;                      // Invert
  steinhart -= 273.15;                              // convert to C
  return steinhart;
}
