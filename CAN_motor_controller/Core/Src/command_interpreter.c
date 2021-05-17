#include <stdint.h>
#include "stm32f103xb.h"

#include "timers.h"
#include "clock.h"
#include "adc.h"
#include "uart.h"
#include "printf.h"
#include "canbus.h"
#include "thermistor.h"
#include "ringbuffer.h"
#include "command_interpreter.h"

extern volatile uint16_t ADC_raw_values[6];
extern volatile CANbus_RX_buffer_t rx_buffer;

uint8_t _heartbeat_interval = 1;
uint8_t _motor_inertia = 10;
uint8_t _emergency_stop = 0;
uint8_t _motor_dir = 0;
uint16_t _motor_speed = 0;
uint8_t _target_motor_dir = 0;
uint16_t _target_motor_speed = 0;

uint8_t _max_current = 30;
uint8_t _max_temperature_int = 90;
uint8_t _max_temperature_ext = 90;

int8_t command_interpreter(volatile CANbus_RX_buffer_t *rx_buffer)
{
  if (ringbuffer_elements_pending(rx_buffer) > 0)
  {
    CANbus_msg_t msg = ringbuffer_get_msg(rx_buffer);

    if (msg.stdID == COMMAND_MSG_ID)
    {
      switch (msg.data[0])
	{
	case SET_SPEED:
	  set_speed(msg);
	  break;
	case SET_MAX_CURR:
	  set_max_curr(msg);
	  break;
	case SET_MAX_TEMP_INTERNAL:
	  set_max_temp_int(msg);
	  break;
	case SET_MAX_TEMP_EXTERNAL:
	  set_max_temp_ext(msg);
	  break;
	case EMERGENCY_STOP:
	  emergency_stop();
	  break;
	case SET_INERTIA:
	  set_motor_inertia(msg);
	  break;
	case SET_HEARTBEAT:
	  set_heartbeat(msg);
	  break;
	default:
	  return UNKNOWN_COMMAND;
	  break;
	}
    }
    else if (msg.stdID == DATARQ_MSG_ID)
    {
      switch (msg.data[0])
	{
	case GET_TEMP_INTERNAL:
	  transmit_temperature_internal();
	  break;
	case GET_TEMP_EXTERNAL:
	  transmit_temperature_external();
	  break;
	case GET_VOLTAGE:
	  transmit_voltage();
	  break;
	case GET_CURRENT:
	  transmit_current();
	  break;
	case GET_SPEED:
	  transmit_motor_speed();
	  break;
	case GET_TARGET_SPEED:
	  transmit_target_motor_speed();
	  break;
	case GET_ERRORS:
	  transmit_errors();
	  break;
	default:
	  return UNKNOWN_DATARQ;
	  break;
	}
    }
    else
    {
      return UNKNOWN_MSG_ID;
    }
  }
  else
    return BUFFER_EMPTY;

  return MESSAGE_PROCESSED;
}

void transmit_temperature_internal()
{
  CANbus_msg_t msg;
  msg.stdID = 0x445;
  msg.RTR = 0;
  msg.DLC = 2;

  msg.data[0] = GET_TEMP_INTERNAL;
  msg.data[1] = get_temperature_internal();

  while(CAN1_transmit_message(msg) < 0);
}

void transmit_temperature_external()
{
  CANbus_msg_t msg;
  msg.stdID = 0x445;
  msg.RTR = 0;
  msg.DLC = 2;

  msg.data[0] = GET_TEMP_EXTERNAL;
  msg.data[1] = get_temperature_external();

  while(CAN1_transmit_message(msg) < 0);
}

void transmit_voltage()
{
  CANbus_msg_t msg;
  msg.stdID = 0x445;
  msg.RTR = 0;
  msg.DLC = 3;

  uint16_t voltage = get_voltage();
  msg.data[0] = GET_VOLTAGE;
  msg.data[1] = voltage >> 8;     //MSB of voltage variable
  msg.data[2] = voltage & 0x00FF; //LSB of voltage variable

  while(CAN1_transmit_message(msg) < 0);
}

void transmit_current()
{
  CANbus_msg_t msg;
  msg.stdID = 0x445;
  msg.RTR = 0;
  msg.DLC = 3;

  uint16_t current = get_current();
  msg.data[0] = GET_CURRENT;
  msg.data[1] = current >> 8;     //MSB of current variable
  msg.data[2] = current & 0x00FF; //LSB of current variable

  while(CAN1_transmit_message(msg) < 0);
}

void transmit_motor_speed()
{
  CANbus_msg_t msg;
  msg.stdID = 0x445;
  msg.RTR = 0;
  msg.DLC = 4;

  msg.data[0] = GET_SPEED;
  msg.data[1] = _motor_dir;
  msg.data[2] = _motor_speed >> 8;     //MSB of motor speed variable
  msg.data[3] = _motor_speed & 0x00FF; //LSB of motor speed variable

  while(CAN1_transmit_message(msg) < 0);
}

void transmit_target_motor_speed()
{
  CANbus_msg_t msg;
  msg.stdID = 0x445;
  msg.RTR = 0;
  msg.DLC = 4;

  msg.data[0] = GET_TARGET_SPEED;
  msg.data[1] = _target_motor_dir;
  msg.data[2] = _target_motor_speed >> 8;     //MSB of targetmotor speed variable
  msg.data[3] = _target_motor_speed & 0x00FF; //LSB of motor speed variable

  while(CAN1_transmit_message(msg) < 0);
}

void transmit_errors()
{
  CANbus_msg_t msg;
  msg.stdID = DATARQ_MSG_ID;
  msg.RTR = 0;
  msg.DLC = 5;

  msg.data[0] = GET_ERRORS;
  if (get_mosfet_fault() == 1)
    msg.data[1] = 1;
  else
    msg.data[1] = 0;

  if (get_current() > _max_current)
    msg.data[2] = 1;
  else
    msg.data[2] = 0;

  if (get_temperature_internal() > _max_temperature_ext)
    msg.data[3] = 1;
  else
    msg.data[3] = 0;

  if (get_temperature_external() > _max_temperature_ext)
    msg.data[4] = 1;
  else
    msg.data[4] = 0;

  if (_emergency_stop == 1)
    msg.data[4] = 1;
  else
    msg.data[4] = 0;

  while(CAN1_transmit_message(msg) < 0);
}

uint8_t get_temperature_internal()
{
  uint32_t millivolts = ADC_raw_values[1] * 3300 / 4095;
  uint32_t resistance = (millivolts * 10000) / (3301 - millivolts);
  float temperature = thermistor_getTemperature_steinhart(resistance, 10000, 3455, 25);
  return (uint8_t) temperature;
}

uint8_t get_temperature_external()
{
  uint32_t millivolts = ADC_raw_values[2] * 3300 / 4095;
  uint32_t resistance = (millivolts * 10000) / (3301 - millivolts);
  float temperature = thermistor_getTemperature_steinhart(resistance, 10000, 3455, 25);
  return (uint8_t) temperature;
}

uint16_t get_voltage()
{
  //exact formula: (Vref/MaxADCval)*ADC_reading * (10/1.1) where 10/1.1 is voltage divider constant
  uint32_t millivolts = ADC_raw_values[0] * 3300 / 4095;
  millivolts = millivolts * (10000 + 1100) / 1100;
  return (uint16_t) millivolts;
}

uint8_t get_current()
{
  const float alpha = 0.1;
  float millivolts;
  float Isense;
  static float current = 0;
  static float new_current = 0;

  if (ADC_raw_values[3] >= ADC_raw_values[4])
    millivolts = ADC_raw_values[3] * 3300 / 4095;
  if (ADC_raw_values[3] < ADC_raw_values[4])
    millivolts = ADC_raw_values[4] * 3300 / 4095;

  Isense = (millivolts / 470) - 0.170f;
  new_current = Isense * 9.165f * 1000;

  current = alpha * new_current + (1 - alpha) * current;

  if (current < 0)
    return 0;

  return (uint8_t) (current / 1000);
}

uint8_t get_mosfet_fault()
{
  if (ADC_raw_values[3] > 2700)
    return 1;
  if (ADC_raw_values[4] > 2700)
    return 1;

  return 0;
}

void set_speed(CANbus_msg_t msg)
{
  _emergency_stop = 0;
  _target_motor_dir = msg.data[1];
  _target_motor_speed = msg.data[2] << 8 | msg.data[3];

  if (_target_motor_speed > 1000)
    _target_motor_speed = 1000;

  if (_target_motor_dir > 1)
    _target_motor_dir = 1;
}

void set_max_curr(CANbus_msg_t msg)
{
  _max_current = msg.data[1] << 8 | msg.data[2];
}

void set_max_temp_int(CANbus_msg_t msg)
{
  _max_temperature_int = msg.data[1];
}

void set_max_temp_ext(CANbus_msg_t msg)
{
  _max_temperature_ext = msg.data[1];
}

void set_motor_inertia(CANbus_msg_t msg)
{
  _motor_inertia = msg.data[1];
}

void set_heartbeat(CANbus_msg_t msg)
{
  _heartbeat_interval = msg.data[1];
}

void transmit_heartbeat()
{
  if (_heartbeat_interval == 0)  //if interval is 0, heartbeat is disabled
    return;

  static uint32_t timer = 0;
  if (getTick() - timer > _heartbeat_interval * 1000)  //if time defined by _heartbeat_interval passed, transmit heartbeat;
  {
    timer = getTick();
    CANbus_msg_t msg;
    msg.DLC = 0;
    msg.stdID = COMMAND_MSG_ID;
    while(CAN1_transmit_message(msg) < 0);
  }
}

void emergency_stop()
{
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;
  _emergency_stop = 1;
  _motor_speed = 0;
  _motor_dir = 0;
  _target_motor_dir = 0;
  _target_motor_speed = 0;
}

void follow_target_speed()
{
  static uint32_t timer = 0;

  if (_emergency_stop == 1)  //if device is in emergency stop mode, return
    return;

  if (getTick() - timer > _motor_inertia)  //if time defined by _motor_inertia passed, go further
  {
    timer = getTick();

    if (_target_motor_dir == 1 && _motor_dir == 1)
    {
      if (_motor_speed < _target_motor_speed)
      {
	++_motor_speed;
	TIM1->CCR1 = _motor_speed;
	TIM1->CCR2 = 0;
      }
      if (_motor_speed > _target_motor_speed)
      {
	--_motor_speed;
	TIM1->CCR1 = _motor_speed;
	TIM1->CCR2 = 0;
      }
    }
    if (_target_motor_dir == 1 && _motor_dir == 0)
    {
      if (_motor_speed > 0)
      {
	--_motor_speed;
	TIM1->CCR2 = _motor_speed;
	TIM1->CCR1 = 0;
      }
      if (_motor_speed == 0)
      {
	_motor_dir = 1;
      }
    }
    if (_target_motor_dir == 0 && _motor_dir == 1)
    {
      if (_motor_speed > 0)
      {
	--_motor_speed;
	TIM1->CCR1 = _motor_speed;
	TIM1->CCR2 = 0;
      }
      if (_motor_speed == 0)
      {
	_motor_dir = 0;
      }
    }
    if (_target_motor_dir == 0 && _motor_dir == 0)
    {
      if (_motor_speed < _target_motor_speed)
      {
	++_motor_speed;
	TIM1->CCR2 = _motor_speed;
	TIM1->CCR1 = 0;
      }
      if (_motor_speed > _target_motor_speed)
      {
	--_motor_speed;
	TIM1->CCR2 = _motor_speed;
	TIM1->CCR1 = 0;
      }
    }
  }
}
