#ifndef COMMAND_INTERPRETER_H_
#define COMMAND_INTERPRETER_H_

#define SET_SPEED             0x01
#define SET_MAX_TEMP_INTERNAL 0x02
#define SET_MAX_TEMP_EXTERNAL 0x03
#define SET_MAX_CURR          0x04

#define GET_TEMP_INTERNAL     0x01
#define GET_TEMP_EXTERNAL     0x02
#define GET_VOLTAGE           0x03
#define GET_CURRENT           0x04
#define GET_SPEED             0x05
#define GET_TARGET_SPEED      0x06

#define MESSAGE_PROCESSED 1
#define BUFFER_EMPTY      0
#define UNKNOWN_MSG_ID  (-1)
#define UNKNOWN_DATARQ  (-2)
#define UNKNOWN_COMMAND (-3)

int8_t command_interpreter(volatile CANbus_RX_buffer_t *rx_buffer);

void transmit_temperature_internal();
void transmit_temperature_external();
void transmit_voltage();
void transmit_current();
void transmit_motor_speed();
void transmit_target_motor_speed();

uint8_t get_temperature_internal();
uint8_t get_temperature_external();
uint16_t get_voltage();
uint16_t get_current();

void motor_set_target_speed(uint8_t dir, uint8_t target_speed);

#endif
