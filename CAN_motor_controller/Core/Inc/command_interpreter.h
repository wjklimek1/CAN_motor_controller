#ifndef COMMAND_INTERPRETER_H_
#define COMMAND_INTERPRETER_H_

#define SET_SPEED             0x01
#define SET_MAX_TEMP_INTERNAL 0x02
#define SET_MAX_TEMP_EXTERNAL 0x03
#define SET_MAX_CURR          0x04
#define EMERGENCY_STOP        0x05
#define SET_INERTIA           0x06
#define SET_HEARTBEAT         0x07

#define GET_TEMP_INTERNAL     0x01
#define GET_TEMP_EXTERNAL     0x02
#define GET_VOLTAGE           0x03
#define GET_CURRENT           0x04
#define GET_SPEED             0x05
#define GET_TARGET_SPEED      0x06
#define GET_ERRORS            0x07

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
void transmit_errors();

uint8_t get_temperature_internal();
uint8_t get_temperature_external();
uint16_t get_voltage();
uint8_t get_current();
uint8_t get_mosfet_fault();

void set_speed(CANbus_msg_t msg);
void set_max_curr(CANbus_msg_t msg);
void set_max_temp_int(CANbus_msg_t msg);
void set_max_temp_ext(CANbus_msg_t msg);
void set_motor_inertia(CANbus_msg_t msg);
void set_heartbeat(CANbus_msg_t msg);

void transmit_heartbeat();
void emergency_stop();
void follow_target_speed();

#endif
