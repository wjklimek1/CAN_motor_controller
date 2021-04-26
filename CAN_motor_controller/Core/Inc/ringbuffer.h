/**
 ****************************************************************************************
 *
 * \file ringbuffer.h
 *
 * \brief Header for circular buffer (FIFO) keeping incoming CANbus messages for further processing.
 *
 * Buffer keeps maximum amount of messages defined with RINGBUFFER_SIZE constant. In case of buffer overflow the oldest message is replaced by newest.
 *
 * Copyright (C) 2021 Wojciech Klimek
 * MIT license:
 * https://github.com/wjklimek1/CAN_motor_controller
 *
 ****************************************************************************************
 */

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#define RINGBUFFER_SIZE 12

typedef struct CANbus_RX_buffer_t
{
  uint8_t head;
  uint8_t tail;
  uint8_t elements;
  CANbus_msg_t buffer[RINGBUFFER_SIZE];
} CANbus_RX_buffer_t;

void ringbuffer_init(volatile CANbus_RX_buffer_t *rx_buffer);
uint8_t ringbuffer_elements_pending(volatile  CANbus_RX_buffer_t *rx_buffer);
uint8_t ringbuffer_put_msg(CANbus_msg_t msg, volatile CANbus_RX_buffer_t *rx_buffer);
CANbus_msg_t ringbuffer_get_msg(volatile CANbus_RX_buffer_t *rx_buffer);

#endif /* RINGBUFFER_H */
