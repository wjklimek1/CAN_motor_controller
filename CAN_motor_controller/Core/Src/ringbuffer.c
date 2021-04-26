/**
 ****************************************************************************************
 *
 * \file ringbuffer.c
 *
 * \brief Functions operating on static circular buffer (FIFO) keeping incoming CANbus messages for further processing
 *
 * Buffer keeps maximum amount of messages defined with RINGBUFFER_SIZE constant. In case of buffer overflow the oldest message is replaced by newest.
 *
 * Copyright (C) 2021 Wojciech Klimek
 * MIT license:
 * https://github.com/wjklimek1/CAN_motor_controller
 *
 ****************************************************************************************
 */

#include <stdint.h>

#include "canbus.h"
#include "ringbuffer.h"

//====================== initialization ========================//
/**
 *  @brief Initializes ringbuffer with empty values
 *
 *  @param[in] rx_buffer pointer to volatile structure of ringbuffer
 */

void ringbuffer_init(volatile CANbus_RX_buffer_t *rx_buffer)
{
  rx_buffer->head = 0;
  rx_buffer->tail = 0;
  rx_buffer->elements = 0;
}

//====================== elements pending ========================//
/**
 *  @brief Returns number of messages currently available to get from buffer.
 *
 *  In case of buffer overflow this variable value is equal to RINGBUFFER_SIZE - it just indicates amount of valid messages in array
 *
 *  @param[in] rx_buffer pointer to volatile structure of ringbuffer
 *
 *  @return amount of messages waiting in ringbuffer
 */

uint8_t ringbuffer_elements_pending(volatile CANbus_RX_buffer_t *rx_buffer)
{
  return rx_buffer->elements;
}

//====================== put message to buffer ========================//
/**
 *  @brief Places new message in the buffer.
 *
 *  In case of overflow oldest message is replaced by current one.
 *
 *  @param[in] msg CANbus message to place in buffer
 *  @param[in] rx_buffer pointer to volatile structure of ringbuffer
 *
 *  @retval 0: overflow occured, oldest message was overwritten by current one
 *  @retval 1: message saved in buffer
 */

uint8_t ringbuffer_put_msg(CANbus_msg_t msg, volatile CANbus_RX_buffer_t *rx_buffer)
{
  uint8_t head_tmp = rx_buffer->head + 1;

  if (head_tmp == RINGBUFFER_SIZE)
    head_tmp = 0;

  rx_buffer->buffer[head_tmp] = msg;
  rx_buffer->head = head_tmp;
  rx_buffer->elements++;

  if (rx_buffer->elements == RINGBUFFER_SIZE+1)
  {
    rx_buffer->elements = RINGBUFFER_SIZE;
    return 0;
  }

  return 1;
}

//====================== get message from buffer ========================//
/**
 *  @brief Gets oldest message from buffer.
 *
 *  This function should always be followed by checking if there are valid messages in buffer!
 *
 *  @param[in] msg CANbus message to place in buffer
 *  @param[in] rx_buffer pointer to volatile structure of ringbuffer
 *
 *  @return oldest message from buffer
 */

CANbus_msg_t ringbuffer_get_msg(volatile CANbus_RX_buffer_t *rx_buffer)
{
  rx_buffer->elements--;
  rx_buffer->tail++;
  if (rx_buffer->tail > RINGBUFFER_SIZE - 1)
    rx_buffer->tail = 0;
  return rx_buffer->buffer[rx_buffer->tail];
}
