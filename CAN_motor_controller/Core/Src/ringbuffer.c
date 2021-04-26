#include <stdint.h>

#include "canbus.h"
#include "ringbuffer.h"

void ringbuffer_init(volatile CANbus_RX_buffer_t *rx_buffer)
{
  rx_buffer->head = 0;
  rx_buffer->tail = 0;
  rx_buffer->elements = 0;

}

uint8_t ringbuffer_elements_pending(volatile CANbus_RX_buffer_t *rx_buffer)
{
  return rx_buffer->elements;
}

uint8_t ringbuffer_put_msg(CANbus_msg_t msg, volatile CANbus_RX_buffer_t *rx_buffer)
{
  uint8_t head_tmp = rx_buffer->head + 1;

  if (head_tmp == RINGBUFFER_SIZE)
    head_tmp = 0;

  rx_buffer->buffer[head_tmp] = msg;
  rx_buffer->head = head_tmp;
  rx_buffer->elements++;

  if (rx_buffer->elements == RINGBUFFER_SIZE+1)
    rx_buffer->elements = RINGBUFFER_SIZE;

  return 1;
}

CANbus_msg_t ringbuffer_get_msg(volatile CANbus_RX_buffer_t *rx_buffer)
{
  rx_buffer->elements--;
  rx_buffer->tail++;
  if (rx_buffer->tail > RINGBUFFER_SIZE - 1)
    rx_buffer->tail = 0;
  return rx_buffer->buffer[rx_buffer->tail];
}
