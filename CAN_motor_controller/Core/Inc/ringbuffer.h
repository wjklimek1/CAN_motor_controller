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
