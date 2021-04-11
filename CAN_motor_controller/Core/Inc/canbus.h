#ifndef INC_CANBUS_H_
#define INC_CANBUS_H_

struct CANbus_msg_t
{
  uint32_t stdID;
  uint32_t DLC;
  uint32_t RTR;
  uint8_t data[8];
};

uint8_t CAN1_init(uint32_t baudrate);
uint8_t CAN1_transmit_message(struct CANbus_msg_t msg);
uint8_t CAN1_get_message(struct CANbus_msg_t *msg);
uint8_t CAN1_messages_pending_FIFO0();
uint8_t CAN1_messages_pending_FIFO1();
uint8_t CAN1_messages_pending();


#endif /* INC_CANBUS_H_ */
