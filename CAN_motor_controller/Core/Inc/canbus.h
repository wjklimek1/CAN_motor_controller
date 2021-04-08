#ifndef INC_CANBUS_H_
#define INC_CANBUS_H_

struct CANbus_tx_msg_t
{
  uint32_t stdID;
  uint32_t DLC;
  uint32_t RTR;
  uint8_t data[8];
};

uint8_t CAN1_init(uint32_t baudrate);
uint8_t CAN1_transmit_frame(struct CANbus_tx_msg_t msg);

#endif /* INC_CANBUS_H_ */
