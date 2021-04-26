#ifndef INC_CANBUS_H_
#define INC_CANBUS_H_

#define COMMAND_MSG_ID 0x444
#define DATARQ_MSG_ID  0x445

typedef struct CANbus_msg_t
{
  uint32_t stdID;
  uint32_t DLC;
  uint32_t RTR;
  uint8_t data[8];
} CANbus_msg_t;

uint8_t CAN1_init(uint32_t baudrate);
int8_t CAN1_transmit_message(CANbus_msg_t msg);
uint8_t CAN1_get_message(CANbus_msg_t *msg);
uint8_t CAN1_messages_pending_FIFO0();
uint8_t CAN1_messages_pending_FIFO1();
uint8_t CAN1_messages_pending();

//CAN1 interrupts
void USB_LP_CAN1_RX0_IRQHandler();
void CAN1_RX1_IRQHandler();

#endif /* INC_CANBUS_H_ */
