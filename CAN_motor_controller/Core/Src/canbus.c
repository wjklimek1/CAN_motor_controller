#include <stdint.h>
#include "stm32f103xb.h"

#include "canbus.h"

uint8_t CAN1_init(uint32_t baudrate)
{
  uint32_t canbus_presc;              //canbus baudrate prescaler
  uint32_t canbus_bit_segment_1 = 7;  //canbus bit sampling 1
  uint32_t canbus_bit_segment_2 = 8;  //canbus bit sampling 2

  //select prescaler for different baudrates
  switch (baudrate)
    {
    case 250000:
      canbus_presc = 8;
      break;
    case 500000:
      canbus_presc = 4;
      break;
    case 1000000:
      canbus_presc = 2;
      break;
    default:
      return 0;
    }

  //init GPIOs for CAN - PB8 as CAN_RX, PB9 as CAN_TX
  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;                                                                     //enable GPIOB clock
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;                                                                     //enable alternate function clock
  GPIOB->CRH &= ~(GPIO_CRH_MODE9_Msk | GPIO_CRH_CNF9_Msk | GPIO_CRH_MODE8_Msk | GPIO_CRH_CNF8_Msk);       //reset MODE and CNF bits
  GPIOB->CRH |= (0b11 << GPIO_CRH_MODE9_Pos | 0b10 << GPIO_CRH_CNF9_Pos);                                 //set PB9 TX pin as AF push-pull output
  GPIOB->CRH |= (0b00 << GPIO_CRH_MODE8_Pos | 0b01 << GPIO_CRH_CNF8_Pos);                                 //set PB8 RX pin as floating input

  //CAN1 peripherial init
  RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;                            //enable peripherial clock
  AFIO->MAPR |= (0b10 << AFIO_MAPR_CAN_REMAP_Pos);               //remap CAN1 peripherial to PB9(TX) and PB8(RX)
  CAN1->MCR &= ~CAN_MCR_SLEEP;                                   //exit sleep mode
  CAN1->MCR |= CAN_MCR_INRQ;                                     //enter into initialization mode
  while(!(CAN1->MCR & CAN_MSR_INAK));                            //wait for the hardware to confirm entering the initialization mode

  CAN1->BTR &= ~CAN_BTR_SILM;                                    //go into normal mode to both transmit and receive messages
  CAN1->BTR &= ~CAN_BTR_LBKM;                                    //disable loopback mode
  CAN1->BTR &= ~CAN_BTR_SJW_Msk;                                 //set max sync jump width to default 1 time quant
  CAN1->BTR &= ~(CAN_BTR_TS2_Msk | CAN_BTR_TS1_Msk);             //clear time segments bits
  CAN1->BTR |= ((canbus_bit_segment_2 - 1) << CAN_BTR_TS2_Pos);  //set time segment 2
  CAN1->BTR |= ((canbus_bit_segment_1 - 1) << CAN_BTR_TS1_Pos);  //set time segment 1
  CAN1->BTR |= ((canbus_presc - 1) << CAN_BTR_BRP_Pos);          //set canbus prescaler
  CAN1->MCR |= CAN_MCR_NART;                                     //disable automatic retransmission

  /*
   * TODO: init CAN filters here
   */
  //disable all filters - for testing only
  CAN1->FA1R |= (0b1111111111111);
  CAN1->FMR &= ~(CAN_FMR_FINIT);
  /*
   * init CAN filters end
   */

  CAN1->MCR &= ~CAN_MCR_INRQ;  //exit initialization mode
  while((CAN1->MSR & CAN_MSR_INAK));  //wait for the hardware to confirm entering normal mode

  return 1;
}

uint8_t CAN1_transmit_message(struct CANbus_msg_t msg)
{
  if (CAN1->TSR & CAN_TSR_TME0) //check if mailbox0 is empty
  {
    CAN1->sTxMailBox[0].TIR = 0; //reset all values remaining from last transmission
    CAN1->sTxMailBox[0].TIR |= (msg.stdID << CAN_TI0R_STID_Pos) & CAN_TI0R_STID_Msk; //set message ID
    CAN1->sTxMailBox[0].TIR &= ~(CAN_TI0R_IDE); //set ID as standard 11bit ID
    CAN1->sTxMailBox[0].TIR |= (msg.RTR << CAN_TI0R_RTR_Pos) & CAN_TI0R_RTR_Msk; //set message as RTR or normal

    CAN1->sTxMailBox[0].TDTR = 0; //reset all values remaining from last transmission
    CAN1->sTxMailBox[0].TDTR |= (msg.DLC << CAN_TDT0R_DLC_Pos) & CAN_TDT0R_DLC_Msk;

    //fill data registers with data from message
    CAN1->sTxMailBox[0].TDLR = 0;
    CAN1->sTxMailBox[0].TDLR = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];
    CAN1->sTxMailBox[0].TDHR = 0;
    CAN1->sTxMailBox[0].TDHR = (msg.data[7] << 24) | (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4];

    //transmit message
    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ;

    /*
     * TODO: add error checking after transmission
     */

    return 1;

  }
  else return 0;
}

uint8_t CAN1_get_message(struct CANbus_msg_t *msg)
{
  //check for messages in mailbox FIFO0
  if (CAN1->RF0R & CAN_RF0R_FMP0_Msk)
  {
    msg->stdID = ((CAN1->sFIFOMailBox[0].RIR & CAN_RI0R_STID_Msk) >> CAN_RI0R_STID_Pos);   //get message ID
    msg->RTR = ((CAN1->sFIFOMailBox[0].RIR & CAN_RI0R_RTR_Msk) >> CAN_RI0R_RTR_Pos);       //check if message is RTR type
    msg->DLC = ((CAN1->sFIFOMailBox[0].RDTR & CAN_RDT0R_DLC_Msk) >> CAN_RDT0R_DLC_Pos);    //get amount of bytes in message

    //get received data from registers
    msg->data[0] = (CAN1->sFIFOMailBox[0].RDLR & 0xFF);
    msg->data[1] = (CAN1->sFIFOMailBox[0].RDLR & 0xFF00) >> 8;
    msg->data[2] = (CAN1->sFIFOMailBox[0].RDLR & 0xFF0000) >> 16;
    msg->data[3] = (CAN1->sFIFOMailBox[0].RDLR & 0xFF000000) >> 24;
    msg->data[4] = (CAN1->sFIFOMailBox[0].RDHR & 0xFF);
    msg->data[5] = (CAN1->sFIFOMailBox[0].RDHR & 0xFF00) >> 8;
    msg->data[6] = (CAN1->sFIFOMailBox[0].RDHR & 0xFF0000) >> 16;
    msg->data[7] = (CAN1->sFIFOMailBox[0].RDHR & 0xFF000000) >> 24;

    CAN1->RF0R |= CAN_RF0R_RFOM0;  //release mailbox 0

    return 1;
  }
  //check for messages in mailbox FIFO1
  else if (CAN1->RF1R & CAN_RF1R_FMP1_Msk)
  {
    msg->stdID = ((CAN1->sFIFOMailBox[1].RIR & CAN_RI1R_STID_Msk) >> CAN_RI1R_STID_Pos);   //get message ID
    msg->RTR = ((CAN1->sFIFOMailBox[1].RIR & CAN_RI1R_RTR_Msk) >> CAN_RI1R_RTR_Pos);       //check if message is RTR type
    msg->DLC = ((CAN1->sFIFOMailBox[1].RDTR & CAN_RDT1R_DLC_Msk) >> CAN_RDT1R_DLC_Pos);    //get amount of bytes in message

    //get received data from registers
    msg->data[0] = (CAN1->sFIFOMailBox[1].RDLR & 0xFF);
    msg->data[1] = (CAN1->sFIFOMailBox[1].RDLR & 0xFF00) >> 8;
    msg->data[2] = (CAN1->sFIFOMailBox[1].RDLR & 0xFF0000) >> 16;
    msg->data[3] = (CAN1->sFIFOMailBox[1].RDLR & 0xFF000000) >> 24;
    msg->data[4] = (CAN1->sFIFOMailBox[1].RDHR & 0xFF);
    msg->data[5] = (CAN1->sFIFOMailBox[1].RDHR & 0xFF00) >> 8;
    msg->data[6] = (CAN1->sFIFOMailBox[1].RDHR & 0xFF0000) >> 16;
    msg->data[7] = (CAN1->sFIFOMailBox[1].RDHR & 0xFF000000) >> 24;

    CAN1->RF0R |= CAN_RF1R_RFOM1;  //release mailbox 1

    return 2;
  }
  else return 0;
}

uint8_t CAN1_messages_pending_FIFO0()
{
  return (CAN1->RF0R & CAN_RF0R_FMP0_Msk) >> CAN_RF0R_FMP0_Pos;
}

uint8_t CAN1_messages_pending_FIFO1()
{
  return (CAN1->RF1R & CAN_RF1R_FMP1_Msk) >> CAN_RF1R_FMP1_Pos;
}

uint8_t CAN1_messages_pending()
{
  uint8_t FIFO0_pending = (CAN1->RF0R & CAN_RF0R_FMP0_Msk) >> CAN_RF0R_FMP0_Pos;
  uint8_t FIFO1_pending = (CAN1->RF1R & CAN_RF1R_FMP1_Msk) >> CAN_RF1R_FMP1_Pos;
  return FIFO0_pending + FIFO1_pending;
}

