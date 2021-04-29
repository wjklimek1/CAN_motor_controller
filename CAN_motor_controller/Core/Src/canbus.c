/**
 ****************************************************************************************
 *
 * \file canbus.c
 *
 * \brief Functions operating on registers of bxCAN peripharial in STM32F103C8T6 microcontroller.
 *
 * Mesages are received in interrupts. Received message is placed in ringbuffer.
 *
 * Copyright (C) 2021 Wojciech Klimek
 * MIT license:
 * https://github.com/wjklimek1/CAN_motor_controller
 *
 ****************************************************************************************
 */

#include <stdint.h>
#include "stm32f103xb.h"

#include "canbus.h"
#include "ringbuffer.h"

extern volatile CANbus_RX_buffer_t rx_buffer;

//====================== initialization ========================//
/**
 *  @brief Initializes CAN1 with given baudrate on pins PB8 as CAN_RX and PB9 as CAN_TX.
 *
 *  Available baudrates are: 250kb/s, 500kb/s 1Mb/s
 *
 *  This function also configures CAN filters to pass only messages with ID equal to DATA_MSG_ID or DATA_RQ_ID.
 *  This constants are defined in canbus.h file. Messages with ID = DATA_MSG_ID are placed in FIFO0 and ones with ID = DATA_MSG_ID are placed in FIFO1.
 *
 *  @param[in] baudrate CAN1 baudrate in b/s (250000 for 250kb/s and so on)
 *
 *  @retval 0: baudrate not supported, CAN1 not initialized
 *  @retval 1: initialization correct
 */

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
  CAN1->MCR &= ~(CAN_MCR_NART);                                  //enable automatic retransmission
  CAN1->MCR |= CAN_MCR_ABOM;                                     //enable automatic bus-off recovery

  //Init CAN identifier filters
  CAN1->FMR   |= CAN_FMR_FINIT;                           //enter init mode
  CAN1->FS1R  |= (CAN_FS1R_FSC0 | CAN_FS1R_FSC1);         //set filters as 32bit values

  CAN1->sFilterRegister[0].FR1 = COMMAND_MSG_ID << 21;    //config filter 0 to pass COMMAND_MSG_ID
  CAN1->sFilterRegister[0].FR2 = 0x7FF << 21;             //config mask to match all bits
  CAN1->sFilterRegister[1].FR1 = DATARQ_MSG_ID << 21;     //config filter 1 to pass DATARQ_MSG_ID
  CAN1->sFilterRegister[1].FR2 = 0x7FF << 21;             //config mask to match all bits

  CAN1->FFA1R &= ~CAN_FFA1R_FFA0;                         //place messages passing through filter 0 in FIFO0
  CAN1->FFA1R |=  CAN_FFA1R_FFA1;                         //place messages passing through filter 1 in FIFO1
  CAN1->FM1R  &= ~(CAN_FM1R_FBM0 | CAN_FM1R_FBM1);        //set filter bank 0 and 1 to identifier mask mode
  CAN1->FA1R |= (CAN_FA1R_FACT0 | CAN_FA1R_FACT1);        //activate filters 0 and 1
  CAN1->FMR &= ~CAN_FMR_FINIT;                            //exit init mode

  //enable CAN1_RX0 and CAN1_RX1 interrupts
  CAN1->IER |= (CAN_IER_FMPIE0 | CAN_IER_FMPIE1);   //enable interrupts for receiving
  NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 6);        //set RX0 interrupt to 6 in NVIC
  NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);             //enable RX0 interrupt in NVIC
  NVIC_SetPriority(CAN1_RX1_IRQn, 7);               //set RX1 interrupt to 6 in NVIC
  NVIC_EnableIRQ(CAN1_RX1_IRQn);                    //enable RX1 interrupt in NVIC

  CAN1->MCR &= ~CAN_MCR_INRQ;         //exit initialization mode
  while((CAN1->MSR & CAN_MSR_INAK));  //wait for the hardware to confirm entering normal mode

  return 1;
}

//====================== transmit message ========================//
/**
 *  @brief Places CAN message in transmission mailbox.
 *
 *  After placing message in TX mailbox it will be automatically send by hardware.
 *  Return value indicates empty places in mailbox. If it is equal -1 message was not transmitted because there ware no empty mailboxes.
 *
 *  @param[in] msg CAN message to transmit
 *
 *  @retval -1: no emty mailboxes, transmission has to be repeated
 *  @retval  0: message placed, all mailboxes are true
 *  @retval  1: message placed, 1 mailbox still free
 *  @retval  2: message placed, 2 mailboxes still free
 */

int8_t CAN1_transmit_message(CANbus_msg_t msg)
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
    return 2;
  }
  else if (CAN1->TSR & CAN_TSR_TME1) //check if mailbox1 is empty
  {
    CAN1->sTxMailBox[1].TIR = 0; //reset all values remaining from last transmission
    CAN1->sTxMailBox[1].TIR |= (msg.stdID << CAN_TI1R_STID_Pos) & CAN_TI1R_STID_Msk; //set message ID
    CAN1->sTxMailBox[1].TIR &= ~(CAN_TI1R_IDE); //set ID as standard 11bit ID
    CAN1->sTxMailBox[1].TIR |= (msg.RTR << CAN_TI1R_RTR_Pos) & CAN_TI1R_RTR_Msk; //set message as RTR or normal

    CAN1->sTxMailBox[1].TDTR = 0; //reset all values remaining from last transmission
    CAN1->sTxMailBox[1].TDTR |= (msg.DLC << CAN_TDT1R_DLC_Pos) & CAN_TDT1R_DLC_Msk;

    //fill data registers with data from message
    CAN1->sTxMailBox[1].TDLR = 0;
    CAN1->sTxMailBox[1].TDLR = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];
    CAN1->sTxMailBox[1].TDHR = 0;
    CAN1->sTxMailBox[1].TDHR = (msg.data[7] << 24) | (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4];

    //transmit message
    CAN1->sTxMailBox[1].TIR |= CAN_TI1R_TXRQ;
    return 1;
  }
  else if (CAN1->TSR & CAN_TSR_TME2) //check if mailbox1 is empty
  {
    CAN1->sTxMailBox[2].TIR = 0; //reset all values remaining from last transmission
    CAN1->sTxMailBox[2].TIR |= (msg.stdID << CAN_TI2R_STID_Pos) & CAN_TI2R_STID_Msk; //set message ID
    CAN1->sTxMailBox[2].TIR &= ~(CAN_TI2R_IDE); //set ID as standard 11bit ID
    CAN1->sTxMailBox[2].TIR |= (msg.RTR << CAN_TI2R_RTR_Pos) & CAN_TI2R_RTR_Msk; //set message as RTR or normal

    CAN1->sTxMailBox[2].TDTR = 0; //reset all values remaining from last transmission
    CAN1->sTxMailBox[2].TDTR |= (msg.DLC << CAN_TDT2R_DLC_Pos) & CAN_TDT2R_DLC_Msk;

    //fill data registers with data from message
    CAN1->sTxMailBox[2].TDLR = 0;
    CAN1->sTxMailBox[2].TDLR = (msg.data[3] << 24) | (msg.data[2] << 16) | (msg.data[1] << 8) | msg.data[0];
    CAN1->sTxMailBox[2].TDHR = 0;
    CAN1->sTxMailBox[2].TDHR = (msg.data[7] << 24) | (msg.data[6] << 16) | (msg.data[5] << 8) | msg.data[4];

    //transmit message
    CAN1->sTxMailBox[1].TIR |= CAN_TI2R_TXRQ;
    return 0;
  }

  else return -1;  //return error if all mailboxes are filled
}

//====================== receive message ========================//
/**
 *  @brief Gets message from receive FIFO.
 *
 *  If there is a pending message in one of receive mailboxes, this function rewrites all of it's content to
 *  given message structure and releases mailbox.
 *
 *  @param[in] msg pointer to CAN message data structure
 *
 *  @retval  0: no messages available in FIFOs
 *  @retval  1: message received from FIFO0
 *  @retval  2: message received from FIFO1
 */

uint8_t CAN1_get_message(CANbus_msg_t *msg)
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

    CAN1->RF1R |= CAN_RF1R_RFOM1;  //release mailbox 1

    return 2;
  }
  else return 0;
}

//====================== messages pending in FIFOs ========================//

/**
 *  @brief Returns amount of messages in FIFO0.
 *
 *  @return amount of messages waiting in FIFO0
 */
uint8_t CAN1_messages_pending_FIFO0()
{
  return (CAN1->RF0R & CAN_RF0R_FMP0_Msk) >> CAN_RF0R_FMP0_Pos;
}

/**
 *  @brief Returns amount of messages in FIFO1.
 *
 *  @return amount of messages waiting in FIFO1
 */
uint8_t CAN1_messages_pending_FIFO1()
{
  return (CAN1->RF1R & CAN_RF1R_FMP1_Msk) >> CAN_RF1R_FMP1_Pos;
}

/**
 *  @brief Returns sum of messages in FIFO0 and FIFO1.
 *
 *  @return sum of messages waiting in FIFO0 and FIFO1
 */
uint8_t CAN1_messages_pending()
{
  uint8_t FIFO0_pending = (CAN1->RF0R & CAN_RF0R_FMP0_Msk) >> CAN_RF0R_FMP0_Pos;
  uint8_t FIFO1_pending = (CAN1->RF1R & CAN_RF1R_FMP1_Msk) >> CAN_RF1R_FMP1_Pos;
  return FIFO0_pending + FIFO1_pending;
}

/**
 *  @brief Interrupt handler for receiving messages form FIFO0.
 */
void USB_LP_CAN1_RX0_IRQHandler()
{
  static CANbus_msg_t msg;
  CAN1_get_message(&msg);
  ringbuffer_put_msg(msg, &rx_buffer);
  NVIC_ClearPendingIRQ(USB_LP_CAN1_RX0_IRQn);
}

//====================== interrupt handlers ========================//

/**
 *  @brief Interrupt handler for receiving messages form FIFO1.
 */
void CAN1_RX1_IRQHandler()
{
  static CANbus_msg_t msg;
  CAN1_get_message(&msg);
  ringbuffer_put_msg(msg, &rx_buffer);
  NVIC_ClearPendingIRQ(CAN1_RX1_IRQn);
}

