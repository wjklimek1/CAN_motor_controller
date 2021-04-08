#ifndef INC_UART_H_
#define INC_UART_H_

void UART3_init(uint32_t sys_core_clk, uint32_t baudrate);
void UART3_send_byte(uint8_t tx_byte);
void UART3_print_string(char *string);

#endif /* INC_UART_H_ */
