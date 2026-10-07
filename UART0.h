#ifndef _UART0_
#define _UART0_

extern void init_UART0(unsigned long);
extern void send_uart0(char *,char);
extern void receive_uart0(char *);
extern char receive_counter_uart0(void);
#endif /* _UART0_ */
