#ifndef _TIMER2_
#define _TIMER2_

#define TX_BREAK    37
#define RX_BREAK    33

extern bool is_break_complete(void);
extern void start_break(char);

#endif /* _TIMER2_ */
