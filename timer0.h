#ifndef _TIMER0_
#define _TIMER0_
#define TIME_OUT        0xFF-(char)(0.0021*MCU_CLOCK/1024)
#define TIME_REQ_WIN    0xFF-(char)(0.00085*MCU_CLOCK/1024)

extern void init_timer0(void);
extern void timer0_start(char);
extern void timer0_stop(void);
#endif
