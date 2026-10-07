#ifndef _MB_Net_
#define _MB_Net_

/* Флаг управления данными */
extern volatile char contr_flag;
/* Где */
#define RX_DATA     (1<<0)
#define BAD_DATA    (1<<1)
#define READY_DATA  (1<<2)
#define TX_ENDED    (1<<3)
#define CHANGE_DATA (1<<4)
#define DELAY_TX    (1<<5)

#define ANA_FROM_MONOBLOCK  45
#define DSCR_FROM_MONOBLOCK 46
#define REQUEST             0xE0
#define SEND_DATA           0

extern void init_MB_net (void);                     // Инициализация сети моноблоков
extern bool check_net(char adr);                    // Проверка наличия устройства в сети
extern void send_to_monoblock(struct monoblock_data *data);   // Передать данные или команду в монблок

struct com
    {
    char buffer[35];
    char counter;
    char number;
    char check_sum;
    };
extern struct com volatile rx,tx;
extern volatile unsigned short net_member;         // Реестр членов сети

#ifdef _TIMER0_
#define MY_ADR  14     // Свой адрес
/* Флаг сети */
extern volatile char net_flag;
/* Где */
#define DATA_MY     (1<<0)
#define MARKER_MY   (1<<1)
#define REQ_REG     (1<<2)
#define IN_NET      (1<<3)
#define ONE_SIDE_TX (1<<4)
#define REQ_WIN     (1<<5)
#define TX_MY       (1<<6)
#define CMD_DATA    (1<<7)
extern volatile char adr_next;                     // Адрес преемника
extern volatile char m_counter;                    // Счетчик для контроля выпадения из сети
extern volatile char i_counter;                    // Счетчик, обеспечивающий порядок начальной инициализации сети
extern volatile char l_counter;                    // Счетчик, обеспечивающий порядок линкования сети

// Тип команды
typedef enum {MARKER=0,SEND_MARKER,REQUEST_ON_REGISTRATION,REGISTRATION,SET_SUCCESSOR,TICKET_OK,TICKET_FALSE,DATA}cmd_type;
extern void build_command(char,cmd_type);
#endif /* _TIMER0_ */
#endif /* _MB_net_ */
