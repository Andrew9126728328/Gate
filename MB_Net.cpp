/*************************************************************************************************
MB_Net.cpp
Файл включается в проект "Шлюз между сетью моноблоков и ModBus"
Содержит функции и данные для реализации сети моноблоков
В данном впроекте используется для передачи и приема данных
в/из моноблоки
**************************************************************************************************/
#include "gate.h"
#include "timer0.h"
#include "MB_Net.h"

#ifdef __cplusplus
extern "C" {
#endif
// Внешние процедуры на ассемблере
char check_cmd(char);

#ifdef __cplusplus
}
#endif

#define BAUDRATE 28800
#define RXB8    1
#define TXB8    0
#define UPE     2
#define OVR     3
#define FE      4
#define TXEN    3 
#define RXEN    4
#define UDRIE   5
#define TXCIE   6
#define RXCIE   7

#define FRAMING_ERROR       (1<<FE)
#define PARITY_ERROR        (1<<UPE)
#define DATA_OVERRUN        (1<<OVR)
#define DATA_REGISTER_EMPTY (1<<UDRE)
#define RX_COMPLETE         (1<<RXC)

#define _100_uS             0.0001*MCU_CLOCK

__no_init volatile char net_flag;                     // Флаг сети 
__no_init volatile char contr_flag;                   // Флаг управления данными 
__no_init volatile char adr_next;                     // Адрес преемника
__no_init volatile char m_counter;                    // Счетчик для контроля выпадения из сети
__no_init volatile char i_counter;                    // Счетчик, обеспечивающий порядок начальной инициализации сети
__no_init volatile char l_counter;                    // Счетчик, обеспечивающий порядок линкования сети
__no_init volatile unsigned short net_member;         // Реестр членов сети

__no_init volatile static char adr_marker;            // Адрес текущего мастера сети
__no_init volatile static char adr_send;              // Адрес пересылки данных
__no_init volatile static char adr_send_old;          // Копия адреса пересылки данных

__no_init volatile static char p_counter;             // Количество попыток передачи данных
__no_init volatile static char net_par;               // Номер запрашиваемого по сети параметра

__no_init struct com volatile rx,tx;

    
static void send_check_OK(void);            // П/п передачи команды "Квитанция ОК"
static void send_check_false(void);         // П/п передачи команды "Квитанция FALSE" 
static void rx_marker(char);                // П/п реакции на принятую команду "Маркер"
static void rx_send_marker(char);           // П/п реакции на принятую команду "Передача маркера"
static void rx_req_reg(char);               // П/п реакции на принятую команду "Запрос на регистрацию"
static void rx_registration(char);          // П/п реакции на принятую команду "Регистрация"
static void rx_set_member(char);            // П/п реакции на принятую команду "Установить преемника"
static void rx_check(char);                 // П/п реакции на принятую команду "Квитанция"
static void rx_send_data(char);             // П/п реакции на принятую команду "Данные"
static char dist_com(char);                 // П/п вычисления адреса назначения команды
static void rec_memb(char);                 // П/п записи члена в реестр сети
/*************************************************************************************************
Инициализация сети
**************************************************************************************************/
__monitor void init_MB_net (void)
{ 
    /* Set baud rate */
    UBRR1H = (unsigned char)((MCU_CLOCK/16/BAUDRATE-1)>>8);
    UBRR1L = (unsigned char)(MCU_CLOCK/16/BAUDRATE-1);
    UCSR1B=(unsigned char)((1<<RXCIE1)|(1<<RXEN1)|(1<<TXEN1)|(1<<UCSZ12));
    UCSR1C=(unsigned char)((1<<URSEL1)|(1<<UCSZ11)|(1<<UCSZ10));    
    DDRB|=(1<<PORTB4);
    net_flag=contr_flag=0;
    init_timer0();
    
}

static void (*pfunc[])(char)={rx_marker,rx_send_marker,rx_req_reg,rx_registration,rx_set_member,rx_check,rx_check,rx_send_data};

// USART Receiver interrupt service routine
#pragma vector=USART1_RXC_vect
__interrupt void usart_rx_isr(void)
{
    char status=UCSR1A;
    char control=UCSR1B;
    char data=UDR1;

    //__enable_interrupt();
    /* Перезапустить тайм-аут */
    timer0_start(TIME_OUT);
    if ((status & (FRAMING_ERROR | DATA_OVERRUN))!=0) // Если аппаратный сбой приема
        {
        timer0_start(TIME_OUT);             // Перезапустить тайм-аут
        UCSR1B&=~((1<<RXCIE)|(1<<UDRIE));   // Запретить прерывания по приему и передаче
        net_flag=0;
        return;
        };
    if(net_flag&TX_MY)                      // Приняли свое эхо
        {
        com volatile *ptx=&tx; 
        if(data!=ptx->buffer[ptx->counter]) // Коллизия
            {
            UCSR1B&=~((1<<RXCIE)|(1<<UDRIE));   // Запретить прерывания по приему и передаче
            net_flag=0;
            contr_flag=0;
            return;
            };
        ++ptx->counter;
        if(ptx->counter==ptx->number)   // Если передали все закончить передачу
            {
            UCSR1B&=~(1<<UDRIE);
            ptx->counter=0;
            }
        else                            // Если передали не все продолжить передачу
            {
            net_flag&=~TX_MY;           // Сбросить флаг "передача моя"
            net_flag&=~CMD_DATA;        // Взводим флаг "в передатчике данные"                 
            UCSR1B|=(1<<UDRIE);
            return;
            };
        }
    else
        {
        __enable_interrupt();    
        /* Задержка 100 мкс */
        __delay_cycles(_100_uS);
        __disable_interrupt();
        };
    if(control&(1<<RXB8))                // Если принята команда
        {
        
        if(check_cmd(data)&0x01)        // Если команда принята с ошибками
            {
            UCSR1B&=~((1<<RXCIE)|(1<<UDRIE));   // Запретить прерывания по приему и передаче
            net_flag=0;
            contr_flag=0;
            return;
            };
        rx.counter=0;                       // Обнулить счетчик принятых байтов
        pfunc[data&0x07](data);             // Вызываем функцию, обслуживающую принятую команду
        }
    else                                // Если приняты данные
        {
        struct com volatile *prx=&rx;
        if(net_flag&DATA_MY)       // Данные наши
            {
            if(prx->counter==0)         // Первый байт данных
                {
                if(contr_flag&RX_DATA)  // Если предыдущие данные не прочитаны
                    {
                    contr_flag|=BAD_DATA;       // Взводим флаг "данные испорчены"
                    };
                prx->check_sum=data;            // Начинаем подсчет контрольной суммы
                prx->number=(data<REQUEST)? (data&0x1F)+2:2;
                prx->buffer[prx->counter++]=data;
                }
            else
                {
                if(prx->counter==prx->number)   // Если принят последний байт посылки
                    {
                    prx->counter=0;
                    if(prx->check_sum==data)  // Проверяем контрольную сумму
                        {
                        send_check_OK();            // Квитанция ОК
                        contr_flag|=RX_DATA;        // Взводим флаг "данные приняты"
                        put_event(MONOBLOCK_DATA_RECEIVED);    // Информируем стэйт-машину о прибытии запрошенных данных
                        }
                    else
                        {
                        send_check_false();         // Квитанция FALSE
                        contr_flag&=~RX_DATA;       // Сбрасываем флаг "данные приняты"
                        };
                    net_flag&=~DATA_MY;             // Сбрасываем флаг "данные наши"
                    }
                else
                    prx->check_sum^=data;               // Считаем промежуточную контрольную сумму
                    prx->buffer[prx->counter++]=data;   // Принятый байт в буфер приемника
                };            
            };
        };            
    net_flag&=~TX_MY;                   // Сбросить флаг "передача моя"
}
// USART Transmitter Complete interrupt service routine
#pragma vector=USART1_TXC_vect
__interrupt void usart_txc_isr(void)
{
    PORTB&=~(1<<PORTB4);      // Выключить передатчик 
    PORTB|=(1<<PORTB1);
    UCSR1B&=~(1<<TXCIE1);
}
// USART Transmitter UDRIE interrupt service routine
#pragma vector=USART1_UDRE_vect
__interrupt void usart_txudrie_isr(void)
{
    com volatile *ptx=&tx;      // 
    /* Перезапустить тайм-аут */
    TCCR0=0x05;
    TCNT0=TIME_OUT;
    OCR0=0x00; 
    UCSR1A|=(1<<TXC1);      // Сбросить флаг прерывания по окончанию передачи
    UCSR1B&=~(1<<TXB81);    
    if(net_flag&CMD_DATA)
        UCSR1B|=(1<<TXB81); // Если передаем команду взвести девятый бит
    PORTB|=(1<<PORTB4);     // Включить передатчик
    PORTB&=~(1<<PORTB1);
    UDR1=ptx->buffer[ptx->counter]; // Очередной байт из буфера в передатчик
    UCSR1B|=(1<<TXCIE1);    // Разрешить прерывания по окончанию передачи
    net_flag|=TX_MY;        // Установить флаг "передача моя"
    net_flag&=~CMD_DATA;    // Сбросить флаг "команда управления сетью"
    UCSR1B&=~(1<<UDRIE1);   // Запретить прерывания по готовности передатчика
    UCSR1B|=(1<<TXEN1);     // Разрешить передачу
}
/*************************************************************************************************
Вычислить адрес назначения команды
**************************************************************************************************/
static char dist_com(char cmd)  
{
    return cmd>>4;
}
/*************************************************************************************************
Запомнить члена сети
**************************************************************************************************/
static void rec_memb(char cmd)  
{
    net_member|=(1<<(cmd>>4));
}
/*************************************************************************************************
Формирование команды
**************************************************************************************************/
void build_command(char adr,cmd_type cmd)
{
    char tmp;
    tmp=cmd|(adr<<4);           // Формируем команду
    if(check_cmd(tmp)&0x01)     
        tmp|=(1<<3);
    tx.buffer[0]=tmp;           // Команду в буфер передатчика
    tx.number=1;                // Количество байт для передачи
    net_flag|=CMD_DATA;         // Взводим флаг "в передатчике команда"
    UCSR1B|=(1<<UDRIE1);        // Разрешить передачу
}
/*************************************************************************************************
Передать "квитанция ОК"
**************************************************************************************************/
static void send_check_OK(void) 
{
    build_command(adr_marker,TICKET_OK);
    contr_flag|=RX_DATA;
}
/*************************************************************************************************
Передать "квитанция FALSE"
**************************************************************************************************/
static void send_check_false(void)  
{
    build_command(adr_marker,TICKET_FALSE);
    contr_flag&=~RX_DATA;
} 
/*************************************************************************************************
Реакция на прием команды "Маркер"
**************************************************************************************************/
static void rx_marker(char cmd)     
{
    adr_marker=dist_com(cmd);       // Запомнить текущего мастера сети
    rec_memb(cmd);                  // Записать его в реестр членов сети
    if(net_flag&MARKER_MY)          // Если маркер мой
        {
        m_counter=0;                // Сбросить счетчик макеров
        if(contr_flag&READY_DATA)   
            {
            build_command(adr_send,DATA);   // Если есть данные для передачи начинаем передачу         
            }
        else
            {
            build_command(adr_next,SEND_MARKER);    // Если данных нет передаем маркер
            net_flag&=~MARKER_MY;                   // Сбросить флаг "макер мой"
            };
        }
    else                            // Если маркер не мой
        {
        if(!(net_flag&IN_NET)) return;    // Если я не в сети - на выход
        if(++m_counter>=16)             // Иначе проверяем не забыли обо мне в сети?
            m_counter=net_flag=0;       // Если забыли деинициализируем
        };
}
/*************************************************************************************************
Реакция на прием команды "Передача маркера"
**************************************************************************************************/
static void rx_send_marker(char cmd)
{
    if(dist_com(cmd)==MY_ADR)
        {
        net_flag|=(MARKER_MY | REQ_WIN);    // Если маркер передается мне - взводим флаг "маркер мой"
        timer0_start(TIME_REQ_WIN);         // Формируем окно запроса
        if((adr_next!=MY_ADR) && (!(contr_flag&READY_DATA)) && (net_member & 0x0FFF))
            put_event(NET_ACCESS);              
        }
    else
        {
        if(net_flag&IN_NET) return; // Если я уже в сети - на выход
        if(l_counter!=MY_ADR)       
            {
            ++l_counter;            // Если еще не пора регистрироваться - инкрементируем счетчик
            }
        else
            {
            l_counter=0;            // Если пора регистрироваться обнулить счетчик
            net_flag|=REQ_REG;      // Взвести флаг "запрос на регистрацию мой"
            build_command(MY_ADR,REQUEST_ON_REGISTRATION);  // Послать команду "Запрос на регистрацию"
            };
        };
};
/*************************************************************************************************
Реакция на прием команды "Запрос на регистрацию"
**************************************************************************************************/
static void rx_req_reg(char cmd)    
{
    char temp_adr_next=adr_next;
    char temp_adr_reg=dist_com(cmd);
    if(!(net_flag&IN_NET)) return;              // Если я не в сети - на выход
    if(temp_adr_next<=MY_ADR) temp_adr_next+=16; // Если адрес преемника меньше своего делаем попраку на круг
    if(temp_adr_reg<=MY_ADR) temp_adr_reg+=16;   // То же самое с адресом устройства, попросившего регистрацию
    if(temp_adr_reg<temp_adr_next)              // Если адрес претендента меньше адреса моего преемника
        build_command(adr_next,SET_SUCCESSOR);  // Передаем команду "Установить преемника"
}
/*************************************************************************************************
Реакция на прием команды "Регистрация"
**************************************************************************************************/
static void rx_registration(char cmd)
{
    char temp_adr_next=adr_next;
    char temp_adr_reg=dist_com(cmd);
    if(!(net_flag&IN_NET)) return;              // Если я не в сети - на выход
    if(temp_adr_next<=MY_ADR) temp_adr_next+=16; // Если адрес преемника меньше своего делаем попраку на круг
    if(temp_adr_reg<=MY_ADR) temp_adr_reg+=16;   // То же самое с адресом устройства, попросившего регистрацию
    if(temp_adr_reg<temp_adr_next)              // Если адрес претендента меньше адреса моего преемника 
        adr_next=temp_adr_reg&0x0F;             // Фиксируем его своим преемником
    if(net_flag&MARKER_MY)
        build_command(MY_ADR,MARKER);            // Если маркер мой - передаем команду "Маркер мой"
}
/*************************************************************************************************
Реакция на прием команды "Установить преемника"
**************************************************************************************************/
static void rx_set_member(char cmd) 
{
    if(!(net_flag&REQ_REG)) return;               // Если запрос на регистрацию не мой - на выход    
    adr_next=dist_com(cmd);                     // Запомнить адрес преемника
    net_flag|=IN_NET;                           // Установить флаг "я в сети"
    net_flag&=~REQ_REG;                         // Сбросить флаг "запрос на регистрацию мой"
    p_counter=0;                                // Сбросить счетчик количества попыток передачи
    build_command(MY_ADR,REGISTRATION);         // Передаем команду "Регистрация"
};
/*************************************************************************************************
Реакция на прием команды "Квитанция"
**************************************************************************************************/
static void rx_check(char cmd)   
{
    if(dist_com(cmd)!=MY_ADR) return;           // Если команде не мне - на выход
    if((cmd&0x07)==TICKET_OK)                   // Если "Квитанция ОК"
        {
        contr_flag&=~READY_DATA;                // Сбросить флаг "данные для передачи готовы"
        p_counter=0;                            // Обнулить счетчик попыток передачи
        put_event(MONOBLOCK_DATA_SENDED);       // Информируем стэйт-машину об успешной передаче данных
        }
    else                                        // Если "Квитанция FALSE"
        {
        if(p_counter==4)                        // Если счетчик попыток передачи равен 4 - на выход
            {
            UCSR1B&=~((1<<RXCIE)|(1<<UDRIE));   // Запретить прерывания по приему и передаче
            net_flag=0;
            contr_flag=0;
            return;
            };
        ++p_counter;                            // Иначе инкрементируем счетчик попыток передачи
        }; 
    build_command(adr_next,SEND_MARKER);        // Передаем команду "Передача маркера"
    net_flag&=~MARKER_MY;                       // Сбросить флаг "макер мой"
}
/*************************************************************************************************
Реакция на прием команды "Передача данных"
**************************************************************************************************/
static void rx_send_data(char cmd)  
{
    if(dist_com(cmd)==MY_ADR)                   // Если команде мне 
        {
        net_flag|=DATA_MY;                      // Взвести флаг "данные мне"
        adr_send=adr_marker;                    // Установить адрес устройства для передачи = адресу текущего мастера
        }
    else
        {
        if(net_flag&TX_MY)                      // Данные передаю я?
            {
            struct com volatile *ptx=&tx;
            char i=1;
            char num;
            char ch_sum;
            ptx->number=num=((ch_sum=ptx->buffer[i++]) & 0xE0)? 4:(ch_sum&0x1F)+4;  // Вычислить число байтов для передачи
            num-=2;
            while(--num)                        // Считаем контрольную сумму
                {
                ch_sum^=ptx->buffer[i++];
                };
            ptx->buffer[i]=ch_sum;              // Положить контрольную сумму в буфер
            ptx->counter=1;                     // Установить начальное значение счетчика данных для передатчика
            net_flag&=~CMD_DATA;                // Взводим флаг "в передатчике данные"
            UCSR1B|=(1<<UDRIE1);                // Разрешить передачу
            };
        };
}
/*************************************************************************************************
Проверить наличие члена сети
**************************************************************************************************/
bool check_net(char adr)                
{
    return (net_member & (1<<adr));
}
/*************************************************************************************************
Передать данные или команду в монблок
**************************************************************************************************/
void send_to_monoblock(struct monoblock_data *data)   
{
    char i;
    char *from,*to;
    if(!check_net(data->address))        // Если моноблока нет в сети
        {
        if((data->first_byte&0xE0)==REQUEST)
            {
            to=&data->c_data[0];
            for(i=0;i<32;++i)               // Подготовить нулевые данные
                {
            *to=0;
                };
            put_event(MONOBLOCK_DATA_RECEIVED);      // Данные готовы
            }
        else
            {
            put_event(MONOBLOCK_DATA_SENDED);       // Данные переданы
            };
        return;
        };
    while(contr_flag&READY_DATA);           // Ждем ести предыдущие данные не отправлены
    tx.buffer[1]=data->first_byte|data->number;      // В буфер передатчика первый служебный байт
    tx.buffer[2]=data->second_byte;                  // В буфер передатчика второй служебный байт
    i=data->number;
    to=(char*)&tx.buffer[3];
    from=&data->c_data[0];
    while(i--)                              // Переписать данные в буфер передатчика
        *to++=*from++;
    adr_send=data->address;                 // Установить адрес моноблока
    contr_flag|=READY_DATA;                 // Установить признак "данные готовы к передаче"
    contr_flag&=~(RX_DATA|BAD_DATA);        // Сбросить флаги "данные приняты" и "данные испорчены"
}

