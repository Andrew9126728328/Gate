/*************************************************************************************************
Main.cpp
Основной файл программы "Шлюз между сетью моноблоков и ModBus"
Алгоритм работы: Автомат конечных состояний
Таблицы состояний, условий переходов, действий: STATE_MODBUS_NASTER_TABLE[]
                                                STATE_MODBUS_SLAVE_TABLE[]
                                                STATE_CONSOLE_TABLE[]
Подпрограмма реакции на событие: receive_event()
События формируются подпрограммой: put_event() как в этом файле, так и в других файлах проекта
**************************************************************************************************/
#include <iom162.h>
#include "gate.h"
#include "MB_Net.h"
#include "timer1.h"
#include "timer2.h"
#include "modbus.h"

extern char modbus_buffer[MODBUS_BUFFER_SIZE];

struct monoblock_data monoblock;
    
#define FIFO_SIZE   16          /* Должно быть 4, 8, 16, 32, 64 */
struct sequence                 // Очередь событий
    {
    event_type fifo[FIFO_SIZE];
    char in;
    char out;
    } event_sequence;     
// Тип режима
typedef enum {  MODBUS_MASTER=0,MODBUS_SLAVE,CONSOLE,LAST_MODE} mode_type;
#define NUM_MODES=MEASURE;
// Тип состояния
typedef enum {  NO_NEW_STATE=0,ANY_STATE,INIT,WAIT_MONOBLOCK_ANA,
                WAIT_CHECK_MODBUS_ANA,WAIT_MODBUS_ANA,
                WAIT_MONOBLOCK_SEND_ANA,WAIT_MONOBLOCK_DSCR,
                WAIT_CHECK_MODBUS_DSCR,WAIT_MODBUS_DSCR,
                WAIT_MONOBLOCK_SEND_DSCR,LAST_STATE} state_type;
// Тип указателя на функцию
typedef void (*fuction_pointer_type)(void);
// Базовая структура стейт машины
typedef struct state_table_type
    {
    state_type              current_state;      // Текущее состояние
    event_type              received_event;     // Принятое событие
    fuction_pointer_type    execute_function;   // Выполнить функцию
    state_type              next_state;         // Перейти в состояние
    mode_type               next_mode;          // Перейти в режим
    }state_table_type;

__no_init static mode_type    current_mode;           // Текущий режим
__no_init static state_type   current_state;          // Текущее состояние

static void null_function(void){};              // Пустая функция
static void init_state_mashine(void);           // Инициализация машины состояний
static void request_ana_from_monoblock(void);   // Сформировать и отправить запрос аналоговых данных из моноблока
static void send_ana_to_modbus(void);           // Сформировать и отправить аналоговые данные по модбасу
static void receive_ana_modbus_check(void);     // Прием квитанции по модбасу
static void test_ana_modbus_check(void);        // Проверка квитанции по модбасу 
static void request_ana_from_modbus(void);      // Сформировать и отправить запрос аналоговых данных данных из модбаса
static void receive_ana_modbus(void);           // Прием запрошенных аналоговых данных по модбасу
static void test_ana_modbus(void);              // Проверка принятых данных
static void send_ana_to_monoblock(void);        // Сформировать и отправить аналоговые данные в моноблок
static void request_dscr_from_monoblock(void);  // Сформировать и отправить запрос дискретных данных из моноблока
static void send_dscr_to_modbus(void);          // Сформировать и отправить дискретные данные по модбасу
static void receive_dscr_modbus_check(void);    // Прием квитанции по модбасу
static void test_dscr_modbus_check(void);       // Проверка квитанции по модбасу 
static void request_dscr_from_modbus(void);     // Сформировать и отправить запрос дискретных данных данных из модбаса
static void receive_dscr_modbus(void);          // Прием запрошенных дискретных данных по модбасу
static void test_dscr_modbus(void);             // Проверка принятых данных
static void send_dscr_to_monoblock(void);       // Сформировать и отправить дискретные данные в моноблок

__flash state_table_type STATE_MODBUS_NASTER_TABLE[]=    // Таблица остояний, условий перехода и действий режима ModBus мастер
    {
/*      из состояния                по событию              выполнить                   в состояние             в режим     */
        
        {ANY_STATE,                 NOT_NET_ACCESS,         init_state_mashine,         INIT,                   MODBUS_MASTER},
        {ANY_STATE,                 TIMER_OVFL,             init_state_mashine,         INIT,                   MODBUS_MASTER},
        {INIT,                      NET_ACCESS,             request_ana_from_monoblock, WAIT_MONOBLOCK_ANA,     MODBUS_MASTER},
        {WAIT_MONOBLOCK_ANA,        MONOBLOCK_DATA_RECEIVED,send_ana_to_modbus,         WAIT_CHECK_MODBUS_ANA,  MODBUS_MASTER},
        {WAIT_CHECK_MODBUS_ANA,     MODBUS_SENDED,          receive_ana_modbus_check,   NO_NEW_STATE,           MODBUS_MASTER},
        {WAIT_CHECK_MODBUS_ANA,     MODBUS_RECEIVED,        test_ana_modbus_check,      NO_NEW_STATE,           MODBUS_MASTER},
        {WAIT_CHECK_MODBUS_ANA,     MODBUS_CHECK_RECEIVED,  request_ana_from_modbus,    WAIT_MODBUS_ANA,        MODBUS_MASTER},
        {WAIT_MODBUS_ANA,           MODBUS_SENDED,          receive_ana_modbus,         NO_NEW_STATE,           MODBUS_MASTER},
        {WAIT_MODBUS_ANA,           MODBUS_RECEIVED,        test_ana_modbus,            NO_NEW_STATE,           MODBUS_MASTER},
        {WAIT_MODBUS_ANA,           MODBUS_DATA_RECEIVED,   send_ana_to_monoblock,      WAIT_MONOBLOCK_SEND_ANA,MODBUS_MASTER},
        {WAIT_MONOBLOCK_SEND_ANA,   MONOBLOCK_DATA_SENDED,  request_dscr_from_monoblock,WAIT_MONOBLOCK_DSCR,    MODBUS_MASTER},
        {WAIT_MONOBLOCK_DSCR,       MONOBLOCK_DATA_RECEIVED,send_dscr_to_modbus,        WAIT_CHECK_MODBUS_DSCR, MODBUS_MASTER},
        {WAIT_CHECK_MODBUS_DSCR,    MODBUS_SENDED,          receive_dscr_modbus_check,  NO_NEW_STATE,           MODBUS_MASTER},
        {WAIT_CHECK_MODBUS_DSCR,    MODBUS_RECEIVED,        test_dscr_modbus_check,     NO_NEW_STATE,           MODBUS_MASTER},
        {WAIT_CHECK_MODBUS_DSCR,    MODBUS_CHECK_RECEIVED,  request_dscr_from_modbus,   WAIT_MODBUS_DSCR,       MODBUS_MASTER},
        {WAIT_MODBUS_DSCR,          MODBUS_SENDED,          receive_dscr_modbus,        NO_NEW_STATE,           MODBUS_MASTER},
        {WAIT_MODBUS_DSCR,          MODBUS_RECEIVED,        test_dscr_modbus,           NO_NEW_STATE,           MODBUS_MASTER},
        {WAIT_MODBUS_DSCR,          MODBUS_DATA_RECEIVED,   send_dscr_to_monoblock,     WAIT_MONOBLOCK_SEND_DSCR,MODBUS_MASTER},
        {WAIT_MONOBLOCK_SEND_DSCR,  MONOBLOCK_DATA_SENDED,  request_ana_from_monoblock, WAIT_MONOBLOCK_ANA,     MODBUS_MASTER},
    };
    
__flash state_table_type STATE_MODBUS_SLAVE_TABLE[]=      // Состояния консольного ModBus слэйв
    {
        {ANY_STATE,NO_EVENT,null_function,NO_NEW_STATE,MODBUS_SLAVE}
    };

__flash state_table_type STATE_CONSOLE_TABLE[]=      // Состояния консольного режима
    {
        {ANY_STATE,NO_EVENT,null_function,NO_NEW_STATE,CONSOLE}
    };

__flash state_table_type __flash *MODE_TABLE[LAST_MODE]={   &STATE_MODBUS_NASTER_TABLE[0],     // Индекс по режимам
                                                            &STATE_MODBUS_SLAVE_TABLE[0],
                                                            &STATE_CONSOLE_TABLE[0]
                                                            };

__flash char STATE_TABLE_SIZE[]={sizeof(STATE_MODBUS_NASTER_TABLE)/sizeof(state_table_type),    // Количество состояний в каждом режиме
                                 sizeof(STATE_MODBUS_SLAVE_TABLE)/sizeof(state_table_type),
                                 sizeof(STATE_CONSOLE_TABLE)/sizeof(state_table_type)
                                 };
/**********************************************************************************************************
Постановка события в очередь на обработку. 
В качестве аргумента принимает тип "событие" и укладывает его в буфер очереди event_sequence.
***********************************************************************************************************/
__monitor void put_event(event_type event)             
{
    event_sequence.fifo[event_sequence.in]=event;
    event_sequence.in=++event_sequence.in&(FIFO_SIZE-1);
    if(event_sequence.in==event_sequence.out)       // Если переполнение буфера
        {
        ++event_sequence.out;                       // Теряем самое старое событие
        };       
}
/**********************************************************************************************************
Основная подпрограмма работы автомата конечных состояний
В качестве аргумента принимает событие, осуществляет поиск данной ситуации в таблице текущего режима,
выполняет заданную в таблице функцию и переводит автомат в следующее состояние и режим.
***********************************************************************************************************/
void receive_event(event_type event)
{
    struct state_table_type __flash *thisTable=MODE_TABLE[current_mode];

    for(char i=0;i<STATE_TABLE_SIZE[current_mode];++i,++thisTable)
        {
        if((event==thisTable->received_event) && ((current_state==thisTable->current_state) || (thisTable->current_state==ANY_STATE)))
            {                                                   
                (*thisTable->execute_function)();
                if ( thisTable->next_state != NO_NEW_STATE )
                {
                    current_state = thisTable->next_state;
                };
                current_mode = thisTable->next_mode;
                break;
            };
        };
}
/**********************************************************************************************************
Инициализация автомата конечных состояний
***********************************************************************************************************/
static void init_state_mashine(void)            
{    
    struct monoblock_data *pm=&monoblock;
    PORTB|=((1<<PORTB0)|(1<<PORTB1));
    pm->address=0;                      // Начинаем с привода подъема
    for(;;)                             
        {
        if(pm->address==12)             // Если последний адрес - переходим к первому
            pm->address=0;
        if(check_net(pm->address))      // Если текущий адрес присутствует в сети - поиск закончен
            break;
        else
            ++pm->address;              // Перейти к следующему моноблоку
        };
}
/**********************************************************************************************************
Сформировать и отправить запрос аналоговых данных из моноблока
***********************************************************************************************************/
static void request_ana_from_monoblock(void)    
{
    struct monoblock_data *pm=&monoblock;
    
    start_timeout(_250_mS);
    pm->number=0;                               // Число байт в посылке
    pm->first_byte=REQUEST;                     // Посылка - запрос
    pm->second_byte=ANA_FROM_MONOBLOCK;         // Код команды
    send_to_monoblock(pm);                      // Послать
    //put_event(MONOBLOCK_DATA_RECEIVED);    
}
/**********************************************************************************************************
Сформировать и отправить аналоговые данные по модбасу
***********************************************************************************************************/
static void send_ana_to_modbus(void)             
{    
    struct monoblock_data *pm=&monoblock;
    
    start_timeout(_250_mS);     
    pm->number = rx.number-2;                           // Число байт в посылке
    pm->c_data[0]=(char)net_member;       // Реестр членов сети
    pm->c_data[1]=(char)(net_member>>8);
    memcpy(&pm->c_data[2],(void*)&rx.buffer[2],pm->number); // Копируем данные для передачи в систему сбора
    write_reg(pm,0,(pm->number>>1)+1);                 // Послать аналоговые данные в систему сбора

    //put_event(MODBUS_SENDED);
}
/**********************************************************************************************************
Начинаем прием ответа по ModBus
***********************************************************************************************************/
static void receive_ana_modbus_check(void)
{
    start_timeout(_250_mS);
    start_modbus_rx();                          // Начать прием ответа от системы сбора
    //put_event(MODBUS_RECEIVED);
}
/**********************************************************************************************************
Проверка принятой посылки ModBus
***********************************************************************************************************/
static void test_ana_modbus_check(void)
{
    start_timeout(_250_mS);
    if(test_modbus_rx())                        // Если ответ корректен
        {
        put_event(MODBUS_CHECK_RECEIVED);       // Переходим в следующее состояние
        }
    else
        {
        };
}
/**********************************************************************************************************
Сформировать и отправить запрос аналоговых данных данных из модбаса
***********************************************************************************************************/
static void request_ana_from_modbus(void)        
{
    start_timeout(_250_mS);
    
    read_reg(&monoblock,16,(monoblock.number>>1));   // Послать запрос на чтение аналоговых данных из системы сбора
    //put_event(MODBUS_SENDED);    
}
/**********************************************************************************************************
Начинаем прием ответа по ModBus
***********************************************************************************************************/
static void receive_ana_modbus(void)
{
    start_timeout(_250_mS);
    start_modbus_rx();                          // Начать приеем ответа от системы сбора
    //put_event(MODBUS_RECEIVED);
}
/**********************************************************************************************************
Проверка принятой посылки ModBus
***********************************************************************************************************/
static void test_ana_modbus(void)
{
    start_timeout(_250_mS);
    if(test_modbus_rx())                        // Если ответ корректен
        {
        save_modbus_data(&monoblock.c_data[0]); // Переписать полученные данные в промежуточное хранилище
        put_event(MODBUS_DATA_RECEIVED);        // Переходим в следующее состояние
        }
    else
        {
        };
    //put_event(MODBUS_DATA_RECEIVED);
}
/**********************************************************************************************************
Сформировать и отправить аналоговые данные в моноблок
***********************************************************************************************************/
static void send_ana_to_monoblock(void)         
{
    struct monoblock_data *pm=&monoblock;
    
    start_timeout(_250_mS);
    pm->first_byte=SEND_DATA;                   // Посылка - данные
    pm->second_byte=ANA_FROM_MONOBLOCK;         // Код команды
    send_to_monoblock(pm);                      // Послать
    //put_event(MONOBLOCK_DATA_SENDED);       
}
/**********************************************************************************************************
Сформировать и отправить запрос дискретных данных из моноблока
***********************************************************************************************************/
static void request_dscr_from_monoblock(void)   
{
    struct monoblock_data *pm=&monoblock;
    
    start_timeout(_250_mS);
    pm->number=0;                               // Число байт в посылке
    pm->first_byte=REQUEST;                     // Посылка - запрос
    pm->second_byte=DSCR_FROM_MONOBLOCK;        // Код команды
    send_to_monoblock(pm);                      // Послать
    //put_event(MONOBLOCK_DATA_RECEIVED); 
}
/**********************************************************************************************************
Сформировать и отправить дискретные данные по модбасу
***********************************************************************************************************/
static void send_dscr_to_modbus(void)           
{
    struct monoblock_data *pm=&monoblock;
    
    start_timeout(_250_mS);
    pm->number = rx.number-2;                           // Число байт в посылке
    memcpy(pm->c_data,(void*)&rx.buffer[2],pm->number); // Копируем данные для передачи в систему сбора
    write_reg(pm,32,(pm->number>>1));                   // Послать аналоговые данные в систему сбора
}
/**********************************************************************************************************
Начинаем прием ответа по ModBus
***********************************************************************************************************/
static void receive_dscr_modbus_check(void)
{
    start_timeout(_250_mS);
    start_modbus_rx();                          // Начать приеем ответа от системы сбора
    //put_event(MODBUS_RECEIVED);
}
/**********************************************************************************************************
Проверка принятой посылки ModBus
***********************************************************************************************************/
static void test_dscr_modbus_check(void)
{
    start_timeout(_250_mS);
    if(test_modbus_rx())                        // Если ответ корректен
        {
        put_event(MODBUS_CHECK_RECEIVED);       // Переходим в следующее состояние
        }
    else
        {
        };
    //put_event(MODBUS_CHECK_RECEIVED);
}
/**********************************************************************************************************
Сформировать и отправить запрос дискретных данных данных из модбаса
***********************************************************************************************************/
static void request_dscr_from_modbus(void)      
{
    start_timeout(_250_mS);
    
    read_reg(&monoblock,40,(monoblock.number>>1));   // Послать запрос на чтение дискретных данных из системы сбора
    //put_event(MODBUS_SENDED); 
}
/******************************************************************************************************************
Включаем приемник и ожидаем окончания приема посылки по ModBus
*******************************************************************************************************************/
static void receive_dscr_modbus(void)
{
    start_timeout(_250_mS);
    start_modbus_rx();                          // Начать приеем ответа от системы сбора
    //put_event(MODBUS_RECEIVED);
}
/******************************************************************************************************************
Проверяем корректность принятой ModBus посылки
*******************************************************************************************************************/
static void test_dscr_modbus(void)
{
    start_timeout(_250_mS);
    if(test_modbus_rx())                        // Если ответ корректен
        {
        //PORTA = modbus_buffer[4];
        save_modbus_data(&monoblock.c_data[0]); // Переписать полученные данные в промежуточное хранилище
        put_event(MODBUS_DATA_RECEIVED);        // Переходим в следующее состояние
        }
    else
        {
        };    
    //put_event(MODBUS_DATA_RECEIVED); 
}
/******************************************************************************************************************
Сформировать и отправить дискретные данные в моноблок
*******************************************************************************************************************/
static void send_dscr_to_monoblock(void)        
{
    struct monoblock_data *pm=&monoblock;       
    
    start_timeout(_250_mS);
    pm->first_byte=SEND_DATA;                   // Посылка - данные
    pm->second_byte=DSCR_FROM_MONOBLOCK;        // Код команды
    send_to_monoblock(pm);                      // Послать
    //PORTA = pm->c_data[0];
    for(;;)                                     // Перейти к следующему моноблоку
        {
        if(++pm->address==12)                   // Если последний адрес - переходим к первому
            pm->address=0;
        if(check_net(pm->address))              // Если текущий адрес присутствует в сети - поиск закончен
            break;
        };
    //put_event(MONOBLOCK_DATA_SENDED);        
}
/******************************************************************************************************************
Основная программа
*******************************************************************************************************************/
__C_task int main(void)
{
    CLKPR=0x80;             // Клок прескайлер
    CLKPR=0x00;
    MCUCR=0x00;
    EMCUCR=0x00;
   __watchdog_reset();         // Сторожевой таймер
    WDTCR=(1<<WDCE)|(1<<WDE);
    WDTCR=(1<<WDE)|(6<<WDP0);   // 256 мс
    TIMSK=0x00;
    ETIMSK=0x00;
    DDRA=DDRC=0xFF;
    PORTA=PORTC=0x00;
    DDRB=0xFF;                      // Порт В на вывод
    PORTB=0x00;
    DDRD=(1<<PORTD2);               // Порт D2 на вывод, остальное на ввод
    PORTD=(1<<PORTD3)|(1<<PORTD4);  // Включить подтяжку на опросе дип-переключателей
    init_MB_net();                  // Инициализируем сеть моноблоков
    __enable_interrupt();           // Разрешить прерывания
//init_modbus(MODBUS_MASTER);
    if(!(PIND & (1<<PIND4)))
        current_mode=CONSOLE;       // Текущий режим - консоль
    else
        if(PIND & (1<<PIND3))
            {
            current_mode=MODBUS_MASTER;     // Текущий режим - ModBus master
            init_modbus(MODBUS_MASTER);     // Инициализация режима ModBus - master
            }
        else
            {
            current_mode=MODBUS_SLAVE;      // Текущий режим - ModBus slave
            init_modbus(MODBUS_SLAVE);      // Инициализация режима ModBus - slave
            };
    put_event(NOT_NET_ACCESS);  // Инициализируем машину состояний   
    for(;;)                     // Основной бесконечный цикл
        {
        if(event_sequence.in!=event_sequence.out)       // Если есть событие
            {
            receive_event(event_sequence.fifo[event_sequence.out]);     // Обработать его
            ++event_sequence.out&=(FIFO_SIZE-1);        // Убрать из буфера обработанное событие
            __watchdog_reset();                         // Сбросить сторожевой таймер
            };        
        switch(current_state)           // Синхронизация осциллографа по состояниям
            {
            case INIT:
                PORTA=(1<<0);
                break;
            case WAIT_MONOBLOCK_ANA:
                PORTA=(1<<1);
                break;
            case WAIT_CHECK_MODBUS_ANA:
                PORTA=(1<<2);
                break;
            case WAIT_MODBUS_ANA:
                PORTA=(1<<3);
                break;
            case WAIT_MONOBLOCK_SEND_ANA:
                PORTA=(1<<4);
                break;
            case WAIT_MONOBLOCK_DSCR:
                PORTA=(1<<5);
                break;
            case WAIT_MODBUS_DSCR:
                PORTA=(1<<6);
                break;
            case WAIT_MONOBLOCK_SEND_DSCR:
                PORTA=(1<<7);
                break;
            default:
                break;
            };
            
        };
    return ERROR;
}
