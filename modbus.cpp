/*************************************************************************************************
Modbus.cpp
Файл включается в проект "Шлюз между сетью моноблоков и ModBus"
Содержит функции и данные для реализации протокола ModBus
В данном впроекте используется для передачи и приема данных
в/из систему/ы сбора данных по протоколу ModBus
**************************************************************************************************/
#include "gate.h"
#include "modbus.h"
#include "uart0.h"
#include "timer2.h"

__no_init struct // Структура, описывающая slave куда (откуда) складываются (забираются) данные по ModBus
    {
    char address;           // Адрес slave
    unsigned long baudrate; // Скорость работы
    }__eeprom e_modbus@0x01;
    
char modbus_buffer[MODBUS_BUFFER_SIZE]; // Буффер данных ModBus
char modbus_cmd_old;
    
static unsigned short crc(char buf[],char); // П/п подсчета контрольной суммы
/*************************************************************************************************
Инициализация ModBus
В качестве аргумента получает режим работы: мастер или слэйв
**************************************************************************************************/
__monitor void init_modbus(char mode)
{
/*modbus_buffer[0]=0x01;
modbus_buffer[1]=0x10;
modbus_buffer[2]=0x21;
modbus_buffer[3]=0x00;
modbus_buffer[4]=0x00;
modbus_buffer[5]=0x02;
modbus_buffer[6]=0x4B;
modbus_buffer[7]=0xF4;
crc(modbus_buffer,8);*/

    if(mode==MASTER)                
        {
        if(e_modbus.address==0xff)  // Первое включение
            {
            e_modbus.address=1;
            e_modbus.baudrate=19200;
            };
        init_UART0((unsigned long)e_modbus.baudrate);   // Инициализация UART0
        TCCR2=(1<<WGM21)|(3<<CS20);                     // Настроить таймер 3 на обработку BREAK
        OCR2=(char)(MCU_CLOCK/32/e_modbus.baudrate);
        start_break(TX_BREAK);                          // Отсчет BREAK
        modbus_cmd_old=0;
        }
    else                            // Инициализация Modbus - slave
        {
        }; 
}
/*************************************************************************************************
Запись N регистров с заданного адреса 
**************************************************************************************************/
void write_reg(monoblock_data *data,char adrreg,char numb)
{
    char *ptm=modbus_buffer;
    unsigned short crc16;
    

    *ptm++=1;                // Slave address
    *ptm++=modbus_cmd_old=16;               // Command
    *ptm++=(char)((data->address*48+adrreg)>>8);  // Старший адрес регистров
    *ptm++=(char)(data->address*48+adrreg);       // Младший адрес регистров
    *ptm++=0;
    *ptm++=numb;                // Число регистров
    *ptm++=numb*2;              // Число байт
    for(char i=0;i<numb*2;i=i+2)  // Данные
        {
        *ptm++=data->c_data[i+1];
        *ptm++=data->c_data[i];
        };
    crc16=crc(modbus_buffer,numb*2+7);      // Считаем контрольную сумму
    *ptm++=(char)(crc16>>8);                // Контрольную сумму в буфер
    *ptm=(char)crc16;
    while(!is_break_complete());            // Ожидаем окончания BREAK
    send_uart0(modbus_buffer,numb*2+9);     // Передаем подготовленный буфер через UART0
}
/*************************************************************************************************
Чтение N регистров с заданного адреса
**************************************************************************************************/
void read_reg(monoblock_data *data,char adrreg,char numb)
{
    char *ptm=modbus_buffer;
    unsigned short crc16;
    

    *ptm++=1;                // Slave address
    *ptm++=modbus_cmd_old=03;               // Command
    *ptm++=(char)((data->address*48+adrreg)>>8);  // Старший адрес регистров
    *ptm++=(char)(data->address*48+adrreg);       // Младший адрес регистров
    *ptm++=0;
    *ptm++=numb;                // Число регистров
    crc16=crc(modbus_buffer,6); // Считаем контрольную сумму
    *ptm++=(char)(crc16>>8);    // Контрольную сумму в буфер
    *ptm=(char)crc16;
    while(!is_break_complete());    // Ждем окончания BREAK
    send_uart0(modbus_buffer,8);    // Передаем подготовленный буфер через UART0
}
/*************************************************************************************************
Прием ModBus посылки от слэйва
**************************************************************************************************/
void start_modbus_rx(void)          
{
    receive_uart0(modbus_buffer);
}
/*************************************************************************************************
Проверка корректности принятой ModBus посылки
**************************************************************************************************/
bool test_modbus_rx(void)           
{
    char rx_byte_number=receive_counter_uart0();
    unsigned short rx_crc=crc(modbus_buffer,rx_byte_number);    // Проверяем контрольную сумму
    return  (!rx_crc)&&(modbus_buffer[1]==modbus_cmd_old);      // Истина - если контрольная сумма сошлась и нет ошибки
        
}
/*************************************************************************************************
Сохранить принятые данные
**************************************************************************************************/
void save_modbus_data(char *dist)           
{
    char *source=modbus_buffer;
    char number=source[2];
    for(char i=0;i<number;i+=2)  // Данные
        {
        *dist++=source[i+4];
        *dist++=source[i+3];
        };    
}
/*
*****************************************************************************
***************************** [  BEGIN:  crc ] ******************************
*****************************************************************************
INPUTS:
   buf   ->  Array containing message to be sent to controller.
   cnt   ->  Amount of bytes in message being sent to controller/
OUTPUTS:
   temp  ->  Returns crc byte for message.
COMMENTS:
   This routine receives the data message to be sent down to the controller
and calculates the crc high and low byte of that message.
*****************************************************************************
*/
static unsigned short crc(char buf[],char cnt)
{
    char      i,j;
    unsigned short temp,flag;
    
    for(temp=0xFFFF,i=0; i<cnt; i++)
        {
        temp=temp ^ buf[i];
        for (j=1; j<=8; j++)
            {
	    flag=temp & 0x0001;
	    temp=temp >> 1;
	    if (flag) temp=temp ^ 0xA001;
            };
        };
    /* Reverse byte order. */
    i=(char)(temp >> 8);
    temp=(temp << 8) | i;
    return(temp);
}

