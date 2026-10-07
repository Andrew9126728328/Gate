#ifndef _GATE_
#define _GATE_
#include <iom162.h>
#include <ina90.h>
#include <string.h>

#define MCU_CLOCK   14745600
#define ERROR       1
#define MODBUS_BUFFER_SIZE  64

struct monoblock_data
    {
 
    char address;
    char number;
    char first_byte;
    char second_byte;
    char c_data[32];
    };
extern struct monoblock_data monoblock;
// Тип события
typedef enum {  NO_EVENT=0,TIMER_OVFL,IM_IN_NET,NET_ACCESS,NOT_NET_ACCESS,
                MONOBLOCK_DATA_RECEIVED,MODBUS_SENDED,MODBUS_RECEIVED,
                MODBUS_CHECK_RECEIVED,MODBUS_DATA_RECEIVED,
                MONOBLOCK_DATA_SENDED}event_type;
                
extern void put_event(event_type);      // Постановка события в очередь на обработку
#endif
