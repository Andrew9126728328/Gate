#ifndef _MODBUS_
#define _MODBUS_

#define MASTER  0
#define SLAVE   1

extern void init_modbus(char);
extern void write_reg(monoblock_data *,char,char);
extern void read_reg(monoblock_data *,char,char);
extern void start_modbus_rx(void);
extern bool test_modbus_rx(void);
extern void save_modbus_data(char *);
#endif /* _MODBUS_ */
