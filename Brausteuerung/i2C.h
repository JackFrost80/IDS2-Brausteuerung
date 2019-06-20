/*
 * i2C.h
 *
 * Created: 16.06.2019 16:56:47
 *  Author: JackFrost
 */ 


#ifndef I2C_H_
#define I2C_H_
#define read 1
#define write 0

void lcd_init_SH1106();
void sendCommand_array(uint8_t *data,uint8_t length);

void sendCommand(uint8_t data);
void init_i2c();


#endif /* I2C_H_ */