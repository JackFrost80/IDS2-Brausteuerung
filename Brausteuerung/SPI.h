/*
 * SPI.h
 *
 * Created: 17.06.2019 22:42:14
 *  Author: JackFrost
 */ 


#ifndef SPI_H_
#define SPI_H_

#define CS_low PORT->Group[0].OUTCLR.reg = PORT_PA27
#define CS_high PORT->Group[0].OUTSET.reg = PORT_PA27

void init_SPI();
void RTD_Init();



#endif /* SPI_H_ */