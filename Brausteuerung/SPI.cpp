/*
 * SPI.cpp
 *
 * Created: 17.06.2019 22:42:02
 *  Author: JackFrost
 */ 
#include "sam.h"
#include "SPI.h"


void init_SPI()
{
	PORT->Group[1].PMUX[31/2].bit.PMUXE = MUX_PB31A_EIC_EXTINT15;
	//const PORT_PINCFG_Type pincfg = { .bit.PMUXEN = 1,.bit.INEN = 1,.bit.PULLEN = 1	};
	PORT->Group[1].PINCFG[31].bit.PMUXEN = 1;
	PORT->Group[1].PINCFG[31].bit.INEN = 1;
	PORT->Group[1].PINCFG[31].bit.PULLEN = 1;
	PORT->Group[1].OUTSET.reg = PORT_PB31;
	EIC->CONFIG[3].reg = EIC_CONFIG_SENSE7_FALL; // None of RISE,FALL or BOTH work for this PB31
	EIC->INTENSET.bit.EXTINT = 1<<15;
	NVIC_EnableIRQ(EIC_IRQn);
	MCLK->APBCMASK.bit.SERCOM5_ = 1;
	GCLK->PCHCTRL[SERCOM5_GCLK_ID_CORE].bit.GEN = 0;
	GCLK->PCHCTRL[SERCOM5_GCLK_ID_CORE].bit.CHEN = 1;
	while(GCLK->PCHCTRL[SERCOM5_GCLK_ID_CORE].bit.CHEN != 1);
	SERCOM5->SPI.CTRLA.bit.ENABLE = 0;
	SERCOM5->SPI.SYNCBUSY.bit.ENABLE = 1;
	while (SERCOM5->SPI.SYNCBUSY.bit.ENABLE);
	PORT->Group[0].OUTSET.reg = PORT_PA27; //CS High
	PORT->Group[0].DIRSET.reg = PORT_PA27; //CS
	SERCOM5->SPI.CTRLA.bit.DIPO = 0x00; // PAD0
	SERCOM5->SPI.CTRLA.bit.DOPO = 0x01; // PAD2 / SCK PAD3
	SERCOM5->SPI.CTRLA.bit.MODE = 0x03; // Master Mode
	SERCOM5->SPI.CTRLB.bit.RXEN = 1;
	SERCOM5->SPI.CTRLA.bit.CPHA = 1;
	SERCOM5->SPI.CTRLA.bit.CPOL = 1;
	PORT->Group[1].PMUX[1].bit.PMUXE = MUX_PB02D_SERCOM5_PAD0; //PAD0 MISO
	PORT->Group[1].PMUX[0].bit.PMUXE = MUX_PB00D_SERCOM5_PAD2; //PAD2 MOSI
	PORT->Group[1].PMUX[0].bit.PMUXO = MUX_PB01D_SERCOM5_PAD3; //PAD3 SCK
	PORT->Group[1].PINCFG[0].bit.PMUXEN = 1;
	PORT->Group[1].PINCFG[2].bit.PMUXEN = 1;
	PORT->Group[1].PINCFG[1].bit.PMUXEN = 1;
	SERCOM5->SPI.BAUD.bit.BAUD = 0x01;
	SERCOM5->SPI.CTRLA.bit.ENABLE = 1;
	SERCOM5->SPI.SYNCBUSY.bit.ENABLE = 1;
	while (SERCOM5->SPI.SYNCBUSY.bit.ENABLE);
}

void RTD_Init()
{
	volatile uint32_t test = 0;
	CS_low;
	SERCOM5->SPI.DATA.reg = 0x80;  // Write Config
	while(!SERCOM5->SPI.INTFLAG.bit.TXC);
	test= SERCOM5->SPI.DATA.bit.DATA;
	SERCOM5->SPI.DATA.reg = (1<<7) | (1<<6) | (1<<0); //BIAS on / Automode / 50Hz filter
	while(!SERCOM5->SPI.INTFLAG.bit.TXC);
	test= SERCOM5->SPI.DATA.bit.DATA;
	CS_high;
	
	
	
}