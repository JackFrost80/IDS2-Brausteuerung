/*
 * Brausteuerung.cpp
 *
 * Created: 14.06.2019 23:36:55
 * Author : JackFrost
 */ 


#include "sam.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "I2C.h"
#include "LCD.h"
#include "RTC2.h"
#include "SPI.h"
#include "PID.h"


#define KEY0            PIN_PA28
#define KEY1            PIN_PB13
#define cycle_time_calc	5*60

#define ALL_KEYS        (1<<KEY0 | 1<<KEY1)

#define REPEAT_MASK     (1<<KEY0)       // repeat: key1, key2
#define REPEAT_START    50                        // after 500ms
#define REPEAT_NEXT     20                        // every 200ms

#define PHASE_A     (PORT->Group[1].IN.reg & PORT_PB04)
#define PHASE_B     (PORT->Group[1].IN.reg & PORT_PB04)

#define cli __disable_irq
#define sei __enable_irq

int CMD[6][32] = {
	{0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},  // Aus
	{0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0},  // P1
	{0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0},  // P2
	{0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0},  // P3
	{0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},  // P4
	{0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0} };// P5
volatile bool praeamble = false;
volatile bool _end = false;
volatile uint32_t counter = 31;
volatile uint8_t Power_level = 0;
volatile uint32_t key_state = 0;                                // debounced and inverted key state:
// bit = 1: key pressed
volatile uint32_t key_press = 0;                                // key press detect

volatile uint32_t key_rpt = 0;      
volatile uint32_t count_1 = 0;
volatile uint32_t count_2 = 0;                          // key long press and repeat

static volatile uint16_t cap0[128]; // Period with PPW capture
static volatile uint16_t cap1[128]; // Pulse width with PPW capture
static volatile size_t nCap;
volatile uint8_t blink = 0;
volatile bool Data_ready =  false;
volatile uint16_t RTD_data = 0;
float setpoint = 33.0;
uint16_t output = 0;
PID_parameters PID;
float temp = 0;
volatile int8_t enc_delta;          // -128 ... 127
static int8_t last;
bool running =  false;
uint16_t counter_power = 0;

void RTC_Handler()
{
	if(RTC->MODE0.INTFLAG.bit.PER7)  // 1 Second
	{
		RTC->MODE0.INTFLAG.bit.PER7 = 1;
		counter_power++;
		counter_power %= cycle_time_calc;
	}
	if(RTC->MODE0.INTFLAG.bit.PER6)  // 0,5 Second
	{
		RTC->MODE0.INTFLAG.bit.PER6 = 1;
	}
	if(RTC->MODE0.INTFLAG.bit.PER5)  // 0,25 Second
	{
		RTC->MODE0.INTFLAG.bit.PER5 = 1;
		blink++;
		PID_calculations(&PID,&temp,&setpoint,&output,false,running);
		
	}
}

void TCC0_Handler()
{
	if (TCC0->INTFLAG.bit.OVF) {
		TCC0->INTFLAG.bit.OVF = 1;
		PORT->Group[0].OUTCLR.reg = PORT_PA22;
		//PORT->Group[0].OUTCLR.reg = PORT_PA15;
		if(counter >= 32)
		{
			if(!_end)
			{
				REG_TCC0_PERBUF =  0x1C250;
				REG_TCC0_CCBUF0 =  0x1400;
				TCC0->CTRLBCLR.bit.LUPD = 1;
				_end = true;
			}
			else
			{
				praeamble = true;
				_end = false;
				counter = 0;
				REG_TCC0_PERBUF =  0x11288;
				REG_TCC0_CCBUF0 =  0xC670;
				
			}
			
			
			//TCC0->PERBUF.reg = 0xFFFF<<6;
			//TCC0->CCBUF[0].reg = 0x1FFF<<6;
			//TCC0->CTRLBSET.bit.CMD = 0x03;
		}
		else
		if(!praeamble)
		REG_TCC0_CCBUF0 =  0x1400;
		
		
		//PORT->Group[0].OUTTGL.reg = PORT_PA15;
	}
	if (TCC0->INTFLAG.bit.MC0) {
		TCC0->INTFLAG.bit.MC0 = 1;
		PORT->Group[0].OUTSET.reg =  PORT_PA22;
		//PORT->Group[0].OUTSET.reg = PORT_PA15;
		if(!praeamble && !_end)
		{
			if(CMD[Power_level][counter] == 0)
			{
				REG_TCC0_PERBUF = 0x27B0;
				TCC0->CTRLBCLR.bit.LUPD = 1;
			}
			else
			{
				REG_TCC0_PERBUF = 0x6200;
				TCC0->CTRLBCLR.bit.LUPD = 1;
			}
			
			counter++;
		}
		if(praeamble)
		{
			
			REG_TCC0_PERBUF = 0x11288;
			TCC0->CTRLBCLR.bit.LUPD = 1;
			
			praeamble = false;
		}
	}
	

}

void TCC2_Handler()
{
	if (TCC1->INTFLAG.bit.MC1) {
		//TCC1->INTFLAG.bit.MC0 = 1;
		//TCC1->INTFLAG.bit.MC1 = 1;
		count_1 = TCC2->CC[0].reg;
		count_2 = TCC2->CC[1].reg;
		PORT->Group[0].OUTTGL.reg = PORT_PA15;
		
	}
}

void EIC_Handler()
{
	if (EIC->INTFLAG.bit.EXTINT) {
		EIC->INTFLAG.reg = 0xFFFF;
		Data_ready = true;
	}
}

void TC0_Handler()
{
	if (TC0->COUNT16.INTFLAG.bit.MC0) {
		// The interrupt flag is cleared by reading CC
		cap0[nCap] = TC0->COUNT16.CC[0].bit.CC;
		cap1[nCap] = TC0->COUNT16.CC[1].bit.CC;
		PORT->Group[0].OUTTGL.reg = PORT_PA15 | PORT_PA09;
		if (++nCap == sizeof(cap0)/sizeof(cap0[0])) {
			static volatile size_t done;
			done++;
			nCap = 0;
		
		}
	}
}

void setupTimer(void)
{
	MCLK->APBCMASK.bit.TCC0_ = 1;
	GCLK->PCHCTRL[TCC0_GCLK_ID].bit.GEN = 0;
	GCLK->PCHCTRL[TCC0_GCLK_ID].bit.CHEN = 1;
	while(GCLK->PCHCTRL[TCC0_GCLK_ID].bit.CHEN != 1);
	TCC0->CTRLA.bit.ENABLE = 0;
	while (TCC0->SYNCBUSY.bit.ENABLE);

	TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1;
	REG_TCC0_PERBUF = 0x5000;
	REG_TCC0_CCBUF0 = 0x2800;
	//while (TCC0->SYNCBUSY.bit.PER);
	TCC0->CTRLBSET.bit.CMD = 0x03;

	//TCC0->EVCTRL.reg = TCC_EVCTRL_TCEI1 |  // enable input event
	//TCC_EVCTRL_EVACT1_PPW; // event action = PPW
	// | TCC_EVCTRL_MCEI0 | TCC_EVCTRL_MCEI1;

	// Interrupts
	TCC0->INTENSET.bit.OVF = 1;
	TCC0->INTENSET.bit.MC0 = 1;
	//TCC0->INTENSET.bit.MC1 = 1; // Not needed, can read out both in MC0 interrupt
	NVIC_EnableIRQ(TCC0_IRQn);
	// Enable TCC
	TCC0->CTRLA.bit.ENABLE = 1;
	while (TCC0->SYNCBUSY.bit.ENABLE); // wait for sync
	

	MCLK->APBCMASK.bit.TCC1_ = 1;
	GCLK->PCHCTRL[TCC1_GCLK_ID].bit.GEN = 0;
	GCLK->PCHCTRL[TCC1_GCLK_ID].bit.CHEN = 1;
	while(GCLK->PCHCTRL[TCC1_GCLK_ID].bit.CHEN != 1);
	TCC1->CTRLA.bit.ENABLE = 0;
	while (TCC1->SYNCBUSY.bit.ENABLE);

	TCC1->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1;
	REG_TCC1_PERBUF = 0xE64;
	//while (TCC1->SYNCBUSY.bit.PER);
	TCC1->CTRLBSET.bit.CMD = 0x03;

	//TCC1->EVCTRL.reg = TCC_EVCTRL_TCEI1 |  // enable input event
	//TCC_EVCTRL_EVACT1_PPW; // event action = PPW
	// | TCC_EVCTRL_MCEI0 | TCC_EVCTRL_MCEI1;

	// Interrupts
	TCC1->INTENSET.bit.OVF = 1;
	//TCC1->INTENSET.bit.MC1 = 1; // Not needed, can read out both in MC0 interrupt
	NVIC_EnableIRQ(TCC1_IRQn);

	// Enable TCC
	TCC1->CTRLA.bit.ENABLE = 1;
	while (TCC1->SYNCBUSY.bit.ENABLE); // wait for sync
	
	//MCLK->APBCMASK.bit.TCC2_ = 1;
	//GCLK->PCHCTRL[TCC2_GCLK_ID].bit.GEN = 0;
	//GCLK->PCHCTRL[TCC2_GCLK_ID].bit.CHEN = 1;
	//while(GCLK->PCHCTRL[TCC2_GCLK_ID].bit.CHEN != 1);
	//TCC2->CTRLA.bit.ENABLE = 0;
	//while (TCC2->SYNCBUSY.bit.ENABLE);
//
	////TCC2 Input capture
	//TCC2->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_CPTEN0 | TCC_CTRLA_CPTEN1;
	//REG_TCC2_PERBUF = 0xFFFFFF;
	////while (TCC2->SYNCBUSY.bit.PER);
//
	//TCC2->EVCTRL.reg =   // enable input event
	//TCC_EVCTRL_EVACT1_PPW |  TCC_EVCTRL_TCEI1 | TCC_EVCTRL_TCEI0; // event action = PPW
	//// | TCC_EVCTRL_MCEI0 | TCC_EVCTRL_MCEI1;
//
	//// Interrupts
	////TCC2->INTENSET.bit.OVF = 1;
	//TCC2->INTENSET.bit.MC1 = 1;
	////TCC2->INTENSET.bit.MC1 = 1; // Not needed, can read out both in MC0 interrupt
	//NVIC_EnableIRQ(TCC2_IRQn);
//
	//// Enable TCC
	//TCC2->CTRLA.bit.ENABLE = 1;
	//while (TCC2->SYNCBUSY.bit.ENABLE); // wait for sync
	//
	//MCLK->APBCMASK.bit.EVSYS_ = 1;
	//// EVSYS GCLK is needed for interrupts and sync path (so not now)
	//EVSYS->USER[EVSYS_ID_USER_TCC2_EV_1].bit.CHANNEL = 2; // Channel n-1 selected so 1 here
	//while(!EVSYS->CHSTATUS.bit.USRRDY1);
	//
	//// EVSYS_CHANNEL_EDGSEL_BOTH_EDGES // not usable with  PATH_ASYNCHRONOUS
	//EVSYS->CHANNEL[1].reg = EVSYS_CHANNEL_PATH_ASYNCHRONOUS|EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_7);
	//while(EVSYS->CHSTATUS.bit.CHBUSY1);

	
}

void TCC1_Handler()
{
	if (TCC1->INTFLAG.bit.OVF) {
		TCC1->INTFLAG.bit.OVF = 1;
		static uint32_t counter = 0;
		if(counter%10 == 0)
		{
			static uint32_t ct0 = ~0x00, ct1 = ~0x00, rpt;
			volatile uint32_t i;
			i = PORT->Group[1].IN.reg & PORT_PB13;
			i += PORT->Group[0].IN.reg & (1<<KEY0);
			i = ~i;
			i = key_state ^ i;                      // key changed ?
			ct0 = ~( ct0 & i );                             // reset or count ct0
			ct1 = ct0 ^ (ct1 & i);                          // reset or count ct1
			i &= ct0 & ct1;                                 // count until roll over ?
			key_state ^= i;                                 // then toggle debounced state
			key_press |= key_state & i;                     // 0->1: key press detect
			
			if( (key_state & REPEAT_MASK) == 0 )            // check repeat function
			rpt = REPEAT_START;                          // start delay
			if( --rpt == 0 ){
				rpt = REPEAT_NEXT;                            // repeat delay
				key_rpt |= key_state & REPEAT_MASK;
			}
			counter = 0;
		}
		counter++;
		
		int32_t _new, diff;

		_new = 0;
		if( PORT->Group[1].IN.reg & PORT_PB04 ) _new = 3;
		if( PORT->Group[1].IN.reg & PORT_PB30 ) _new ^= 1;          // convert gray to binary
		diff = last - _new;               // difference last - new
		if( diff & 1 ) {                 // bit 0 = value (1)
			last = _new;                    // store new as next last
			enc_delta += (diff & 2) - 1;   // bit 1 = direction (+/-)
		}
		
		
	}
	
}

void setupInput_TC0(void)
{
	MCLK->APBAMASK.bit.EIC_ = 1;
	GCLK->PCHCTRL[EIC_GCLK_ID].bit.GEN = 0;
	GCLK->PCHCTRL[EIC_GCLK_ID].bit.CHEN = 1;
	while(GCLK->PCHCTRL[EIC_GCLK_ID].bit.CHEN != 1);
	// Input on PA22 which is EXTINT6
	PORT->Group[0].PMUX[23/2].bit.PMUXE = MUX_PA23A_EIC_EXTINT7;
	PORT->Group[0].PINCFG[23].bit.PMUXEN = 1;
	PORT->Group[0].PINCFG[23].bit.INEN = 1;
	PORT->Group[0].PINCFG[23].bit.PULLEN = 1;
	PORT->Group[0].OUTSET.reg = PORT_PA23;
	//PORT->Group[0].PINCFG[22].reg = pincfg.reg;
	EIC->CONFIG[0].bit.SENSE7 = EIC_CONFIG_SENSE7_HIGH_Val;
	EIC->EVCTRL.bit.EXTINTEO = 1 << 7;
	EIC->CTRLA.bit.ENABLE = 1;
	while (EIC->SYNCBUSY.bit.ENABLE == 1);
}


void setupInput(void)
{
	MCLK->APBAMASK.bit.EIC_ = 1;
	GCLK->PCHCTRL[EIC_GCLK_ID].bit.GEN = 0;
	GCLK->PCHCTRL[EIC_GCLK_ID].bit.CHEN = 1;
	while(GCLK->PCHCTRL[EIC_GCLK_ID].bit.CHEN != 1);
	
	// Input on PA04
	PORT->Group[0].PMUX[23/2].bit.PMUXE = MUX_PA23A_EIC_EXTINT7;
	//const PORT_PINCFG_Type pincfg = { .bit.PMUXEN = 1,.bit.INEN = 1,.bit.PULLEN = 1	};
	PORT->Group[0].PINCFG[23].bit.PMUXEN = 1;
	PORT->Group[0].PINCFG[23].bit.INEN = 1;
	PORT->Group[0].PINCFG[23].bit.PULLEN = 1;
	PORT->Group[0].OUTSET.reg = PORT_PA23;
	EIC->CONFIG[0].reg = EIC_CONFIG_SENSE7_LOW; // None of RISE,FALL or BOTH work for this
	EIC->EVCTRL.bit.EXTINTEO = 1<<7;
	
	// The EIC interrupt is only for verifying the input
	//EIC->INTENSET.bit.EXTINT = 1<<7;
	//NVIC_EnableIRQ(EIC_IRQn);
	while(EIC->SYNCBUSY.bit.ENABLE);
	EIC->CTRLA.bit.ENABLE = 1;
	while(EIC->SYNCBUSY.bit.ENABLE);
}


void setupTimer_TC0(void)
{
	MCLK->APBCMASK.bit.TC0_ = 1;
	GCLK->PCHCTRL[TC0_GCLK_ID].bit.GEN = 0;
	GCLK->PCHCTRL[TC0_GCLK_ID].bit.CHEN = 1;
	while(GCLK->PCHCTRL[TC0_GCLK_ID].bit.CHEN != 1);
	
	TC0->COUNT16.CTRLA.bit.ENABLE = 0;
	while (TC0->COUNT16.SYNCBUSY.reg);
	TC0->COUNT16.CTRLA.bit.MODE = TC_CTRLA_MODE_COUNT16;
	TC0->COUNT16.CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV1_Val;
	TC0->COUNT16.CTRLA.bit.CAPTEN0 = 1;
	TC0->COUNT16.CTRLA.bit.CAPTEN1 = 1;
	TC0->COUNT16.CTRLA.bit.COPEN0 = 0;
	TC0->COUNT16.CTRLA.bit.COPEN1 = 0;
	while (TC0->COUNT16.SYNCBUSY.reg);

	TC0->COUNT16.EVCTRL.bit.TCEI = 1;
	TC0->COUNT16.EVCTRL.bit.TCINV = 1;
	TC0->COUNT16.EVCTRL.bit.EVACT = TC_EVCTRL_EVACT_PPW_Val;
	// Interrupts
	TC0->COUNT16.INTENSET.bit.MC0 = 1;
	TC0->COUNT16.INTENSET.bit.MC1 = 0; // Not needed, can read out both in MC0 interrupt
	
	// Enable TC
	TC0->COUNT16.CTRLA.bit.ENABLE = 1;
	while (TC0->COUNT16.SYNCBUSY.reg);
	
	// Enable InterruptVector
	NVIC_EnableIRQ(TC0_IRQn);

	MCLK->APBCMASK.bit.EVSYS_ = 1;
	// EVSYS GCLK is needed for interrupts and sync path (so not now)
	EVSYS->USER[EVSYS_ID_USER_TC0_EVU].bit.CHANNEL = 2; // Channel n-1 selected so 1 here
	while(!EVSYS->CHSTATUS.bit.USRRDY1);
	
	// EVSYS_CHANNEL_EDGSEL_BOTH_EDGES // not usable with  PATH_ASYNCHRONOUS
	EVSYS->CHANNEL[1].reg = EVSYS_CHANNEL_PATH_ASYNCHRONOUS|EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_7);
	while(EVSYS->CHSTATUS.bit.CHBUSY1);
}

void encode_init( void )
{
	int8_t _new;

	_new = 0;
	if( PHASE_A ) _new = 3;
	if( PHASE_B ) _new ^= 1;       // convert gray to binary
	last = _new;                   // power on state
	
}

int8_t encode_read1( void )         // read single step encoders
{
	int8_t val;

	cli();
	val = enc_delta;
	enc_delta = 0;
	sei();
	return val;                   // counts since last call
}


int8_t encode_read2( void )         // read two step encoders
{
	int8_t val;

	cli();
	val = enc_delta;
	enc_delta = val & 1;
	sei();
	return val >> 1;
}


int8_t encode_read4( void )         // read four step encoders
{
	int8_t val;

	cli();
	val = enc_delta;
	enc_delta = val & 3;
	sei();
	return val >> 2;
}






///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed. Each pressed key is reported
// only once
//
uint32_t get_key_press( uint32_t key_mask__ )
{
	//NVIC_DisableIRQ(TCC1_IRQn);                                        // read and clear atomic !
	key_mask__ &= key_press;                          // read key(s)
	key_press ^= key_mask__;                          // clear key(s)
	//NVIC_EnableIRQ(TCC1_IRQn);
	return key_mask__;
}

///////////////////////////////////////////////////////////////////
//
// check if a key has been pressed long enough such that the
// key repeat functionality kicks in. After a small setup delay
// the key is reported being pressed in subsequent calls
// to this function. This simulates the user repeatedly
// pressing and releasing the key.
//
uint32_t get_key_rpt( uint32_t key_mask )
{
	//NVIC_DisableIRQ(TCC1_IRQn);                                         // read and clear atomic !
	key_mask &= key_rpt;                            // read key(s)
	key_rpt ^= key_mask;                            // clear key(s)
	//NVIC_EnableIRQ(TCC1_IRQn);
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
// check if a key is pressed right now
//
uint32_t get_key_state( uint32_t key_mask )

{
	key_mask &= key_state;
	return key_mask;
}

///////////////////////////////////////////////////////////////////
//
uint32_t get_key_short( uint32_t key_mask_ )
{
	//NVIC_DisableIRQ(TCC1_IRQn);                                         // read key state and key press atomic !
	return get_key_press( ~key_state & key_mask_ );
}

///////////////////////////////////////////////////////////////////
//
uint32_t get_key_long( uint32_t key_mask )
{
	return get_key_press( get_key_rpt( key_mask ));
}


int main(void)
{
	static bool override = false;
	uint8_t Power_level_limit = 5;
	uint8_t Encoder_status = 0;
	char temperature[30];
	bool change_heizung = false;
	bool change_setpoint = false;
	bool change_power_limit = false;
	uint8_t status_heizung = 0;
	uint8_t menu_point = 0;
	PID.cycle_time = 0.25;
	PID.Kp = 10;
	PID.Tv = 1.2;
	PID.Tn = 20;
	PORT->Group[1].PINCFG[4].bit.PULLEN = true;
	PORT->Group[1].PINCFG[30].bit.PULLEN = true;
	PORT->Group[1].PINCFG[13].bit.PULLEN = true;
	PORT->Group[1].PINCFG[4].bit.INEN = true;
	PORT->Group[1].PINCFG[30].bit.INEN = true;
	PORT->Group[1].PINCFG[13].bit.INEN = true;
	PORT->Group[1].OUTSET.reg = PORT_PB04 | PORT_PB30 | PORT_PB13;;
    /* Initialize the SAM system */
	char text_buffer[5];
    SystemInit();
	setupInput_TC0();
	setupTimer_TC0();
	setupTimer();
	init_i2c();
	rtc_init();
	init_SPI();
	RTD_Init();
	encode_init();
	PORT->Group[0].DIRSET.reg = PORT_PA15 | PORT_PA22 | PORT_PA09;
	PORT->Group[0].OUTTGL.reg = PORT_PA15 | PORT_PA22;
	PORT->Group[0].DIRCLR.reg = PORT_PA28;
	PORT->Group[0].PINCFG[28].bit.PULLEN = true;
	PORT->Group[0].OUTSET.reg = PORT_PA28;
	PORT->Group[0].PINCFG[28].bit.INEN = true;
	PORT->Group[1].DIRSET.reg = PORT_PB05 |PORT_PB06 |PORT_PB07;
	PORT->Group[1].OUTSET.reg = PORT_PB05;
	PORT->Group[0].DIRSET.reg = PORT_PA15 | PORT_PA22;
	init_LCD();
	lcd_gotoxy(1,0);
	lcd_puts("IDS2 Brauregelung");
	lcd_gotoxy(1,1);
	lcd_puts("PWR-Level: ");
	utoa(Power_level,text_buffer,10);
	lcd_puts(text_buffer);
	
    /* Replace with your application code */
    while (1) 
    {
		
		static float delta = 0;
		delta = setpoint - temp;
		if(delta >= -1.5 && delta <= 1.5 )
			PORT->Group[1].OUTSET.reg = PORT_PB07;
		else
			PORT->Group[1].OUTCLR.reg = PORT_PB07;
		Encoder_status = 0;
		if(change_power_limit)
			Encoder_status =+ 0x01;
		if(change_heizung)
			Encoder_status =+ 0x02;
		if(change_setpoint)
			Encoder_status =+ 0x04;
		switch(Encoder_status)
		{
			case 0:
			{
				menu_point += encode_read4();
				menu_point %= 4;
			}
			break;
			case 0x01 :
			{
				Power_level_limit +=encode_read4();
				Power_level_limit %= 6;
			}
			break;
			case 0x02 :
			{
				status_heizung +=encode_read4();
				status_heizung %= 2;
			}
			break;
			case 0x04 :
			{
				setpoint +=encode_read4();
				
			}
			break;
			default:
			{
				menu_point += encode_read4();
				menu_point %= 3;
			}
			break;
		}
		
		
		switch(menu_point)
		{
			case 0:
			{
				get_key_short(1<<KEY1);
				lcd_gotoxy(0,1);
				lcd_puts(" ");
				lcd_gotoxy(0,3);
				lcd_puts(" ");
				lcd_gotoxy(0,7);
				lcd_puts(" ");
			}
			break;
			case 1:
			{
				get_key_short(1<<KEY1);
				lcd_gotoxy(0,1);
				lcd_puts(">");
				lcd_gotoxy(0,3);
				lcd_puts(" ");
				lcd_gotoxy(0,7);
				lcd_puts(" ");
				if(change_power_limit)
				{
					lcd_gotoxy(1,1);
					sprintf(temperature,"PWR-Level: %d/",Power_level);
					lcd_puts(temperature);
					sprintf(temperature,"%d",Power_level_limit);
					lcd_puts_invert(temperature);
				}
				if(get_key_short(PORT_PB13))
				{
					if(change_power_limit)
					change_power_limit = false;
					else
					change_power_limit = true;
					
					
				}
			}
			break;
			case 2:
			{
				lcd_gotoxy(0,1);
				lcd_puts(" ");
				lcd_gotoxy(0,3);
				lcd_puts(">");
				lcd_gotoxy(0,7);
				lcd_puts(" ");
				if(change_setpoint)
				{
					lcd_gotoxy(1,3);
					lcd_puts("Sollwert: ");
					sprintf(temperature,"%.2f",setpoint);
					lcd_puts_invert(temperature);
				}
				if(get_key_short(PORT_PB13))
				{
					if(change_setpoint)
					change_setpoint = false;
					else
					change_setpoint = true;
					
					
				}
			}
			break;
			case 3:
			{
				lcd_gotoxy(0,1);
				lcd_puts(" ");
				lcd_gotoxy(0,3);
				lcd_puts(" ");
				lcd_gotoxy(0,7);
				lcd_puts(">");
				if(change_heizung)
				{
					if(status_heizung == 1)
					{
						lcd_gotoxy(1,7);
						lcd_puts_invert("Heizung an ");
					}
					else
					{
						lcd_gotoxy(1,7);
						lcd_puts_invert("Heizung aus");
					}
				}
				else
				{
					if(status_heizung == 1)
					{
						lcd_gotoxy(1,7);
						lcd_puts("Heizung an ");
					}
					else
					{
						lcd_gotoxy(1,7);
						lcd_puts("Heizung aus");
					}
				}
				if(get_key_short(PORT_PB13))
				{
					if(change_heizung)
					{
						change_heizung = false;
						if(status_heizung)
							running = true;
						else
							running = false;
					}
					else
						change_heizung = true;
					
						
				}
				
			}
			break;
			default:
			break;
		}
		
		
		
		if(Data_ready || override)
		{
			Data_ready = false;
			
			CS_low;
			SERCOM5->SPI.DATA.reg = 0x01;  // Read RTD MSB
			while(!SERCOM5->SPI.INTFLAG.bit.TXC);
			while(!SERCOM5->SPI.INTFLAG.bit.RXC);
			RTD_data = SERCOM5->SPI.DATA.bit.DATA;
			SERCOM5->SPI.DATA.reg = 0x55; 
			while(!SERCOM5->SPI.INTFLAG.bit.TXC);
			while(!SERCOM5->SPI.INTFLAG.bit.RXC);
			RTD_data = SERCOM5->SPI.DATA.bit.DATA;
			RTD_data = RTD_data << 8;
			SERCOM5->SPI.DATA.reg = 0x55;
			while(!SERCOM5->SPI.INTFLAG.bit.TXC);
			while(!SERCOM5->SPI.INTFLAG.bit.RXC);
			RTD_data += SERCOM5->SPI.DATA.bit.DATA;
			temp = RTD_data >>1;
			
			CS_high;
			temp /= 32;
			temp -= 256;
			char temperature[30];
			sprintf(temperature,"Temperatur: %.2f",temp);
			lcd_gotoxy(1,2);
			lcd_puts(temperature);
		}
		if(PORT->Group[1].IN.reg & (1<<31))
		{
			override = false;
		}
		else
		{
			override = true;
		}
		if(!change_setpoint)
		{
			sprintf(temperature,"Sollwert: %.2f",setpoint);
			lcd_gotoxy(1,3);
			lcd_puts(temperature);
		}
		sprintf(temperature,"Stellgrad: %d %",output);
		lcd_gotoxy(1,4);
		lcd_puts(temperature);
		
		if(get_key_short(1<<KEY0))
		{
			
			if(Power_level<=1)
				Power_level++;
			else
				Power_level = 0;
			
			lcd_gotoxy(1,1);
			lcd_puts("PWR-Level: ");
			utoa(Power_level,text_buffer,10);
			lcd_puts(text_buffer);
			
		}
		if(!change_power_limit)
		{
			sprintf(temperature,"PWR-Level: %d/%d",Power_level,Power_level_limit);
			lcd_gotoxy(1,1);
			lcd_puts(temperature);
		}
		
		if(running)
		{
			uint16_t power_level_calc = 0;
			if(output<= 10)
				Power_level = 0;
			if(output > 10 && output <=20)
			{
				power_level_calc = output - 10;
				power_level_calc *= 15;
				if(power_level_calc >= counter_power)
					Power_level = 1;
				else
					Power_level = 0;
				sprintf(temperature,"Power: %d/%d",power_level_calc,counter_power);
				lcd_gotoxy(1,5);
				lcd_puts(temperature);
				lcd_puts("  ");
				
			}
			if(output > 20 && output <=40)
			{
				power_level_calc = output - 20;
				power_level_calc *= 15;
				if(power_level_calc >= counter_power)
				if(Power_level_limit>=2)
					Power_level = 2;
				else
					Power_level = Power_level_limit;
				else
					Power_level = 1;
				sprintf(temperature,"Power: %d/%d",power_level_calc,counter_power);
				lcd_gotoxy(1,5);
				lcd_puts(temperature);
				lcd_puts("  ");
			}
			if(output > 40 && output <=60)
			{
				power_level_calc = output - 40;
				power_level_calc *= 15;
				if(power_level_calc >= counter_power)
					if(Power_level_limit>=3)
						Power_level = 3;
					else
						Power_level = Power_level_limit;
				else
					if(Power_level_limit>=3)
						Power_level = 2;
					else
						Power_level = Power_level_limit;
				sprintf(temperature,"Power: %d/%d",power_level_calc,counter_power);
				lcd_gotoxy(1,5);
				lcd_puts(temperature);
				lcd_puts("  ");
			}
			if(output > 60 && output <=80)
			{
				power_level_calc = output - 60;
				power_level_calc *= 15;
				if(power_level_calc >= counter_power)
					if(Power_level_limit>=4)
						Power_level = 4;
					else
						Power_level = Power_level_limit;
				else
					if(Power_level_limit>=4)
						Power_level = 3;
					else
						Power_level = Power_level_limit;
				sprintf(temperature,"Power: %d/%d",power_level_calc,counter_power);
				lcd_gotoxy(1,5);
				lcd_puts(temperature);
				lcd_puts("  ");
			}
			if(output > 80)
			{
				power_level_calc = output - 80;
				power_level_calc *= 15;
				if(power_level_calc >= counter_power)
					if(Power_level_limit>=5)
						Power_level = 5;
					else
						Power_level = Power_level_limit;
				else
					if(Power_level_limit>=5)
						Power_level = 4;
					else
						Power_level = Power_level_limit;
				sprintf(temperature,"Power: %d/%d",power_level_calc,counter_power);
				lcd_gotoxy(1,5);
				lcd_puts(temperature);
				lcd_puts("  ");
			}	
			
		}
		else
			Power_level = 0;
		
		if(Power_level > 0)
			PORT->Group[1].OUTSET.reg = PORT_PB06;
		else
			PORT->Group[1].OUTCLR.reg = PORT_PB06;
		//if(temp > 29)
		//{
			//PORT->Group[1].OUTSET.reg = PORT_PB07;
		//}
		//else
		//{
			//PORT->Group[1].OUTCLR.reg = PORT_PB07;
		//}
		
		if(!change_heizung)
		if(running)
		{
			lcd_gotoxy(1,7);
			lcd_puts("Heizung an ");
		}
		else
		{
			lcd_gotoxy(1,7);
			lcd_puts("Heizung aus");
		}
    }
}
