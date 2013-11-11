/*
Copyright (c) 2012, Toasted Circuits, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Neither the name of Toasted Circuits, Inc. nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL TOASTED CIRCUITS, INC. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Author: G. Andrew Stone
*/
#include "tlc594x.h"
#include <ctype.h>
//#include "SPI.h"
#include <avr/io.h> // /usr/lib/avr/include/avr/ (includes your chip specific header)


volatile uint8_t tlc_needXLAT=0;
#if 1
/** Enables the Timer1 Overflow interrupt, which will fire after an XLAT
    pulse */
#define set_XLAT_interrupt()
//TIFR1 |= _BV(TOV1); TIMSK1 = _BV(TOIE1)

/** Disables any Timer1 interrupts */
#define clear_XLAT_interrupt()
//TIMSK1 = 0

/** Enables the output of XLAT pulses */
//#define enable_XLAT_pulses()    TCCR1A = _BV(COM1A1) | _BV(COM1B1)
/** Disables the output of XLAT pulses */
//#define disable_XLAT_pulses()   TCCR1A = _BV(COM1B1)
#define enable_XLAT_pulses() 
#define disable_XLAT_pulses()

volatile void (*userFn)(void)=0;


/** Interrupt called after an XLAT pulse to prevent more XLAT pulses. */
ISR(TIMER1_COMPA_vect) //TIMER1_OVF_vect)
{
  #if 0
  digitalWrite(tlc594x::BLANK_PIN,HIGH);
  if (tlc_needXLAT)
    {
  digitalWrite(tlc594x::XLAT_PIN,HIGH);
  digitalWrite(tlc594x::XLAT_PIN,LOW);
    }
  digitalWrite(tlc594x::BLANK_PIN,LOW);
  tlc_needXLAT=0;
  #endif
  if (!userFn) return;
  userFn();
}

ISR(TIMER1_OVF_vect)
{
  if (!userFn) return;
  sei();
  userFn();
}


#endif
#if 1
//#define set_XLAT_interrupt()
//#define clear_XLAT_interrupt()
#endif

//#define SwBlankXlat() do { TIFR1 = OCF1A;


void    tlc594x::zero(void)
{
  for(int i =0 ; i < CHAIN_LENGTH * CHANNELS_PER_CHIP; i++) brightness[i] = 0;
}

void    tlc594x::init(void)
{
  digitalWrite(XLAT_PIN,LOW);
  digitalWrite(BLANK_PIN,HIGH);  // Let's start with it off
  
  pinMode(XLAT_PIN, OUTPUT);
  pinMode(BLANK_PIN, OUTPUT);
  pinMode(GSCLK_PIN, OUTPUT);

  // LED driver error pin
  pinMode(tlc594x::XERR_PIN,INPUT);

  

  // Setup SPI
  // You've got to set SS to output before you turn on the SPI
  pinMode(SS_PIN, OUTPUT);  // You can use this pin for something else but for SPI to work SS has got to be in output mode
  SPSR = _BV(SPI2X); // twice as fast
  SPCR = _BV(SPE) | _BV(MSTR) | _BV(CPOL) | _BV(CPHA);  // SPI enable master mode 3

  // If you turn these on before the SPCR, you may clock in an accidental bit
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);

  // Start black
  for(int i =0 ; i < CHAIN_LENGTH * CHANNELS_PER_CHIP; i++) brightness[i] = 0;

  setupTimers();
  update();
}

void tlc594x::stopPeriodicCallback(void)
{
  TIMSK1 &= ~(_BV(TOIE1) | _BV(OCIE1A));
}

void tlc594x::initPeriodicCallback(volatile void (&fn)(),bool ovf)
{
  userFn = &fn;
  if (ovf) TIMSK1 |= _BV(TOIE1);
  else TIMSK1 |= _BV(OCIE1A);
}

bool tlc594x::checkTemp(void)
{
  volatile int c=7;
  TCCR1A = 0;  // Turn off blank.

  digitalWrite(BLANK_PIN,HIGH);
  // Delay loop
  //for (volatile int i=0;i<1;i++); // c *= 11;
  bool xerr = !digitalRead(XERR_PIN);
  digitalWrite(BLANK_PIN,LOW);

  TCCR1A = _BV(COM1A1);

  // eliminate overtemperature
  if ((overTempCondition)&&(overTempCondition<millis())) overTempCondition=0;
  
  if (xerr)
    {
      overTempCondition = millis()+OVERTEMP_SOAK;
    }
  
  return xerr;
}


void tlc594x::setupTimers(void)
{
  // Note this timer setup fragment was inspired by the TLC5940 library by Alex Leone.
  // However, that library puts the XLAT inside the blank, which means that pin 9 & 10
  // are both used.  This library moves BLANK to pin 9 (COM1A1) and leaves XLAT outside
  // the timer system entirely.  This allows SPI that uses pin 10 (ethernet for example).

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)  
  // Turn timers on in the mega
  PRR0 &= ~(_BV(PRTIM2) | _BV(PRTIM1));
#endif
  
  TCCR1A = _BV(COM1A1);  
  TCCR1B = _BV(WGM13);   
  OCR1A  = 2;       
  ICR1   = PWM_PERIOD;
  //TIMSK1 = _BV(TOIE1);  // Enable overflow interrupt for user use
  //TIMSK1 = _BV(OCIE1A);  // Enable output compare match enable
  
  // Set up timer 2 to run GSCLK

  
  TCCR2A = _BV(COM2B1) | _BV(WGM21)  | _BV(WGM20);  // set on BOTTOM, clear on OCR2A (non-inverting), output on OC2B
  TCCR2B = _BV(WGM22);                              // Fast pwm with OCR2A top
  OCR2B  = 1;                                       // duty factor uses as short a pulse as possible
  OCR2A  = GSCLK_PERIOD;

  // Start PWM outputs for both timers
  TCCR2B |= _BV(CS20);
  TCCR1B |= _BV(CS10);
}

void tlc594x::resume(void)
{
  SPSR = 0; //_BV(SPI2X); // twice as fast
  SPCR = _BV(SPE) | _BV(MSTR) | _BV(SPR1) | _BV(SPR0); //| _BV(CPOL) | _BV(CPHA);  // SPI enable master mode 3  
}

// Wait for any prior SPI transmission to complete
#define spiWait() do {  } while (!(SPSR & _BV(SPIF)))

//int16_t gcnt=0;
void tlc594x::update(void)
{
  int16_t* bri = &brightness[NUM_CHANNELS-1];
  register int16_t b;
  register uint8_t data[3];
  //if (tlc_needXLAT) return;

  //digitalWrite(SCK_PIN,1);
  //digitalWrite(SCK_PIN,0);
  SPDR = 0;  // Clock in an extra so I don't have to check for 1st time thru in the while(!(SPSR)... code
  while(1)
    {      
      b = *bri;  //brightness[cnt];
      if (overTempCondition) b /= OVERTEMP_DIMMING;
      if (b<0) { data[0]=0; data[1]=0;}
      else
        {
          if (b>MAX_INTENSITY) {data[0]=0xff; data[1]=0xf; } // This code is hard-coded for MAX_INTENSITY = 4096
          else { data[0] = b>>4; data[1] = (b&0xf)<<4;}
        }

      // By waiting FIRST, I make it more likely that useful work can ben done while SPI is going   
      spiWait(); // while (!(SPSR & _BV(SPIF)));  // Wait for any prior SPI transmission to complete
      SPDR = data[0]; 

      // By interspersing the SPI writes I can calculate the next value while SPI is transmitting...
      bri--;
      b = *bri;
      if (overTempCondition) b /= OVERTEMP_DIMMING;
      if (b<0) data[2]=0;
      else
        {
          if (b>MAX_INTENSITY) { data[2] = 0xff; data[1] |= 0x0f; }
          else { data[1] |= b>>8; data[2] = b; }
        }
      
      spiWait(); // while (!(SPSR & _BV(SPIF)));  // Wait for any prior SPI transmission to complete
      SPDR = data[1]; 
      bri--;
      if (bri<=brightness) break;
      spiWait(); // while (!(SPSR & _BV(SPIF)));  // Wait for any prior SPI transmission to complete
      SPDR = data[2]; 
    }

  
  spiWait(); //while (!(SPSR & _BV(SPIF)));  // Wait for any prior SPI transmission to complete
  SPDR = data[2]; 
  spiWait(); //while (!(SPSR & _BV(SPIF)));  // Wait for the final SPI transmission to complete, just in case a different SPI user does not wait before transmitting.

  
#if 1 // This works with no visual difference  
  //digitalWrite(BLANK_PIN,HIGH);
  digitalWrite(XLAT_PIN,HIGH);
  digitalWrite(XLAT_PIN,LOW);
  //digitalWrite(BLANK_PIN,LOW);
#endif

  
  //tlc_needXLAT = 1;
  //SwBlankXlat();
  //enable_XLAT_pulses();
  //set_XLAT_interrupt();
}

