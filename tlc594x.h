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
#ifndef TLC594X_H
#define TLC594X_H

#include "Arduino.h"
#include <ctype.h>

extern volatile void (*tlcUpdate)(void);

class tlc594x
{
public:
  enum
  {
      CHAIN_LENGTH       = 4,         //? Set to the number of chips in your serial chain

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
      XERR_PIN          = 7,
      XLAT_PIN            = 49,    // pick any unused pin
      BLANK_PIN         = 11,  // Timer 1 
      GSCLK_PIN         = 9,   // Cant be changed; this is hooked to a timer (timer2 pin PH6)
#elif defined(__AVR_ATmega48A__) | defined(__AVR_ATmega48__) | defined(__AVR_ATmega48P__) | defined(__AVR_ATmega48PA__) | defined (__AVR_ATmega168__) || defined (__AVR_ATmega328__) || defined (__AVR_ATmega328P__)
      XERR_PIN          = 7,
      XLAT_PIN          = 8,  // pick any unused pin
      BLANK_PIN         = 9,  // Cant be changed; this is hooked to a timer
      GSCLK_PIN         = 3,  // Cant be changed; this is hooked to a timer
#endif
      
    SCK_PIN           = SCK,
    MOSI_PIN          = MOSI,
    SS_PIN            = SS,

      OVERTEMP_DIMMING  = 4,  //? Divide all intensities by this durin an overtemperature condition
      OVERTEMP_SOAK     = 30000, //? The minimum time in mS to dim the LEDs during an overtemperature
    MAX_INTENSITY     = 4095,       //? The maximum brightness level for each channel
    CHANNELS_PER_CHIP = 16,
    NUM_CHANNELS      = CHANNELS_PER_CHIP * CHAIN_LENGTH,
    NUM_CHIPS         = CHAIN_LENGTH,

    GSCLK_PERIOD      = 6,                         //? Length of a single tick of the TLC chip -- each tick is an opportunity for a LED to change state.  3 is about the lowest you can go without seeing artifacts. But you can certainly go much higher before seeing flicker.
    PWM_PERIOD        = ((1+GSCLK_PERIOD)*4096)/2, //? Length of time of a full PWM blink of each LED in CPU clocks
  };

  //? <var>Set this variable to the desired brightness, indexed by the LED you want to control</var>
  int16_t brightness[NUM_CHANNELS];
  //? An indicator of when to leave an overtemperature condition (0 = not in overtemperature)
  unsigned long int overTempCondition;
  
  //? Set a channel to a particular intensity.  Pass the zero-based chip #, zero-based channel and the intensity 0-MAX_INTENSITY
  //  The intensity is saturating -- that is values below zero will turn the channel off, and values above will turn it fully on.
  void set(const uint16_t chip, const uint8_t channel, const int16_t intensity)
  {
    set((chip*CHANNELS_PER_CHIP)+channel, intensity);    
  }
    
  //? Set a channel to a particular intensity.  Pass the zero-based chip #, zero-based channel and the intensity 0-MAX_INTENSITY
  //  The intensity is saturating -- that is values below zero will turn the channel off, and values above will turn it fully on.
  void set(const uint16_t idx,const int16_t intensity)
  {
    brightness[idx] = intensity;
  }

  void    zero(void);
  void    init(void);

  // This may cause the LEDs to flicker so should be used rarely.  True means overtemperature!
  bool    checkTemp(void);

  void    initPeriodicCallback(volatile void (&fn)(),bool ovf=1);
  void    stopPeriodicCallback();
 
  void    pause(void) { /* Nothing to do there's no SS */ };  //? Temporarily stop using SPI for this device
  void    resume(void); //? Continue using SPI

  void    update(void);

  void setupTimers(void);
};

#endif
