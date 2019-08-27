#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "sounddata.h"

// audio variables
uint8_t audEnbl = 7;  //chip enable to HT82V739;
uint8_t audOut = 11;  //pwm tp HT82V739
volatile uint16_t audCounter;
uint8_t soundNum=1;
int soundLength = 0;
int soundLength1 = 0;
int soundLength2 = 0;
int soundLength3 = 0;

void stopPlayback()
{
    digitalWrite(audEnbl, HIGH);
    digitalWrite(audOut, LOW);
    cli();
}

ISR(TIMER0_COMPA_vect)
{
  OCR0A = TCNT0 + 30;
  if (audCounter >= soundLength-1) {
    stopPlayback();
  } else {
     switch(soundNum){
     case 0:
       //OCR2A = pgm_read_byte(&[audCounter]);
     break;
     case 1:
       OCR2A = pgm_read_byte(&sound_data2[audCounter]);
     break;
     case 2:
       //OCR2A = pgm_read_byte(&[audCounter]);
     break;
     default:
     break;
   }
  }
  ++audCounter;
}

void startPlayback(char* sp)
{
   if(sp==""){
     soundNum = 0;
     soundLength = soundLength1;
   }else if(sp=="sound_data2"){
     soundNum = 1;
     soundLength = soundLength2;
   }else if(sp==""){
     soundNum = 2;
     soundLength = soundLength3;
   }else{
     return;
   }
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);
    TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0);
    TCCR2B = (TCCR2B & ~(_BV(CS22) | _BV(CS21))) | _BV(CS20);
    OCR2A = pgm_read_byte(&sound_data[0]);
    audCounter = 0;
    digitalWrite(audEnbl, LOW);
    cli();
    TCCR0A &= ~(_BV(WGM01) | _BV(WGM00));
    TIMSK0 |= _BV(OCIE0A);
    TIFR0 |= _BV(OCF0A);
    sei();
}


void setup() {
  pinMode(audOut, OUTPUT);
pinMode(audEnbl, OUTPUT);
digitalWrite(audEnbl, HIGH);
//soundLength1 = sizeof ;
soundLength2 = sizeof sound_data2;
//soundLength3 = sizeof ;

    startPlayback("sound_data2");

}

void loop() {

}
