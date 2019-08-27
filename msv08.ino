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
       OCR2A = pgm_read_byte(&sound_data[audCounter]);
     break;
     case 1:
       //OCR2A = pgm_read_byte(&[audCounter]);
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
   if(sp=="sound_data"){
     soundNum = 0;
     soundLength = soundLength1;
   }else if(sp==""){
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

const uint8_t interruptPin = 2;
// remote
boolean  rmReceived = 0;  //信号受信完了した
uint8_t  i;               //受信データの桁
uint8_t  rmState = 0;     //信号受信状況
uint8_t  dataCode;        //データコード(8bit)
uint8_t  rmData;        //データコード(8bit)外部用
uint8_t  invDataCode;     //反転データコード(8bit)
uint16_t customCode;      //カスタムコード(16bit)
uint32_t rmCode;          //コード全体(32bit)
volatile uint32_t prevMicros = 0; //時間計測用

void rmUpdate() //信号が変化した
{
  uint32_t width; //パルスの幅を計測
  if(rmState != 0){
    width = micros() - prevMicros;  //時間間隔を計算
    if(width > 10000)rmState = 0; //長すぎ
    prevMicros = micros();  //次の計算用
  }
  switch(rmState){
    case 0: //信号未達
    prevMicros = micros();  //現在時刻(microseconds)を記憶
    rmState = 1;  //最初のOFF->ON信号を検出した
    i = 0;
    return;
    case 1: //最初のON状態
      if((width > 9500) || (width < 8500)){ //リーダーコード(9ms)ではない
        rmState = 0;
      }else{
        rmState = 2;  //ON->OFFで9ms検出
      }
      break;
    case 2: //9ms検出した
      if((width > 5000) || (width < 4000)){ //リーダーコード(4.5ms)ではない
        rmState = 0;
      }else{
        rmState = 3;  //OFF->ONで4.5ms検出
      }
      break;
    case 3: //4.5ms検出した
      if((width > 700) || (width < 400)){
        rmState = 0;
      }else{
        rmState = 4;  //ON->OFFで0.56ms検出した
      }
      break;
    case 4: //0.56ms検出した
      if((width > 1800) || (width < 400)){  //OFF期間(2.25-0.56)msより長い or (1.125-0.56)msより短い
        rmState = 0;
      }else{
        if(width > 1000){ //OFF期間長い -> 1
          bitSet(rmCode, (i));
        }else{             //OFF期間短い -> 0
          bitClear(rmCode, (i));
        }
        i++;  //次のbit
        if(i > 31){ //完了
          rmReceived = 1;
          return;
        }
        rmState = 3;  //次のON->OFFを待つ
      }
      break;
    }
}


void setup() {
  pinMode(audOut, OUTPUT);
pinMode(audEnbl, OUTPUT);
digitalWrite(audEnbl, HIGH);
soundLength1 = sizeof sound_data;
//soundLength2 = sizeof ;
//soundLength3 = sizeof ;

  attachInterrupt(digitalPinToInterrupt(interruptPin), rmUpdate, CHANGE);


}

void loop() {
  if(rmReceived){ //リモコン受信した
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    rmState = 0;      //初期化
    //図とは左右が逆であることに注意
    customCode = rmCode;    //下16bitがcustomCode
    dataCode = rmCode >> 16;  //下16bitを捨てたあとの下8bitがdataCode
    invDataCode = rmCode >> 24; //下24bitを捨てたあとの下8bitがinvDataCode
    if((dataCode + invDataCode) == 0xff){   //反転確認
      rmData = dataCode;
    }
    attachInterrupt(digitalPinToInterrupt(interruptPin), rmUpdate, CHANGE);
  }
  if (rmReceived == true) {
    if (rmData == 8) {
      startPlayback("sound_data");

    }
    rmReceived = false;

  }

}
