#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <Servo.h>
#include "sounddata.h"

#define STOP  0
#define FWRD  1
#define BWRD  2
#define RIGHT  3
#define LEFT  4

const uint8_t interruptPin = 2;

// servo variables
Servo myservo1;
Servo myservo2;
uint8_t actionMode = 0;
int8_t offset1 = -4;
int8_t offset2 = -5;

// audio variables
uint8_t audEnbl = 7;  //chip enable to HT82V739
uint8_t audOut = 11;  //pwm tp HT82V739
volatile uint16_t audCounter;

// remote
boolean  rmRecieved = 0;  //信号受信完了した
uint8_t  i;               //受信データの桁
uint8_t  rmState = 0;     //信号受信状況
uint8_t  dataCode;        //データコード(8bit)
uint8_t  invDataCode;     //反転データコード(8bit)
uint16_t customCode;      //カスタムコード(16bit)
uint32_t rmCode;          //コード全体(32bit)
volatile uint32_t prevMicros = 0; //時間計測用

void stopPlayback()
{
    digitalWrite(audEnbl, HIGH);
    digitalWrite(audOut, LOW);
    cli();
}

ISR(TIMER0_COMPA_vect)
{
  // causing an interrupt every (30+1) x 4us = 124us(=~8000Hz)
  OCR0A = TCNT0 + 30; 
  if (audCounter >= sound_length-1) {
    stopPlayback();
  } else {
    OCR2A = pgm_read_byte(&sound_data[audCounter]);
  }
  ++audCounter;
}

void startPlayback()
{
    // timer 2 settings for make pwm audio
    //
    // Use internal clock (datasheet p.167)
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));

    // Set fast PWM mode  (p.164)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);

    // Do non-inverting PWM on pin OC2A (p.162)
    // On the Arduino this is pin 11.
    TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0); //Table 18-3
    //TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0)); //Table 18-6
    // No prescaler (p.165)
    TCCR2B = (TCCR2B & ~(_BV(CS22) | _BV(CS21))) | _BV(CS20); //Table18-9

    // Set initial pulse width to the first sample.
    OCR2A = pgm_read_byte(&sound_data[0]);
    
    audCounter = 0;
    digitalWrite(audEnbl, LOW); //enable HT82V739 output

    // timer0 setting : interrupt for audio and action timing
    TCCR0A &= ~(_BV(WGM01) | _BV(WGM00));
    TIMSK0 |= _BV(OCIE0A);
    TIFR0 |= _BV(OCF0A);
    sei();
}
void rmUpdate();

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(9600);
  pinMode(audOut, OUTPUT);
  pinMode(audEnbl, OUTPUT);
  digitalWrite(audEnbl, HIGH);
  
  attachInterrupt(digitalPinToInterrupt(interruptPin), rmUpdate, CHANGE);
  pinMode(12, OUTPUT);
  
  myservo1.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo2.attach(10);  // attaches the servo on pin 9 to the servo object
  myservo1.write(90 + offset1);
  myservo2.write(90 + offset2);

  
  
  startPlayback();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(rmRecieved){ //リモコン受信した
    detachInterrupt(digitalPinToInterrupt(interruptPin));
    rmRecieved = 0;   //初期化
    rmState = 0;      //初期化
    //図とは左右が逆であることに注意
    customCode = rmCode;    //下16bitがcustomCode
    dataCode = rmCode >> 16;  //下16bitを捨てたあとの下8bitがdataCode
    invDataCode = rmCode >> 24; //下24bitを捨てたあとの下8bitがinvDataCode
    if((dataCode + invDataCode) == 0xff){   //反転確認
      //Serial.println(dataCode);
      switch(dataCode){  //<-switch文を追加
        case 24:  //"2"ボタン
          actionMode = FWRD;
        break;
        case 28:  //"5"ボタン
          actionMode = STOP;
        break;
        case 82:  //"8"ボタン
          actionMode = BWRD;
        break;
        case 12:  //"1"ボタン
          actionMode = LEFT;
        break;
        case 94:  //"3"ボタン
          actionMode = RIGHT;
        break;
        case 22:  //"0"ボタン
          startPlayback();
        break;
        default:
        break;
      }
    }
    attachInterrupt(digitalPinToInterrupt(interruptPin), rmUpdate, CHANGE);
  }
  switch(actionMode){
    case STOP:
    myservo1.write(90 + offset1);
    myservo2.write(90 + offset2);
    break;
    case FWRD:
    myservo1.write(80 + offset1);
    myservo2.write(100 + offset2);
    delay(100);
    myservo1.write(80 + offset1);
    myservo2.write(80 + offset2);  //左後ろ足を後方に。右後ろ足は浮いている。
    delay(200);
    myservo1.write(100 + offset1);
    myservo2.write(80 + offset2);
    delay(100);
    myservo1.write(100 + offset1);
    myservo2.write(100 + offset2);  //右後ろ足を後方に。左後ろ足は浮いている。
    delay(200);
    break;
    case BWRD:
    myservo1.write(100 + offset1);
    myservo2.write(100 + offset2);  //右後ろ足を後方に。
    delay(200);
    myservo1.write(100 + offset1);  //左前足を下げる。->左後ろ足が浮く。
    myservo2.write(80 + offset2);
    delay(100);
    myservo1.write(80 + offset1);
    myservo2.write(80 + offset2);  //左後ろ足を後方に。
    delay(200);
    myservo1.write(80 + offset1);  //右前足を下げる。->右後ろ足が浮く。
    myservo2.write(100 + offset2);
    delay(100);
    break;
    case LEFT:
    myservo1.write(80 + offset1);  //右前足を下げる。->右後ろ足が浮く。
    myservo2.write(90 + offset2);
    delay(100);
    myservo1.write(80 + offset1);
    myservo2.write(90 + offset2);  //左後ろ足動かない。
    delay(200);
    myservo1.write(100 + offset1);  //左前足を下げる。->左後ろ足が浮く。
    myservo2.write(80 + offset2);
    delay(100);
    myservo1.write(100 + offset1);
    myservo2.write(100 + offset2);  //右後ろ足を後方に。
    delay(200);
    break;
    case RIGHT:
    myservo1.write(80 + offset1);  //右前足を下げる。->右後ろ足が浮く。
    myservo2.write(100 + offset2);
    delay(100);
    myservo1.write(80 + offset1);
    myservo2.write(80 + offset2);  //左後ろ足を後方に。
    delay(200);
    myservo1.write(100 + offset1);  //左前足を下げる。->左後ろ足が浮く。
    myservo2.write(90 + offset2);
    delay(100);
    myservo1.write(100 + offset1);
    myservo2.write(90 + offset2);  //右後ろ足動かない。
    delay(200);
    break;
  }
}

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
    return;
    case 2: //9ms検出した
    if((width > 5000) || (width < 4000)){ //リーダーコード(4.5ms)ではない
      rmState = 0;
    }else{
      rmState = 3;  //OFF->ONで4.5ms検出
    }
    return;
    case 3: //4.5ms検出した
    if((width > 700) || (width < 400)){
      rmState = 0;
    }else{
      rmState = 4;  //ON->OFFで0.56ms検出した
    }
    return;
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
        rmRecieved = 1;
        return;
      }
      rmState = 3;  //次のON->OFFを待つ
    }
  }
}
