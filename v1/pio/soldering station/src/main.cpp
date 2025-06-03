#include <Arduino.h>

#include <Wire.h>
#include "U8glib.h"
#include <RotaryEncoder.h>

#define PINA 8//2
#define PINB 9//3
#define SW   4
#define PWM  6
#define bt1  2 //interrupt
#define bt2  3 //interrupt
#define reed 12 //Connects to GND when not in use

volatile bool rst0 = false;
volatile bool rst60 = false;
unsigned long lastDebounce0 = 0;
unsigned long lastDebounce60 = 0;
unsigned long long debounceDelay = 200;

RotaryEncoder encoder(PINA, PINB);
U8GLIB_SSD1306_128X64 u8g(U8G_I2C_OPT_NONE);  // I2C SSD1106/1306 128x64

int pwn = 0;

void draw(int duty,bool state) {
  char buf[10];
  int mos = 255-duty;
  char buf2[10];
  sprintf(buf2,"%d",mos);
  sprintf(buf, "%d", duty);
  
  u8g.setFont(u8g_font_ncenB14);
  u8g.drawStr(10,19,"DUT");  // Choose font
  u8g.drawStr(10, 40, buf);       // Display the integer
  u8g.drawStr(60,19,"MOS");
  u8g.drawStr(60,40,buf2);
  if(state){
    u8g.drawStr(32,63,"UP");
  }
  else{
    u8g.drawStr(32,63,"DOWN");
  }
  
}

void isrBt1(){
  if(millis() - lastDebounce0 > debounceDelay){
    rst0 = true;
    lastDebounce0 = millis();
  }
}
void isrBt2(){
  if(millis() - lastDebounce60 > debounceDelay){
    rst60 = true;
    lastDebounce60 = millis();
  }
}

void setup() {
  Serial.begin(9600);
  pinMode(SW, INPUT_PULLUP);
  pinMode(PWM, OUTPUT);
  pinMode(bt1,INPUT_PULLUP);
  pinMode(bt2,INPUT_PULLUP);
  pinMode(reed,INPUT_PULLUP);
  encoder.setPosition(0); // Start in middle
  attachInterrupt(digitalPinToInterrupt(bt1),isrBt1,FALLING);
  attachInterrupt(digitalPinToInterrupt(bt2),isrBt2,FALLING);
}

void loop() {
  encoder.tick();
  static int lastPos = -999;
  int newPos = encoder.getPosition();
  bool state = true;

  if(rst0){
    encoder.setPosition(0);
    rst0 =false;
  }

  if(rst60){
    encoder.setPosition(60);
    rst60 = false;
  }

  if (newPos < 0) {
    encoder.setPosition(0);
    newPos = 0;
  } else if (newPos > 255) {
    encoder.setPosition(255);
    newPos = 255;
  }

  if (digitalRead(SW) == LOW) {
    delay(50);
    if (digitalRead(SW) == LOW) {
      encoder.setPosition(127);
      newPos = 127;
    }
  }
  if(digitalRead(reed) == LOW){
    delay(100);
    state = false;
    encoder.setPosition(30);
  }
  else{
    state = true;
  }


  if (newPos != lastPos) {
    pwn = 255 - newPos;
    analogWrite(PWM, pwn);

    Serial.print("PWM: ");
    Serial.println(pwn);

    u8g.firstPage();
    do {
      draw(pwn,state);
    } while (u8g.nextPage());

    lastPos = newPos;
  }


}
