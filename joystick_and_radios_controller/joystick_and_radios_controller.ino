#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include "Nunchuk.h"
#include "RF24.h"
#include "nRF24L01.h"
#include "printf.h"

#define TRIM(i) (digitalRead(trim[i]))
#define PAYLOAD_SIZE 16
#define RADIO_SIGNATURE 197

LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
RF24 radio(7, 10);
int8_t trim[] = {4, 5, 6, 9};  // left, top, right, bottom
volatile bool prevTrim[] = {false, false, false, false};
volatile int16_t trimCtr[] = {-155, -46};
volatile bool radioOn = true, prevD8 = true;
volatile unsigned long debounceT[] = {0, 0, 0, 0, 0};
inline void enablePCI() { PCICR |= 0b00000101; }
inline void clearPCI() { PCICR &= 0b11111010; }
int limInt(int n, int min, int max) {
  if (n < min) return min;
  if (n > max) return max;
  return n;
}
uint16_t data[PAYLOAD_SIZE / 2];
void setup(void) {
  Serial.begin(115200);
  printf_begin();
  Wire.begin();
  nunchuk_init();
  for (int i = 0; i < 4; i++) { pinMode(trim[i], INPUT); }
  pinMode(8, INPUT_PULLUP);
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("init");
  radio.begin();
  radio.setAutoAck(0);
  radio.setRetries(15, 15);
  radio.setPayloadSize(PAYLOAD_SIZE);
  radio.openWritingPipe((byte*)("ctrlr"));
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_8);
  radio.setChannel(76);
  for (int i = 0; i < PAYLOAD_SIZE / 2; i++) { data[i] = 0; }
  //---- use interupts for trim buttons ----
  PCIFR |= 0b00000101;  // clear interupt flags
  PCMSK2 |= 0b01110000;  // enable pin change interupts on pins D4, D5, and D6
  PCMSK0 |= 0b00000011;  // enable pin change interupts on pin D8, D9
  enablePCI();  // enable pin change interupts
  printf("init\n");
}
#define DEBOUNCE_INTERVAL 80
ISR(PCINT0_vect) {  // D8 to D13
  if ((PINB & 0b00000010) && !prevTrim[3] && millis() - debounceT[3] > DEBOUNCE_INTERVAL) {  // D9
    trimCtr[1]--;
    prevTrim[3] = true;
    debounceT[3] = millis();
  } else if (!(PINB & 0b00000001) && prevD8 && millis() - debounceT[4] > 1000) {  // D8
    // radioOn = !radioOn;
    prevD8 = false;
    debounceT[4] = millis();
  }
}
ISR(PCINT2_vect) {  // D0 to D7
  if ((PIND & 0b00010000) && !prevTrim[0] && millis() - debounceT[0] > DEBOUNCE_INTERVAL) {  // D4
    trimCtr[0]--;
    prevTrim[0] = true;
    debounceT[0] = millis();
  } else if ((PIND & 0b00100000) && !prevTrim[1] && millis() - debounceT[1] > DEBOUNCE_INTERVAL) {  // D5
    trimCtr[1]++;
    prevTrim[1] = true;
    debounceT[1] = millis();
  } else if ((PIND & 0b01000000) && !prevTrim[2] && millis() - debounceT[2] > DEBOUNCE_INTERVAL) {  // D6
    trimCtr[0]++;
    prevTrim[2] = true;
    debounceT[2] = millis();
  }
}
int8_t rollUnsc = 0, pitchUnsc = 0, yawUnsc = 0;
long lastRadioT = 0, lastLCDT = 0;
uint8_t oldSREG;
void loop(void) {
  /*while (true) {
      printf("%d %d %d %d\n", digitalRead(trimL), digitalRead(trimT), digitalRead(trimR), digitalRead(trimB));
      delay(100);
  }*/
  if (millis() - lastRadioT > 10) {
    lastRadioT = millis();
    //----- atomic block ------
    clearPCI();
    if (nunchuk_read()) {
      rollUnsc = limInt(nunchuk_joystickX() + 1, -120, 120);
      pitchUnsc = limInt(nunchuk_joystickY(), -120, 120);
    }
    enablePCI();
    //-------------------------
    //----- atomic block ------
    oldSREG = SREG;
    cli();
    int kpUnsc = analogRead(A0), kiUnsc = analogRead(A1), kdUnsc = analogRead(A2), thrUnsc = analogRead(A3);
    yawUnsc = limInt(analogRead(A2) / 5 - 103, -127, 127);
    SREG = oldSREG;
    //-------------------------
    data[0] = kpUnsc;
    data[1] = kiUnsc;
    data[2] = kdUnsc;
    data[3] = thrUnsc;
    data[4] = ((pitchUnsc + 127) << 8) + rollUnsc + 127;
    data[5] = trimCtr[0];
    data[6] = trimCtr[1];
    data[7] = (RADIO_SIGNATURE << 8) + (uint8_t)yawUnsc;
    if (radioOn) {
      //----- atomic block ------
      clearPCI();
      radio.stopListening();
      radio.write(&data, PAYLOAD_SIZE);
      enablePCI();
      //-------------------------
    }
  }
  /*
   Serial.print("sent:\t");
   for (int i = 0; i < 4; i++) {
       Serial.print(data[i]);
       Serial.print("\t");
   }
   Serial.print(data[4] >> 8);
   Serial.print("\t");
   Serial.print(data[4] & 0b11111111);
   Serial.print("\t");
   Serial.println();*/
  if (millis() - lastLCDT > 100) {
    lastLCDT = millis();
    char s[15] = "";
    //----- atomic block -----
    oldSREG = SREG;
    cli();
    uint16_t temp[] = {data[0], data[1], data[2], data[3], data[4], data[5], data[6]};
    SREG = oldSREG;
    //------------------------
    sprintf(s, "%4d %4d %4d", temp[0], temp[1], temp[2]);
    //----- atomic block -----
    clearPCI();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(s);
    enablePCI();
    //-------------------------
    sprintf(s, "%4d %+4d %+4d %s", temp[3], (int16_t)trimCtr[0], (int16_t)trimCtr[1], radioOn ? "1" : "0");
    //----- atomic block -----
    clearPCI();
    lcd.setCursor(0, 1);
    lcd.print(s);
    enablePCI();
    //-------------------------
    printf("END LCD\n");
  }
  prevD8 = digitalRead(8);
  for (int i = 0; i < 4; i++) { prevTrim[i] = TRIM(i); }
  delayMicroseconds(500);
}
/*
200 106 293
*/