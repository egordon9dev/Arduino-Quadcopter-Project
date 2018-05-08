#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include "Nunchuk.h"

LiquidCrystal_I2C lcd(0x3f, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
RF24 radio(7, 8);

void setup(void)
{
  Serial.begin(9600);
  printf_begin();
  Wire.begin();
  nunchuk_init();
  lcd.begin(16,2);
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("init");
  radio.begin();
  radio.setAutoAck(1);
  radio.enableAckPayload();
  radio.setRetries(0, 15);
  radio.setPayloadSize(10);
  radio.openWritingPipe((byte*)("ctrlr"));
  radio.stopListening();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_8);
  radio.setChannel(75);
  radio.printDetails();
}
int8_t rollUnsc = 0, pitchUnsc = 0;
void loop(void) {
  if(nunchuk_read()) {
    rollUnsc = nunchuk_joystickX();
    pitchUnsc = nunchuk_joystickY();
  }
  int kpUnsc = analogRead(A0), kiUnsc = analogRead(A1),
  kdUnsc = analogRead(A2), thrUnsc = analogRead(A3);
    
  uint16_t data[5] = {kpUnsc, kiUnsc, kdUnsc, thrUnsc, ((pitchUnsc + 127) << 8) + rollUnsc + 127};
  radio.stopListening();
  radio.write(&data, sizeof(uint16_t[5]));
  Serial.print("sent:\t");
  for(int i = 0; i < 4; i++) {
    Serial.print(data[i]);
    Serial.print("\t");
  }
  Serial.print(data[4] >> 8);
  Serial.print("\t");
  Serial.print(data[4] & 0b11111111);
  Serial.print("\t");
  Serial.println();
  char s[15] = "";
  sprintf(s, "%4d %4d %4d", data[0], data[1], data[2]);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(s);
  delay(50);
}
