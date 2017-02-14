#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

RF24 radio(9, 10);

const uint64_t pipe = 0xF0F0F0F0E1LL;

void setup(void)
{
  Serial.begin(57600);
  printf_begin();

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15, 15);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  //radio.setPayloadSize(8);

  radio.openWritingPipe(pipe);

  radio.startListening();

  radio.printDetails();
}

double myMap(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop(void) {
  int pot = 1023 - analogRead(A0);
  int joyX = analogRead(A1);
  int joyY = 1023 - analogRead(A2);
  //Serial.println("pot: " + String(pot) + "\t" + "joy x: " + String(joyX) + "joy y: " + String(joyY));

  double throttle = myMap(pot, 0.0, 1023.0, 0.0, 500.0);
  double roll = myMap(joyX, 0, 1023, -20.0, 20.0);
  double pitch = myMap(joyY, 0, 1023, -20.0, 20.0);

  if(roll < 2 && roll > -2) roll = 0;
  if(pitch < 2 && pitch > -2) pitch = 0;

  Serial.println("sending... throttle: " + String(throttle) + "\pitch: " + String(pitch) + "\troll: " + String(roll));
  
  double data[3] = {throttle, pitch, roll};
  // First, stop listening so we can talk
  radio.stopListening();
  // Send the final one back.
  radio.write( &data, sizeof(double[3]) );

  // Now, resume listening so we catch the next packets.
  radio.startListening();

  delay(50);
}
