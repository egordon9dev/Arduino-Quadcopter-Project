#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"

RF24 radio(7, 8);

const uint64_t pipe = 0xF0F0F0F0E1LL;

int data[4];
int FL = 0, FR = 0, BL = 0, BR = 0;

void setup(void)
{
  Serial.begin(57600);

  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15, 15);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  //radio.setPayloadSize(8);

  radio.openReadingPipe(1, pipe);

  radio.startListening();

  radio.printDetails();
}

void loop(void)
{
  Serial.println("loop");
  double data[3];
  // if there is data ready
  if ( radio.available() )
  {
    radio.read( &data, sizeof(double[3]) );
    double throttle = data[0];
    double pitch = data[1];
    double roll = data[2];
    Serial.println("recieved...\tthrottle: " + String(throttle) + "\tpitch: " + String(pitch) + "\troll: " + String(roll));
  }
  
  delay(50);
}
