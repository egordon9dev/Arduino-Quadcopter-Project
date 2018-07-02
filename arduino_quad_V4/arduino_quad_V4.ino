#include <SPI.h>
#include <Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "RF24.h"
#include "Wire.h"
#include "nRF24L01.h"
#include "pid.h"
#include "printf.h"

// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

Servo escfl, escfr, escbl, escbr;
#define LONG_MAX 2147483647
//---- roll ----
PidVars roll_pid = {.unwind = 0.0, .doneZone = 0, .maxInteg = 150, .iActiveZone = 15 /*5 TURN THIS ON AGAIN*/, .target = 0, .sensVal = 0, .prevSensVal = 0, .integ = 0, .kp = 0, .ki = 0, .kd = 0, .deriv = 0, .minSignificantOutput = 25, .prevTime = 0, .doneTime = 0};
//---- pitch ----
PidVars pitch_pid = {.unwind = 0.0, .doneZone = 0, .maxInteg = 150, .iActiveZone = 15 /*5*/, .target = 0, .sensVal = 0, .prevSensVal = 0, .integ = 0, .kp = 0, .ki = 0, .kd = 0, .deriv = 0, .minSignificantOutput = 25, .prevTime = 0, .doneTime = 0};
//---- yaw ----
PidVars yaw_pid = {.unwind = 0.0, .doneZone = 0, .maxInteg = 50, .iActiveZone = 15 /*5*/, .target = 0, .sensVal = 0, .prevSensVal = 0, .integ = 0, .kp = 0, .ki = 0, .kd = 0, .deriv = 0, .minSignificantOutput = 9, .prevTime = 0, .doneTime = 0};
int radioLED = 4, pgmLED = 8;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;  // holds actual interrupt status byte from MPU
uint8_t devStatus;  // return status after each device operation (0 = success, !0
                    // = error)
uint16_t packetSize;  // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;  // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

Quaternion q;  // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float euler[3];  // [psi, theta, phi]    Euler angle container
float ypr[3];  // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

RF24 radio(7, 10);
#define PAYLOAD_SIZE 16
#define RADIO_SIGNATURE 197

volatile bool mpuInterrupt = false;  // indicates whether MPU interrupt pin has gone high
// ISR
void dmpDataReady() { mpuInterrupt = true; }

const int MIN_MOTOR_POWER = 1000;
const int MAX_MOTOR_POWER = 2000;  // 1650
const int dataLength = PAYLOAD_SIZE / 2;
uint16_t data[dataLength];
void setup() {
  Serial.begin(115200);
  printf_begin();
  Serial.print("init.");
  escfl.attach(9, MIN_MOTOR_POWER, MAX_MOTOR_POWER);
  escfr.attach(6, MIN_MOTOR_POWER, MAX_MOTOR_POWER);
  escbr.attach(5, MIN_MOTOR_POWER, MAX_MOTOR_POWER);
  escbl.attach(3, MIN_MOTOR_POWER, MAX_MOTOR_POWER);
  escfl.writeMicroseconds(MIN_MOTOR_POWER);
  escfr.writeMicroseconds(MIN_MOTOR_POWER);
  escbl.writeMicroseconds(MIN_MOTOR_POWER);
  escbr.writeMicroseconds(MIN_MOTOR_POWER);

  Serial.print(".");
  Wire.begin();
  TWBR = 12;  // 24;

  mpu.initialize();

  devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(-2932);
  mpu.setYAccelOffset(-1125);
  mpu.setZAccelOffset(1325);
  mpu.setXGyroOffset(117);
  mpu.setYGyroOffset(72);
  mpu.setZGyroOffset(-9);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  Serial.print(".");

  // configure LED for output
  pinMode(radioLED, OUTPUT);
  pinMode(pgmLED, OUTPUT);
  digitalWrite(radioLED, 0);
  digitalWrite(pgmLED, 0);
  radio.begin();
  Serial.print(".");
  radio.setAutoAck(0);
  radio.setRetries(15, 15);
  radio.setPayloadSize(PAYLOAD_SIZE);
  radio.openReadingPipe(1, (byte *)("ctrlr"));
  radio.startListening();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_250KBPS);
  radio.setCRCLength(RF24_CRC_8);
  radio.setChannel(76);
  Serial.println(" done");
  for (int i = 0; i < dataLength; i++) { data[i] = 0; }
  for (int i = 30; i > 0; i--) {
    Serial.println(i);
    delay(700);
  }
}
int limInt(int n, int min, int max) {
  if (n < min) return min;
  if (n > max) return max;
  return n;
}
double limDbl(double n, double min, double max) {
  if (n < min) return min;
  if (n > max) return max;
  return n;
}
int throttle = 0;
// double kpYaw = 0.7, kiYaw = 0.28, kdYaw = 0.6, targetYaw = 0.0, errorYaw =
// 0.0, totalErrorYaw = 0.0, prevErrorYaw = 0.0; double kpPitch = 3.4, kiPitch =
// 0.28, kdPitch = 2.0, targetPitch = 0.0, errorPitch = 0.0, totalErrorPitch =
// 0.0, prevErrorPitch = 0.0; double kpRoll = 3.4, kiRoll = 0.28, kdRoll = 2.0,
// targetRoll = 0.0, errorRoll = 0.0, totalErrorRoll = 0.0, prevErrorRoll = 0.0;
bool bCal = false;
int outFL = 0, outFR = 0, outBL = 0, outBR = 0;
double pitchCal = 0, rollCal = 0, yawCal = 0;
int calCtr = 0;
unsigned long lastRadioT = 0;
bool fail = false;
unsigned long numLoops = 0;
double pitch = 0, roll = 0, yaw = 0;
int8_t yawSc = 0;
unsigned long yawUpdateT = 0;
bool yawPidRunning = true;
void loop() {
  if (!dmpReady) return;

  radio.startListening();
  byte n;
  if (radio.available(&n)) {
    if (n == 1) {
      uint16_t temp[dataLength];
      radio.read(&temp, PAYLOAD_SIZE);
      if ((uint8_t)(temp[7] >> 8) == RADIO_SIGNATURE) {
        for (int i = 0; i < dataLength; i++) { data[i] = temp[i]; }
        lastRadioT = millis();
      } else {
        lastRadioT = 0;
      }
    }
  }
  // wait for MPU interrupt or extra packet(s) available
  if (mpuInterrupt || fifoCount >= packetSize) {
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
      while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
      mpu.getFIFOBytes(fifoBuffer, packetSize);
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      fifoCount -= packetSize;
      yaw_pid.kp = 1.0 * (roll_pid.kp = pitch_pid.kp = /*data[0]*/ 425.0 / 150.0);
      yaw_pid.ki = 1.0 * (roll_pid.ki = pitch_pid.ki = /*data[1]*/ 30.0 / 40000.0);
      yaw_pid.kd = 1.0 * (roll_pid.kd = pitch_pid.kd = /*data[2]*/ 400.0);
      throttle = data[3] * (975.0 / 1023.0) - 100;
      if (!bCal) {
        pitchCal += ypr[1];
        rollCal += ypr[2];
        yawCal += ypr[0];
        calCtr++;
        if (calCtr == 500) {
          bCal = true;
          pitchCal /= (double)calCtr;
          rollCal /= (double)calCtr;
          yawCal /= (double)calCtr;
        }
      } else {
        roll_pid.sensVal = (ypr[1] - pitchCal) * 180.0 / PI;
        pitch_pid.sensVal = (ypr[2] - rollCal) * 180.0 / PI;
        if (fabs(roll_pid.sensVal) > 60 || fabs(pitch_pid.sensVal) > 60) fail = true;
        yaw_pid.sensVal = (ypr[0] - yawCal) * 180.0 / PI;
        pitch_pid.target = ((int)(data[4] >> 8) - 127) / 8.0;
        roll_pid.target = ((int)(data[4] & 0b11111111) - 127) / 8.0;
        yawSc = (int8_t)(data[7] & 0xFF) / 3;
        if (yawSc != 0) {
          yawUpdateT = millis();
          yawPidRunning = false;
        }
        if (millis() - yawUpdateT > 50 && !yawPidRunning) {
          yawPidRunning = true;
          yaw_pid.target = yaw_pid.sensVal;
        }
        if (millis() - lastRadioT <= 250 && !fail) {
          pitch = updatePID(&pitch_pid);
          roll = updatePID(&roll_pid);
          yaw = yawPidRunning ? limDbl(updatePID(&yaw_pid), -100, 100) : -yawSc;
          outFL = throttle + roll - pitch - yaw - ((int16_t)data[6]) * 0.5 + 0.5;
          outFR = throttle - roll - pitch + yaw - ((int16_t)data[6]) * 0.5 - ((int16_t)data[5]) * 0.5 + 0.5;
          outBL = throttle + roll + pitch + yaw + 0.5;
          outBR = throttle - roll + pitch - yaw - ((int16_t)data[5]) * 0.5 + 0.5;
          escfl.writeMicroseconds(limInt(MIN_MOTOR_POWER + outFL, MIN_MOTOR_POWER, MAX_MOTOR_POWER));
          escfr.writeMicroseconds(limInt(MIN_MOTOR_POWER + outFR, MIN_MOTOR_POWER, MAX_MOTOR_POWER));
          escbl.writeMicroseconds(limInt(MIN_MOTOR_POWER + outBL, MIN_MOTOR_POWER, MAX_MOTOR_POWER));
          escbr.writeMicroseconds(limInt(MIN_MOTOR_POWER + outBR, MIN_MOTOR_POWER, MAX_MOTOR_POWER));
          /*Serial.print(escfl.readMicroseconds());
          Serial.print("  ");
          Serial.print(escfr.readMicroseconds());
          Serial.print("  ");
          Serial.print(escbr.readMicroseconds());
          Serial.print("  ");
          Serial.println(escbl.readMicroseconds());*/
        }
      } /*
          printf("pry %+05d/%+05d %+05d/%+05d %+05d/%+05d\n", (int)(100 * pitch_pid.sensVal), (int)(100 * pitch_pid.target), (int)(100 * roll_pid.sensVal), (int)(100 * roll_pid.target), (int)(100 * yaw_pid.sensVal), (int)(100 * yaw_pid.target));*/
    }
  }

  if (millis() - lastRadioT > 250 || !bCal || fail) {
    escfl.writeMicroseconds(MIN_MOTOR_POWER);
    escfr.writeMicroseconds(MIN_MOTOR_POWER);
    escbl.writeMicroseconds(MIN_MOTOR_POWER);
    escbr.writeMicroseconds(MIN_MOTOR_POWER);
  }
  if (millis() - lastRadioT < 250) { digitalWrite(radioLED, (millis() / 50) % 10 == 0); }
  digitalWrite(pgmLED, (millis() / 30) % 4 == 0);
}

/*
TODO----------------------








*/
