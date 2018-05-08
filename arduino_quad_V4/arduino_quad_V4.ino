#include "pid.h"
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "printf.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <Servo.h>

// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

Servo escfl, escfr, escbl, escbr;
#define LONG_MAX 2147483647
//---- roll ----
PidVars roll_pid = {
  .unwind = 0,  .doneZone = 0,  .maxIntegral = 500, .iActiveZone = 3,
  .target = 0,  .sensVal = 0,   .prevSensVal = 0,   .prevErr = 0,
  .errTot = 0,  .kp = 1.0,      .ki = 0.0,          .kd = 0.0,
  .deriv = 0,   .prevTime = 0,  .doneTime = LONG_MAX,  .prevDUpdateTime = 0
};
//---- pitch ----
PidVars pitch_pid  = {
  .unwind = 0,  .doneZone = 0,  .maxIntegral = 500, .iActiveZone = 3,
  .target = 0,  .sensVal = 0,   .prevSensVal = 0,   .prevErr = 0,
  .errTot = 0,  .kp = 1.0,      .ki = 0.0,          .kd = 0.0,
  .deriv = 0,   .prevTime = 0,  .doneTime = LONG_MAX,  .prevDUpdateTime = 0
};
//---- yaw ----
PidVars yaw_pid = {
  .unwind = 0,  .doneZone = 0,  .maxIntegral = 500, .iActiveZone = 3,
  .target = 0,  .sensVal = 0,   .prevSensVal = 0,   .prevErr = 0,
  .errTot = 0,  .kp = 1.0,      .ki = 0.0,          .kd = 0.0,
  .deriv = 0,   .prevTime = 0,  .doneTime = LONG_MAX,  .prevDUpdateTime = 0
};

#define LED_PIN 4
long prevRadioUpdateT = -LONG_MAX;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

RF24 radio(7, 8);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//ISR
void dmpDataReady() {
    mpuInterrupt = true;
}

const int MIN_MOTOR_POWER = 1148;
const int MAX_MOTOR_POWER = 1600;//1832;
int pitchCal, rollCal, yawCal;
void setup() {
    Serial.begin(9600);
    escfl.attach(9);
    escfr.attach(6);
    escbr.attach(5);
    escbl.attach(3);
    escfl.writeMicroseconds(MIN_MOTOR_POWER);
    escfr.writeMicroseconds(MIN_MOTOR_POWER);
    escbl.writeMicroseconds(MIN_MOTOR_POWER);
    escbr.writeMicroseconds(MIN_MOTOR_POWER);
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(931);
    mpu.setYGyroOffset(377);
    mpu.setZGyroOffset(762);
    mpu.setXAccelOffset(678);
    mpu.setYAccelOffset(358);
    mpu.setZAccelOffset(842);// 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    radio.begin();
    radio.setAutoAck(1);
    radio.enableAckPayload();
    radio.setRetries(0, 15);
    radio.setPayloadSize(10);
    radio.openReadingPipe(1, (byte*)("ctrlr"));
    radio.startListening();
    radio.setPALevel(RF24_PA_MAX);
    radio.setDataRate(RF24_250KBPS);
    radio.setCRCLength(RF24_CRC_8);
    radio.setChannel(75);
    radio.printDetails();
    //delay(30000);
}

uint16_t data[5] = {3, 3, 3, 3, 3};
double deltaTime = 0.0;
long time0 = millis()-5;
int throttle = MIN_MOTOR_POWER;
//double kpYaw = 0.7, kiYaw = 0.28, kdYaw = 0.6, targetYaw = 0.0, errorYaw = 0.0, totalErrorYaw = 0.0, prevErrorYaw = 0.0;
//double kpPitch = 3.4, kiPitch = 0.28, kdPitch = 2.0, targetPitch = 0.0, errorPitch = 0.0, totalErrorPitch = 0.0, prevErrorPitch = 0.0;
//double kpRoll = 3.4, kiRoll = 0.28, kdRoll = 2.0, targetRoll = 0.0, errorRoll = 0.0, totalErrorRoll = 0.0, prevErrorRoll = 0.0;
bool bCal = false;
void loop() {
    deltaTime = (double(millis()-time0))/1000.0;
    time0 = millis();
    
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {}
    
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    //check for interupt
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        radio.startListening();
        // if there is data ready
        if(radio.available()) {
          prevRadioUpdateT = millis();
          radio.read(&data, sizeof(uint16_t[5]));
          Serial.print("recieved:\t");
          for(int i = 0; i < 4; i++) {
            Serial.print(data[i]);
            Serial.print("\t");
          }
          Serial.print(data[4] >> 8);
          Serial.print("\t");
          Serial.print(data[4] & 0b11111111);
          Serial.print("\t");
          Serial.println();
          pitch_pid.kp = roll_pid.kp = data[0] / 1000.0;
          pitch_pid.ki = roll_pid.ki = data[1] / 100000.0;
          pitch_pid.kd = roll_pid.kd = data[2] / 100.0;
          throttle = MIN_MOTOR_POWER + data[3] / 10;
          pitch_pid.target = ((int)(data[4] >> 8) - 127) / 100.0;
          roll_pid.target = ((int)(data[4] & 0b11111111) - 127) / 100.0;
          yaw_pid.target = 0;
        }
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        if(!bCal) {
          pitchCal = ypr[1];
          rollCal = ypr[2];
          yawCal = ypr[0];
          bCal = true;
        }
        pitch_pid.sensVal = ypr[1];
        roll_pid.sensVal = ypr[2];
        yaw_pid.sensVal = ypr[0];
        int pitch = updatePID(&pitch_pid);
        int roll = updatePID(&roll_pid);
        int yaw = updatePID(&yaw_pid);
        int outFL = throttle - roll - pitch - yaw;
        int outFR = throttle + roll - pitch + yaw;
        int outBL = throttle - roll + pitch + yaw;
        int outBR = throttle + roll + pitch - yaw;
        if(outFL < MIN_MOTOR_POWER) outFL = MIN_MOTOR_POWER;
        if(outFL > MAX_MOTOR_POWER) outFL = MAX_MOTOR_POWER;
        if(outFR < MIN_MOTOR_POWER) outFR = MIN_MOTOR_POWER;
        if(outFR > MAX_MOTOR_POWER) outFR = MAX_MOTOR_POWER;
        if(outBL < MIN_MOTOR_POWER) outBL = MIN_MOTOR_POWER;
        if(outBL > MAX_MOTOR_POWER) outBL = MAX_MOTOR_POWER;
        if(outBR < MIN_MOTOR_POWER) outBR = MIN_MOTOR_POWER;
        if(outBR > MAX_MOTOR_POWER) outBR = MAX_MOTOR_POWER;
        escfl.writeMicroseconds(outFL);
        escfr.writeMicroseconds(outFR);
        escbl.writeMicroseconds(outBL);
        escbr.writeMicroseconds(outBR);
    }
    if((long)millis() - prevRadioUpdateT < 1000L) {
      digitalWrite(LED_PIN, (millis() / 50) % 10 == 0);
    }
}
