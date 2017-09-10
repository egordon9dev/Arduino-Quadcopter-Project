#include <Servo.h>
//#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

Servo escfl, escfr, escbl, escbr;

#define LED_PIN 4
bool blinkState = true;

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

const uint64_t pipe = 0xF0F0F0F0E1LL;

int data[4];

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

const int MIN_MOTOR_POWER = 1148;
const int MAX_MOTOR_POWER = 1600;//1832;

void setup() {
    escfl.attach(9);
    escfr.attach(6);
    escbr.attach(5);
    escbl.attach(3);
    escfl.writeMicroseconds(MIN_MOTOR_POWER);
    escfr.writeMicroseconds(MIN_MOTOR_POWER);
    escbl.writeMicroseconds(MIN_MOTOR_POWER);
    escbr.writeMicroseconds(MIN_MOTOR_POWER);
  
    delay(3000);
  
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();

    devStatus = mpu.dmpInitialize();

    //accel: 678 353 842     gyro: 62 -41 41
    mpu.setXGyroOffset(62);
    mpu.setYGyroOffset(-41);
    mpu.setZGyroOffset(41);
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
    setupRadio();
}

const double INTEGRAL_ACTIVE_ZONE = 20.0;
double pid(double kp, double ki, double kd, double target, double &error, double &errorTotal, double &prevError, double sensVal, double dt)
{
  error = target - sensVal;
  errorTotal += error*dt;
  double rateError = (error - prevError)/(double)(dt);

  double proportional = kp * error;
  double integral = ki * errorTotal;
  double derivative = kd * rateError;
  //if error and derivative are the same sign
  // and derivative won't make div by 0 errors
  if((derivative > 0 && error > 0) || (derivative < 0 && error < 0)) {
    derivative = 0;
  }

  // Limit Integral to the Integral Active Zone
  if(integral > INTEGRAL_ACTIVE_ZONE) integral = INTEGRAL_ACTIVE_ZONE;
  if(integral < -INTEGRAL_ACTIVE_ZONE) integral = -INTEGRAL_ACTIVE_ZONE;

  prevError = error;

  return proportional + integral + derivative;
}

void setupRadio()
{
  radio.begin();
  radio.setRetries(15, 15);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  //radio.setPayloadSize(8);
  radio.openReadingPipe(1, pipe);
  radio.startListening();
  radio.printDetails();
  radio.setPALevel(RF24_PA_MAX);
}

int radioData[3];
void updateRadio()
{
  // if there is data ready
  while( radio.available() )
  {
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    radio.read( &radioData, sizeof(int[3]) );
  }
}

double deltaTime = 0.0;
long time0 = millis()-5;
int throttle = MIN_MOTOR_POWER;
double kpYaw = 0.7, kiYaw = 0.28, kdYaw = 0.6, targetYaw = 0.0, errorYaw = 0.0, totalErrorYaw = 0.0, prevErrorYaw = 0.0;
double kpPitch = 3.4, kiPitch = 0.28, kdPitch = 2.0, targetPitch = 0.0, errorPitch = 0.0, totalErrorPitch = 0.0, prevErrorPitch = 0.0;
double kpRoll = 3.4, kiRoll = 0.28, kdRoll = 2.0, targetRoll = 0.0, errorRoll = 0.0, totalErrorRoll = 0.0, prevErrorRoll = 0.0;

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

        updateRadio();
        throttle = radioData[0];
        targetPitch = radioData[1];
        targetRoll = radioData[2];
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        int yawFactor = pid(kpYaw, kiYaw, kdYaw, targetYaw, errorYaw, totalErrorYaw, prevErrorYaw, ypr[0] * (180.0/M_PI), deltaTime);
        int pitchFactor = pid(kpPitch, kiPitch, kdPitch, targetPitch, errorPitch, totalErrorPitch, prevErrorPitch, ypr[1] * (180.0/M_PI), deltaTime);
        int rollFactor = pid(kpRoll, kiRoll, kdRoll, targetRoll, errorRoll, totalErrorRoll, prevErrorRoll, ypr[2] * (180.0/M_PI), deltaTime);
        int outFL = throttle - rollFactor - pitchFactor - yawFactor;
        int outFR = throttle + rollFactor - pitchFactor + yawFactor;
        int outBL = throttle - rollFactor + pitchFactor + yawFactor;
        int outBR = throttle + rollFactor + pitchFactor - yawFactor;
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
}
