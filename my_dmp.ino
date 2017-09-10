

#include <Servo.h>
#include <Wire.h>

float kpRoll = 2.0;
float kiRoll = 0.0;
float kdRoll = 0.0;

float kpPitch = 2.0;
float kiPitch = 0.0;
float kdPitch = 0.0;

float filtPitch = 0.0f, filtRoll = 0.0f, gyroPitch = 0.0f, gyroRoll = 0.0f, accelPitch = 0.0f, accelRoll = 0.0f;
float gx = 0.0f, gy = 0.0f, gz = 0.0f, ax = 0.0f, ay = 0.0f, az = 0.0f;
float gxCal = 0.0f, gyCal= 0.0f, gzCal = 0.0f;

float targetPitch = 0.0f, errorPitch = 0.0f, errorTotalPitch = 0.0f, prevErrorPitch = 0.0f;
float targetRoll = 0.0f, errorRoll = 0.0f, errorTotalRoll = 0.0f, prevErrorRoll = 0.0f;
long time0;
float deltaTime;

Servo escfl, escfr, escbl, escbr;

float GYRO_SENSITIVITY = 65.5;
float ACCEL_SENSITIVITY = 16384.0;

int ctr = 0;

void setupMPU() {
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); //power management index of register
  Wire.write(0b00000000); //turn off sleep mode
  Wire.endTransmission();
 
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); //Gyroscope congfiguration index of register
  Wire.write(0b00001000);
  Wire.endTransmission();
 
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); //Accelerometer congfiguration index of register
  Wire.write(0b00000000);
  Wire.endTransmission();
}

void calibrateGyro() {
    gxCal = 0;
    gyCal = 0;
    gzCal = 0;
    for(int i = 0; i < 15; i++) Serial.print(". ");
    Serial.print("| \n");
    for (int cal_int = 0; cal_int < 2000; cal_int++) {
      if(cal_int % 125 == 0) Serial.print(". ");
      recordRegisters();
      gxCal += gx;
      gyCal += gy;
      gzCal += gz;
      delay(5);
    }
    Serial.print("\n");
    gxCal = gxCal/2000.0;
    gyCal = gyCal/2000.0;
    gzCal = gzCal/2000.0;
}

void processData() {
  float tau = 0.0001;
  float alpha = (tau)/(tau+deltaTime);
  
  double rotX = ((double(gx-gxCal))/GYRO_SENSITIVITY);
  double rotY = ((double(gy-gyCal))/GYRO_SENSITIVITY);
  //double rotZ = ((double(gz-gzCal))/GYRO_SENSITIVITY)/LOOP_RATE;

  gyroPitch = rotX*deltaTime;
  gyroRoll = rotY*deltaTime;

  ax /= ACCEL_SENSITIVITY;
  ay /= ACCEL_SENSITIVITY;
  az /= ACCEL_SENSITIVITY;

  accelRoll = atan2(ay, az) * (180.0/PI);
  accelPitch =  atan2(ax, az) * (180.0/PI);
            
  if(ctr < 10) {
    filtRoll = 0.0;
    filtPitch = 0.0;
    ctr++;
  } else {
    // if !bullshit:      sum sides > hypotenuse
    float forceMagnitude = abs(ax) + abs(ay) + abs(az);
    if(true) //(forceMagnitude > 0.2 && forceMagnitude < 5.0)
    {
      //angle = (1-alpha)*(angle + gyro * dt) + (alpha)*(acc)
      filtRoll = (1-alpha) * (filtRoll + rotY*deltaTime) + (alpha * accelRoll);
      filtPitch = (1-alpha) * (filtPitch + rotX*deltaTime) + (alpha * accelPitch);
    }
  }
}
//0b1101000
void recordRegisters() {
  Wire.beginTransmission(0x68);
  Wire.write(0x43); //starting index of Gyro values in register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); // request indices 43-48 in register

 
  while(Wire.available() < 4/*6*/); //wait until at least 6 bytes fill the buffer
 
  gx = (Wire.read()<<8) | (Wire.read()); // first 2 bytes are the X gyro value
  gy = (Wire.read()<<8) | (Wire.read()); // second 2 bytes are the Y gyro value
  //gz = (Wire.read()<<8) | (Wire.read()); // third 2 bytes are the Z gyro value
  
  gx *= deltaTime;
  gy *= deltaTime;
  
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //staring index of Gyro values in register
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); // request indices 3B-40 in register

  while(Wire.available() < 6); //wait until at least 6 bytes fill the buffer
 
  /*
   * Wire.read() only returns 1 byte at a time
   * read first 8 bits (1 byte) and multpily by 256
   * second 8 bits (1 byte)
   * OR them together
   * a value (-32... to 32...)
   *
   * index of bytes: 3B  3C  3D  3E  3F  40
   * Order of bytes: (X,  X,  Y,  Y,  Z,  Z)
   */
  ax = (Wire.read()<<8) | (Wire.read()); // first 2 bytes are the X accel value
  ay = -(Wire.read()<<8) | (Wire.read()); // second 2 bytes are the Y accel value
  az = -(Wire.read()<<8) | (Wire.read()); // third 2 bytes are the Z accel value
}



const float INTEGRAL_ACTIVE_ZONE = 100.0;

float pid(float kp, float ki, float kd, float target, float &error, float &errorTotal, float &prevError, float sensVal)
{
	error = target - sensVal;
	errorTotal += error;
	float deltaError = error - prevError;

	//set components of pid
	float proportional = kp * error;
	float integral = ki * errorTotal;
	float derivative = kd * deltaError;

	// Prevent Integral from Getting Too High or Too Low
	if (integral > INTEGRAL_ACTIVE_ZONE) {
		integral = INTEGRAL_ACTIVE_ZONE;
	}
	if (integral < -INTEGRAL_ACTIVE_ZONE) {
		integral = -INTEGRAL_ACTIVE_ZONE;
	}

	prevError = error;

	return proportional + integral + derivative;
}

void printData() {
  Serial.print("fp: ");
  Serial.print(filtPitch);
  Serial.print("\tfr: ");
  Serial.print(filtRoll);
  Serial.print("ap: ");
  Serial.print(accelPitch);
  Serial.print("\tar: ");
  Serial.print(accelRoll);
  Serial.print("gp: ");
  Serial.print(gyroPitch);
  Serial.print("\tgr: ");
  Serial.println(gyroRoll);
}
void setup() {
  time0 = millis();
  escbr.attach(3);
  escbl.attach(5);
  escfl.attach(6);
  escfr.attach(10);
  escfl.writeMicroseconds(1000);
  escfr.writeMicroseconds(1000);
  escbl.writeMicroseconds(1000);
  escbr.writeMicroseconds(1000);
  Wire.begin();
  Serial.begin(115200);
  setupMPU();
  calibrateGyro();
}

void loop() {
  deltaTime = float(millis() - time0)/1000.0;
  time0 = millis();
  targetPitch = 0.0f;
  targetRoll = 0.0f;
  
  recordRegisters();
  processData();
  printData();
  
  float throttle = 1200;
  
  float pitchFactor = pid(kpPitch, kiPitch, kdPitch, targetPitch, errorPitch, errorTotalPitch, prevErrorPitch, filtPitch);
  float rollFactor = pid(kpRoll, kiRoll, kdRoll, targetRoll, errorRoll, errorTotalRoll, prevErrorRoll, filtRoll);
  
  int powerFL = throttle + rollFactor + pitchFactor;
  int powerFR = throttle - rollFactor + pitchFactor;
  int powerBL = throttle + rollFactor - pitchFactor;
  int powerBR = throttle - rollFactor - pitchFactor;
  
  if(powerFL > 1300) powerFL = 1300; else if(powerFL < 1000) powerFL = 1000;
  if(powerFR > 1300) powerFR = 1300; else if(powerFR < 1000) powerFR = 1000;
  if(powerBL > 1300) powerBL = 1300; else if(powerBL < 1000) powerBL = 1000;
  if(powerBR > 1300) powerBR = 1300; else if(powerBR < 1000) powerBR = 1000;

  
  escfl.writeMicroseconds(powerFL);
  escfr.writeMicroseconds(powerFR);
  escbl.writeMicroseconds(powerBL);
  escbr.writeMicroseconds(powerBR);
  //FL: 1050
  //FR: 1055
  //BL: 1249
  //BR: 1053
  
  delay(15);
}
