/*
* Name: Drone that can only rotate in one direction
* Author: Vidar Karman 
* Date: 2025-05-08
* Description: A drone project that uses a PD controller to adjust motor speed based on pitch angle.
*/

/* Include required libraries */
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* Create MPU6050 object and declare control/status variables */
MPU6050 mpu;
bool DMPReady = false;
uint8_t MPUIntStatus;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

/* Orientation and motion tracking variables */
Quaternion q;
VectorInt16 aa, gy, aaReal, aaWorld;
VectorFloat gravity;
float euler[3];
float ypr[3];

/* PD control variables */
int activemotorpin = 0;
float motorsignal = 0;
float pitcherror = 0;
float previouspitcherror = 0.0;
int currentTime = 0;
int previousTime = 0;
float deltattime = 0;
float newestreadings[50] = {}; // Reserved for future smoothing
float derivative = 0;

void setup() {
  // Initialize I2C and serial communication
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

  // Configure pin modes for motor output and input (if needed)
  pinMode(9, INPUT);
  pinMode(3, OUTPUT);
  pinMode(6, OUTPUT);

  // Initialize MPU6050 and configure DMP
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  devStatus = mpu.dmpInitialize();

  // Set sensor offsets (calibrated manually or by trial-and-error)
  mpu.setXAccelOffset(-1721);
  mpu.setYAccelOffset(-2802);
  mpu.setZAccelOffset(845);
  mpu.setXGyroOffset(423);
  mpu.setYGyroOffset(-51);
  mpu.setZGyroOffset(-10);

  // If initialization succeeded, enable DMP
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println("These are the Active offsets: ");
    mpu.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loop() {
  // Exit loop if DMP failed to initialize
  if (!DMPReady) return;

  // If a new DMP packet is available, process it
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    // Convert FIFO data to pitch angle in degrees
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    pitcherror = ypr[1] * 180 / M_PI;

    // Time calculation for derivative term
    currentTime = millis();
    deltattime = (currentTime - previousTime) / 1000.0;

    // PD controller calculation
    derivative = (pitcherror - previouspitcherror) / deltattime;
    float Kp = 2.0;
    float Kd = 1.0;
    motorsignal = Kp * pitcherror + Kd * derivative;

    // Determine motor based on pitch direction
    if (pitcherror <= 0.0) {
      activemotorpin = 6;
      motorsignal *= -1.0; // Invert signal if needed
    } else {
      activemotorpin = 3;
    }

    // Clamp motor signal to PWM range
    motorsignal = constrain(motorsignal, 0, 255);

    // Output PWM signal to the active motor
    analogWrite(activemotorpin, (int)motorsignal);

    // Save current state for next loop iteration
    previouspitcherror = pitcherror;
    previousTime = currentTime;

    // Debug output to serial monitor
    Serial.print(activemotorpin);
    Serial.print("    ");
    Serial.print(pitcherror);
    Serial.print("    ");
    Serial.println(motorsignal);
  }
}
