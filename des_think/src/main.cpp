#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>
#include "ICM_42688.h"

//IMU: ICM42688 (SPI CS_42688 --> SPI3_NSS (PA15))
SPIClass spi3(PB4, PB3, PB5);  // MOSI, MISO, SCK for SPI3

#define IMU_CS PA15
#define SERVO_PITCH PA1
#define SERVO_ROLL PA2

// Servo objects
Servo servoPitch;
Servo servoRoll;

// Servo center positions
int pitchCenter = 90;
int rollCenter = 90;

// Servo angle limits mai roo tong tao rai
const int servoMin = 45;
const int servoMax = 135;
const float maxTiltAngle = 30.0;  // Max tilt angle in degrees

// IMU object
ICM_42688 imu;

// IMU data
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float pitch = 0, roll = 0;

// Timing
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 20;  // 50 Hz update rate

//=== Function Prototypes ===//
void imu_init();
void readIMU();
void updateServoAngles();
float mapAngleToServo(float angle, float minAngle, float maxAngle);

//=== IMU Initialization ===//
void imu_init() {
  Serial.println("Initializing IMU...");
  delay(100);
  
  // Begin SPI communication with IMU
  if (imu.begin(IMU_CS, spi3) != ICM_42688_Stat_Ok) {
    Serial.println("ERROR: IMU initialization failed!");
    while (1);  // Halt if IMU fails
  }
  
  // Configure IMU settings
  imu.setAccelFS_g(8);  // 8G range for accelerometer
  imu.setGyroFS_dps(500);  // 500 DPS range for gyroscope
  imu.setAccelODR(44.8);  // Sample rate in Hz
  imu.setGyroODR(44.8);
  
  Serial.println("IMU initialized successfully!");
}

//=== Read IMU Data ===//
void readIMU() {
  // Check if new data is available
  if (imu.getINT_status() & ICM_INT_DATA_RDY) {
    imu.getAGMT();  // Read accelerometer, gyroscope, magnetometer, temperature
    
    accelX = imu.accData[0];
    accelY = imu.accData[1];
    accelZ = imu.accData[2];
    
    gyroX = imu.gyrData[0];
    gyroY = imu.gyrData[1];
    gyroZ = imu.gyrData[2];
    
    // Calculate pitch and roll from accelerometer
    // Pitch = atan2(accelY, accelZ)
    // Roll = atan2(-accelX, accelZ)
    pitch = atan2(accelY, accelZ) * 180.0 / PI;
    roll = atan2(-accelX, accelZ) * 180.0 / PI;
  }
}

//=== Update Servo Angles ===//
void updateServoAngles() {
  // Map pitch angle (-30 to +30) to servo angle (45 to 135)
  int pitchServoAngle = mapAngleToServo(pitch, -maxTiltAngle, maxTiltAngle);
  
  // Map roll angle (-30 to +30) to servo angle (45 to 135)
  int rollServoAngle = mapAngleToServo(roll, -maxTiltAngle, maxTiltAngle);
  
  // Constrain to servo range
  pitchServoAngle = constrain(pitchServoAngle, servoMin, servoMax);
  rollServoAngle = constrain(rollServoAngle, servoMin, servoMax);
  
  // Write to servos
  servoPitch.write(pitchServoAngle);
  servoRoll.write(rollServoAngle);
}

//=== Map Angle to Servo Position ===//
float mapAngleToServo(float angle, float minAngle, float maxAngle) {
  // Map angle range to servo angle range (0-180)
  float mapped = map(angle * 100, minAngle * 100, maxAngle * 100, 
                     servoMin * 100, servoMax * 100);
  return mapped / 100.0;
}

//=== Setup ===//
void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  delay(500);
  
  Serial.println("Mini Helicopter Rotor Control System Starting...");
  
  // Initialize SPI3
  spi3.begin();
  
  // Set IMU CS pin
  pinMode(IMU_CS, OUTPUT);
  digitalWrite(IMU_CS, HIGH);
  
  // Initialize IMU
  imu_init();
  
  // Initialize servos
  servoPitch.attach(SERVO_PITCH);
  servoRoll.attach(SERVO_ROLL);
  
  // Set servos to center position
  servoPitch.write(pitchCenter);
  servoRoll.write(rollCenter);
  
  lastUpdateTime = millis();
  Serial.println("System ready!");
}

//=== Main Loop ===//
void loop() {
  unsigned long currentTime = millis();
  
  // Update at specified interval
  if (currentTime - lastUpdateTime >= updateInterval) {
    lastUpdateTime = currentTime;
    
    // Read IMU data
    readIMU();
    
    // Update servo positions based on IMU data
    updateServoAngles();
    
    // Optional: Print debug information
    Serial.print("Pitch: ");
    Serial.print(pitch, 2);
    Serial.print("° | Roll: ");
    Serial.print(roll, 2);
    Serial.println("°");
  }
}