// ATMEGA328P_BOITE_NOIRE_MASTER_TO_CONTROL_STATION_SLAVE

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h> // Required for I2C communication

// Create MPU6050 object
MPU6050 mpu;

// Constants for I2C addresses
#define MPU6050_ADDRESS 0x68           // I2C address of the MPU6050 (slave)
#define CONTROL_STATION_ADDRESS 0x20   // I2C address of the control station ATMEGA328P (slave)
#define EARTH_GRAVITY_MS2 9.80665      // Earth's gravity in m/s²


// Structure to hold all MPU6050 data
struct MPUData {
  float ax, ay, az;           // World frame accelerations (m/s²)
  float gx, gy, gz;           // Gyroscope data (deg/s)
  float qw, qx, qy, qz;       // Quaternion components
  float pitch, roll, yaw;     // Euler angles (degrees)
  float temperature;          // Temperature (°C)
  byte movementCode;          // Movement direction code
  float dominantAccel;        // Dominant acceleration value
};

MPUData mpuData;

// Variables for the MPU6050's DMP (Digital Motion Processor)
bool DMPReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[32];

// Variables for MPU6050 data
Quaternion q;        // Quaternions for orientation
VectorInt16 aa;      // Raw accelerometer data
VectorInt16 aaWorld; // Gravity-corrected acceleration in world frame
VectorInt16 aaReal;  // Gravity-free accel sensor measurements
VectorInt16 gy;      // Gyro sensor measurements
VectorFloat gravity; // Gravity vector
float ypr[3];        // Yaw, pitch, roll container

// Variables for motion detection
VectorInt16 prevAccel = { 0, 0, 0 }; // Previous acceleration values for comparison
unsigned long prevTime = 0;         // Previous time for deltaTime calculation

// Movement detection threshold (in m/s²)
// Adjust this value to make detection more or less sensitive.
float movementThreshold = 1.0;

// Global variable to store the last detected movement
// This will be the value sent to the control station.
// Movement codes:
// 0: No movement (Idle)
// 1: Up
// 2: Down
// 3: Left
// 4: Right
// 5: Forward
// 6: Backward
byte lastMovementDetected = 0;

// ================================================================
// SETUP FUNCTION
// ================================================================
void setup() {
  // Initialize Wire as Master for both MPU6050 and Control Station communication
  Wire.begin(); // Initialize as I2C Master
  Wire.setClock(400000); // Use fast I2C speed (400kHz)

  // Initialize Serial for debugging (via USB-TTL if standalone ATMEGA)
  Serial.begin(115200);
  Serial.println(F("Initializing Black Box as I2C Master..."));
  Serial.println(F("Initializing MPU6050..."));

  // Initialize and test MPU6050 connection
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
    while (1); // Stop execution if connection fails
  }
  Serial.println("MPU6050 connected.");

  // Initialize the MPU6050 DMP
  devStatus = mpu.dmpInitialize();
  // Set default calibration offsets (adjust if needed)
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0) {
    // Auto-calibration (may take a few seconds)
    Serial.println(F("Calibrating MPU6050... Do not move."));
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets(); // Print offsets if you want to note them
    Serial.println(F("Calibration complete."));

    mpu.setDMPEnabled(true);
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("DMP ready."));
  } else {
    Serial.print("DMP initialization failed with code: ");
    Serial.println(devStatus);
    while (1); // Stop execution
  }

  // Initialize previous time for deltaTime calculation
  prevTime = millis();
  
  Serial.println(F("Black Box Master ready. Searching for Control Station..."));
}

// ================================================================
// LOOP FUNCTION
// ================================================================
void loop() {
  if (!DMPReady) return; // Make sure DMP is ready

  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) { // Check if a new packet is available in FIFO
    // Read current time and calculate deltaTime (for future use, not directly for detection here)
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0;
    prevTime = currentTime;

    // Retrieve DMP data
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetAccel(&aa, FIFOBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q); // Acceleration in world frame (gravity removed)
    mpu.dmpGetGyro(&gy, FIFOBuffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Fill MPU data structure
    // World frame accelerations in m/s²
    mpuData.ax = aaWorld.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    mpuData.ay = aaWorld.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    mpuData.az = aaWorld.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
    
    // Gyroscope data in degrees/second
    mpuData.gx = gy.x * mpu.get_gyro_resolution();
    mpuData.gy = gy.y * mpu.get_gyro_resolution();
    mpuData.gz = gy.z * mpu.get_gyro_resolution();
    
    // Quaternion components
    mpuData.qw = q.w;
    mpuData.qx = q.x;
    mpuData.qy = q.y;
    mpuData.qz = q.z;
    
    // Euler angles in degrees
    mpuData.yaw = ypr[0] * 180/M_PI;
    mpuData.pitch = ypr[1] * 180/M_PI;
    mpuData.roll = ypr[2] * 180/M_PI;
    
    // Temperature in Celsius
    mpuData.temperature = mpu.getTemperature() / 340.0 + 36.53;

    // Compute real accelerations in m/s²
    float ax = mpuData.ax;
    float ay = mpuData.ay;
    float az = mpuData.az;

    // Compute acceleration differences (acceleration derivative)
    float dx = ax - (prevAccel.x * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2);
    float dy = ay - (prevAccel.y * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2);
    float dz = az - (prevAccel.z * mpu.get_acce_resolution() * EARTH_GRAVITY_MS2);

    // Absolute values of differences
    float absDx = abs(dx);
    float absDy = abs(dy);
    float absDz = abs(dz);

    lastMovementDetected = 0; // No movement by default

    // ================================================================
    // MOVEMENT DETECTION LOGIC (6 DIRECTIONS)
    // ================================================================
    bool movementDetected = false;
    
    if (absDx > absDy && absDx > absDz && absDx > movementThreshold) {
      mpuData.dominantAccel = absDx;
      movementDetected = true;
      if (dx > 0) {
        // Movement "Right"
        mpuData.movementCode = 4; // Code for Right
        lastMovementDetected = 4;
        Serial.print("Right\t\t");
      } else {
        // Movement "Left"
        mpuData.movementCode = 3; // Code for Left
        lastMovementDetected = 3;
        Serial.print("Left\t\t");
      }
      Serial.print(absDx, 2);
      Serial.println(" m/s²");
    } else if (absDy > absDx && absDy > absDz && absDy > movementThreshold) {
      mpuData.dominantAccel = absDy;
      movementDetected = true;
      if (dy > 0) {
        // Movement "Forward" (Push)
        mpuData.movementCode = 5; // Code for Forward
        lastMovementDetected = 5;
        Serial.print("Forward (Push)\t");
      } else {
        // Movement "Backward" (Pull)
        mpuData.movementCode = 6; // Code for Backward
        lastMovementDetected = 6;
        Serial.print("Backward (Pull)\t");
      }
      Serial.print(absDy, 2);
      Serial.println(" m/s²");
    } else if (absDz > absDx && absDz > absDy && absDz > movementThreshold) {
      mpuData.dominantAccel = absDz;
      movementDetected = true;
      if (dz > 0) {
        // Movement "Down"
        mpuData.movementCode = 2; // Code for Down
        lastMovementDetected = 2;
        Serial.print("Down\t\t");
      } else {
        // Movement "Up"
        mpuData.movementCode = 1; // Code for Up
        lastMovementDetected = 1;
        Serial.print("Up\t\t");
      }
      Serial.print(absDz, 2);
      Serial.println(" m/s²");
    } else {
      // No significant movement
      mpuData.movementCode = 0; // Code for None
      mpuData.dominantAccel = 0.0;
      lastMovementDetected = 0;
      Serial.println("No significant movement.");
    }

    // Send data to control station whenever there's a change in movement
    // or periodically even if no movement (you can adjust this logic)
    sendDataToControlStation();

    // Update previous acceleration for next cycle
    prevAccel = aaWorld;

    Serial.println(); // Blank line for Serial Monitor readability
  }
  
  // Small delay to avoid overwhelming the I2C bus
  delay(100);
}


// Function to send complete MPU data to control station
void sendDataToControlStation() {
  // Send the complete MPU data structure
  Wire.beginTransmission(CONTROL_STATION_ADDRESS);
  Wire.write((byte*)&mpuData, sizeof(MPUData));
  byte result = Wire.endTransmission();
  
  if (result != 0) {
    Serial.print("I2C transmission error: ");
    Serial.println(result);
  }
}