#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Pin definitions
const uint8_t ledPins[] = {4, 5, 6, 7, 8, 9};
const uint8_t NUM_LEDS = 6;

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Direction enumeration for better readability
enum Direction {
  LEFT = 0, RIGHT = 1, FORWARD = 2, BACKWARD = 3, UP = 4, DOWN = 5, NONE = 6
};

// Custom character sets - organized as 2D array for easier access
const byte customChars[6][6][8] PROGMEM = {
  // Set 1 - LEFT
  {
    {0b00000, 0b00000, 0b00000, 0b00001, 0b00011, 0b00111, 0b01111, 0b11111},
    {0b11111, 0b11111, 0b01111, 0b00111, 0b00011, 0b00001, 0b00000, 0b00000},
    {0b00000, 0b01000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000},
    {0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b11000, 0b01000},
    {0b00001, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000}
  },
  // Set 2 - RIGHT
  {
    {0b00000, 0b00000, 0b00000, 0b10000, 0b11000, 0b11100, 0b11110, 0b11111},
    {0b11111, 0b11111, 0b11110, 0b11100, 0b11000, 0b10000, 0b00000, 0b00000},
    {0b00000, 0b00010, 0b00011, 0b00011, 0b00011, 0b00011, 0b00011, 0b00011},
    {0b00011, 0b00011, 0b00011, 0b00011, 0b00011, 0b00011, 0b00011, 0b00010},
    {0b10000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000}
  },
  // Set 3 - BACKWARD
  {
    {0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b01111, 0b00111, 0b00011},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11111, 0b11111, 0b11111},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b11111, 0b11110, 0b11100, 0b11000},
    {0b00001, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b11111, 0b11111, 0b01110, 0b00100, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b10000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000}
  },
  // Set 4 - FORWARD
  {
    {0b00011, 0b00111, 0b01111, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b11111, 0b11111, 0b11111, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b11000, 0b11100, 0b11110, 0b11111, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00001},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00100, 0b01110, 0b11111, 0b11111},
    {0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b10000}
  },
  // Set 5 - UP
  {
    {0b00000, 0b00000, 0b00000, 0b00001, 0b00011, 0b00111, 0b01111, 0b11111},
    {0b00100, 0b01110, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111},
    {0b00000, 0b00000, 0b00000, 0b10000, 0b11000, 0b11100, 0b11110, 0b11111},
    {0b00000, 0b00000, 0b00000, 0b00001, 0b00011, 0b00111, 0b01111, 0b11111},
    {0b00100, 0b01110, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111},
    {0b00000, 0b00000, 0b00000, 0b10000, 0b11000, 0b11100, 0b11110, 0b11111}
  },
  // Set 6 - DOWN
  {
    {0b11111, 0b01111, 0b00111, 0b00011, 0b00001, 0b00000, 0b00000, 0b00000},
    {0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b01110, 0b00100},
    {0b11111, 0b11110, 0b11100, 0b11000, 0b10000, 0b00000, 0b00000, 0b00000},
    {0b11111, 0b01111, 0b00111, 0b00011, 0b00001, 0b00000, 0b00000, 0b00000},
    {0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b11111, 0b01110, 0b00100},
    {0b11111, 0b11110, 0b11100, 0b11000, 0b10000, 0b00000, 0b00000, 0b00000}
  }
};

// Character positions for each set
const uint8_t charPositions[6][6][2] PROGMEM = {
  {{2,0}, {3,0}, {3,1}, {1,1}, {2,1}, {0,0}}, // LEFT
  {{2,0}, {2,1}, {1,0}, {1,1}, {3,1}, {0,0}}, // RIGHT
  {{1,0}, {2,0}, {3,0}, {1,1}, {2,1}, {3,1}}, // BACKWARD
  {{1,1}, {2,1}, {3,1}, {1,0}, {2,0}, {3,0}}, // FORWARD
  {{1,0}, {2,0}, {3,0}, {1,1}, {2,1}, {3,1}}, // UP
  {{1,0}, {2,0}, {3,0}, {1,1}, {2,1}, {3,1}}  // DOWN
};

// Direction names and labels
const char* directionNames[] = {"Left", "Right", "Forward", "Backward", "Up", "Down"};
const char* speedLabels[] = {" Speed", " Vitesse", " Vitesse", " Vitesse", " Vitesse", " Vitesse"};

// MPU6050 object and variables
MPU6050 mpu;
bool DMPReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t FIFOBuffer[64];

Quaternion q;
VectorInt16 aa, aaWorld, prevAccel = {0, 0, 0};
VectorFloat gravity;

// Timing and threshold
unsigned long prevTime = 0;
const float MOVEMENT_THRESHOLD = 1.9f;
const float EARTH_GRAVITY_MS2 = 9.80665f;
const uint16_t LOOP_DELAY = 100;

void setup() {
  // Initialize I2C and Serial
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  
  // Initialize LCD
  lcd.init();
  lcd.backlight();
  
  // Initialize LED pins
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);
  }
  
  // Initialize MPU6050
  if (!initializeMPU()) {
    Serial.println(F("MPU6050 initialization failed!"));
    while (1);
  }
  
  prevTime = millis();
  Serial.println(F("System ready!"));
}

bool initializeMPU() {
  Serial.println(F("Initializing MPU6050..."));
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println(F("MPU6050 connection failed!"));
    return false;
  }
  
  devStatus = mpu.dmpInitialize();
  
  // Set calibration offsets
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  
  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    mpu.setDMPEnabled(true);
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println(F("DMP ready."));
    return true;
  } else {
    Serial.print(F("DMP initialization failed with code: "));
    Serial.println(devStatus);
    return false;
  }
}

void displayDirection(Direction dir, float value) {
  if (dir == NONE) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("No significant"));
    lcd.setCursor(0, 1);
    lcd.print(F("movement."));
    return;
  }
  
  // Load custom characters from PROGMEM
  for (uint8_t i = 0; i < 6; i++) {
    byte tempChar[8];
    for (uint8_t j = 0; j < 8; j++) {
      tempChar[j] = pgm_read_byte(&customChars[dir][i][j]);
    }
    lcd.createChar(i, tempChar);
  }
  
  lcd.clear();
  
  // Display characters at their positions
  for (uint8_t i = 0; i < 6; i++) {
    uint8_t x = pgm_read_byte(&charPositions[dir][i][0]);
    uint8_t y = pgm_read_byte(&charPositions[dir][i][1]);
    if (x < 16 && y < 2) {  // Bounds check
      lcd.setCursor(x, y);
      lcd.write(byte(i));
    }
  }
  
  // Display label and value
  lcd.setCursor(6, 0);
  lcd.print(speedLabels[dir]);
  lcd.setCursor(6, 1);
  lcd.print(value, 2);
  lcd.print(F(" m/s"));
}

void updateLEDs(Direction dir) {
  // Turn off all LEDs
  for (uint8_t i = 0; i < NUM_LEDS; i++) {
    digitalWrite(ledPins[i], LOW);
  }
  
  // Turn on the appropriate LED
  if (dir != NONE) {
    digitalWrite(ledPins[dir], HIGH);
  }
}

Direction detectDirection() {
  if (!mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    return NONE;
  }
  
  // Get motion data
  mpu.dmpGetQuaternion(&q, FIFOBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetAccel(&aa, FIFOBuffer);
  mpu.dmpConvertToWorldFrame(&aaWorld, &aa, &q);
  
  // Convert to m/s²
  float resolution = mpu.get_acce_resolution() * EARTH_GRAVITY_MS2;
  float ax = aaWorld.x * resolution;
  float ay = aaWorld.y * resolution;
  float az = aaWorld.z * resolution;
  
  float prevAx = prevAccel.x * resolution;
  float prevAy = prevAccel.y * resolution;
  float prevAz = prevAccel.z * resolution;
  
  // Calculate absolute differences
  float absDx = abs(ax - prevAx);
  float absDy = abs(ay - prevAy);
  float absDz = abs(az - prevAz);
  
  // Find dominant axis and direction
  Direction detectedDir = NONE;
  float maxAccel = 0;
  
  if (absDx > MOVEMENT_THRESHOLD && absDx > absDy && absDx > absDz) {
    detectedDir = (ax > prevAx) ? RIGHT : LEFT;
    maxAccel = absDx;
  } else if (absDy > MOVEMENT_THRESHOLD && absDy > absDx && absDy > absDz) {
    detectedDir = (ay > prevAy) ? FORWARD : BACKWARD;
    maxAccel = absDy;
  } else if (absDz > MOVEMENT_THRESHOLD && absDz > absDx && absDz > absDy) {
    detectedDir = (az > prevAz) ? DOWN : UP;
    maxAccel = absDz;
  }
  
  // Update display and LEDs
  displayDirection(detectedDir, maxAccel);
  updateLEDs(detectedDir);
  
  // Print to serial
  if (detectedDir != NONE) {
    Serial.print(directionNames[detectedDir]);
    Serial.print(F("\t\t"));
    Serial.print(maxAccel, 2);
    Serial.println(F(" m/s²"));
  }
  
  prevAccel = aaWorld;
  return detectedDir;
}

void loop() {
  if (!DMPReady) return;
  
  detectDirection();
  Serial.println();
  delay(LOOP_DELAY);
}
