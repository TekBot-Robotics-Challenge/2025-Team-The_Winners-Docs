// ATMEGA328P_CONTROL_STATION_SLAVE_FROM_BLACK_BOX_MASTER

#include <Wire.h>
#include <LiquidCrystal.h>

// LCD Configuration in 4-bit mode (RS, Enable, D4, D5, D6, D7)
LiquidCrystal lcd(6, 7, 2, 3, 4, 5);

// Constants for I2C addresses
#define CONTROL_STATION_ADDRESS 0x20   // I2C address of this ATMEGA328P as slave

// Structure to hold all MPU6050 data (must match the one in master)
struct MPUData {
  float ax, ay, az;           // World frame accelerations (m/s²)
  float gx, gy, gz;           // Gyroscope data (deg/s)
  float qw, qx, qy, qz;       // Quaternion components
  float pitch, roll, yaw;     // Euler angles (degrees)
  float temperature;          // Temperature (°C)
  byte movementCode;          // Movement direction code
  float dominantAccel;        // Dominant acceleration value
};

// Variables to store received data
MPUData receivedData;
volatile bool newDataReceived = false;

// Buffer for I2C data reception
volatile byte dataBuffer[sizeof(MPUData)];
volatile byte bufferIndex = 0;

// Display mode variables
byte displayMode = 0;  // 0: Movement info, 1: Accelerations, 2: Gyro, 3: Angles, 4: Quaternions, 5: Temperature
unsigned long lastModeChange = 0;
const unsigned long MODE_DISPLAY_TIME = 3000; // 3 seconds per mode

// Movement names for display
const char* movementNames[] = {
  "IDLE",           // 0
  "UP",             // 1
  "DOWN",           // 2
  "LEFT",           // 3
  "RIGHT",          // 4
  "FORWARD",        // 5
  "BACKWARD"        // 6
};

// LEDs for Test 2 (Black Box) - Movement indication
const int led1 = 4;  // UP movement
const int led2 = 5;  // DOWN movement
const int led3 = 6;  // LEFT movement
const int led4 = 7;  // RIGHT movement
const int led5 = 8;  // FORWARD movement
const int led6 = 9;  // BACKWARD movement

// LED control variables
unsigned long lastLedUpdate = 0;
const unsigned long LED_UPDATE_INTERVAL = 100; // Update LEDs every 100ms
bool ledBlinkState = false;
unsigned long lastBlinkTime = 0;
const unsigned long BLINK_INTERVAL = 500; // Blink every 500ms for IDLE state

// ================================================================
// SETUP FUNCTION
// ================================================================
void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);
  Serial.println(F("Initializing Control Station as I2C Slave..."));

  // Set LED pins as output
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(led4, OUTPUT);
  pinMode(led5, OUTPUT);
  pinMode(led6, OUTPUT);
  
  // Initialize all LEDs to OFF
  turnOffAllLeds();
  
  // Initialize LCD
  lcd.begin(16, 2); // Set LCD dimensions (16 columns, 2 rows)
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Control Station");
  lcd.setCursor(0, 1);
  lcd.print("Waiting...");
  
  // Configure this ATMEGA as I2C slave
  Wire.begin(CONTROL_STATION_ADDRESS); // Initialize as I2C slave
  Wire.onReceive(receiveEvent);        // Register the callback function for data reception
  
  Serial.print(F("Control Station ready at I2C address: 0x"));
  Serial.println(CONTROL_STATION_ADDRESS, HEX);
  
  delay(2000); // Show startup message for 2 seconds
}

// ================================================================
// LOOP FUNCTION
// ================================================================
void loop() {
  // Check if new data has been received from the black box
  if (newDataReceived) {
    newDataReceived = false;
    
    // Print all data to Serial for debugging
    printAllDataToSerial();
    
    // Update LED status based on movement
    updateLedStatus();
  }
  
  // Handle LED blinking for IDLE state
  handleLedBlinking();
  
  // Auto-cycle through display modes
  if (millis() - lastModeChange > MODE_DISPLAY_TIME) {
    displayMode = (displayMode + 1) % 6; // Cycle through 6 modes
    lastModeChange = millis();
  }
  
  // Update LCD display based on current mode
  updateLCDDisplay();
  
  // Small delay to avoid overwhelming the display
  delay(50);
}

// ================================================================
// LED CONTROL FUNCTIONS
// ================================================================

// Function to turn off all LEDs
void turnOffAllLeds() {
  digitalWrite(led1, LOW);
  digitalWrite(led2, LOW);
  digitalWrite(led3, LOW);
  digitalWrite(led4, LOW);
  digitalWrite(led5, LOW);
  digitalWrite(led6, LOW);
}

// Function to update LED status based on movement code
void updateLedStatus() {
  // First turn off all LEDs
  turnOffAllLeds();
  
  // Then activate the appropriate LED based on movement code
  switch (receivedData.movementCode) {
    case 0: // IDLE - All LEDs will blink (handled in handleLedBlinking)
      // Don't turn on any LED here, blinking is handled separately
      break;
      
    case 1: // UP
      digitalWrite(led1, HIGH);
      Serial.println("LED1 (UP) activated");
      break;
      
    case 2: // DOWN
      digitalWrite(led2, HIGH);
      Serial.println("LED2 (DOWN) activated");
      break;
      
    case 3: // LEFT
      digitalWrite(led3, HIGH);
      Serial.println("LED3 (LEFT) activated");
      break;
      
    case 4: // RIGHT
      digitalWrite(led4, HIGH);
      Serial.println("LED4 (RIGHT) activated");
      break;
      
    case 5: // FORWARD
      digitalWrite(led5, HIGH);
      Serial.println("LED5 (FORWARD) activated");
      break;
      
    case 6: // BACKWARD
      digitalWrite(led6, HIGH);
      Serial.println("LED6 (BACKWARD) activated");
      break;
      
    default: // Unknown movement code
      // Blink all LEDs rapidly to indicate error
      blinkAllLedsError();
      Serial.print("Unknown movement code: ");
      Serial.println(receivedData.movementCode);
      break;
  }
}

// Function to handle LED blinking for IDLE state
void handleLedBlinking() {
  if (receivedData.movementCode == 0) { // IDLE state
    if (millis() - lastBlinkTime > BLINK_INTERVAL) {
      ledBlinkState = !ledBlinkState;
      lastBlinkTime = millis();
      
      // Blink all LEDs together for IDLE indication
      digitalWrite(led1, ledBlinkState);
      digitalWrite(led2, ledBlinkState);
      digitalWrite(led3, ledBlinkState);
      digitalWrite(led4, ledBlinkState);
      digitalWrite(led5, ledBlinkState);
      digitalWrite(led6, ledBlinkState);
    }
  }
}

// Function to blink all LEDs rapidly for error indication
void blinkAllLedsError() {
  static unsigned long lastErrorBlink = 0;
  static bool errorBlinkState = false;
  const unsigned long ERROR_BLINK_INTERVAL = 150; // Fast blink for error
  
  if (millis() - lastErrorBlink > ERROR_BLINK_INTERVAL) {
    errorBlinkState = !errorBlinkState;
    lastErrorBlink = millis();
    
    digitalWrite(led1, errorBlinkState);
    digitalWrite(led2, errorBlinkState);
    digitalWrite(led3, errorBlinkState);
    digitalWrite(led4, errorBlinkState);
    digitalWrite(led5, errorBlinkState);
    digitalWrite(led6, errorBlinkState);
  }
}

// Function to create a LED pattern based on acceleration intensity
void updateLedIntensityPattern() {
  float accelMagnitude = sqrt(receivedData.ax * receivedData.ax + 
                             receivedData.ay * receivedData.ay + 
                             receivedData.az * receivedData.az);
  
  // Determine how many LEDs to light based on acceleration magnitude
  int numLedsToLight = 0;
  if (accelMagnitude > 15.0) numLedsToLight = 6;      // Very high acceleration
  else if (accelMagnitude > 12.0) numLedsToLight = 5; // High acceleration
  else if (accelMagnitude > 9.0) numLedsToLight = 4;  // Medium-high acceleration
  else if (accelMagnitude > 6.0) numLedsToLight = 3;  // Medium acceleration
  else if (accelMagnitude > 3.0) numLedsToLight = 2;  // Low-medium acceleration
  else if (accelMagnitude > 1.0) numLedsToLight = 1;  // Low acceleration
  else numLedsToLight = 0;                             // Very low acceleration
  
  // Light up LEDs in sequence based on intensity
  digitalWrite(led1, numLedsToLight >= 1 ? HIGH : LOW);
  digitalWrite(led2, numLedsToLight >= 2 ? HIGH : LOW);
  digitalWrite(led3, numLedsToLight >= 3 ? HIGH : LOW);
  digitalWrite(led4, numLedsToLight >= 4 ? HIGH : LOW);
  digitalWrite(led5, numLedsToLight >= 5 ? HIGH : LOW);
  digitalWrite(led6, numLedsToLight >= 6 ? HIGH : LOW);
}

// ================================================================
// DISPLAY AND DEBUG FUNCTIONS
// ================================================================

// Function to print all data to Serial
void printAllDataToSerial() {
  Serial.println("=== MPU6050 Data ===");
  Serial.print("Movement: ");
  if (receivedData.movementCode <= 6) {
    Serial.print(movementNames[receivedData.movementCode]);
  } else {
    Serial.print("UNKNOWN");
  }
  Serial.print(" (Code: ");
  Serial.print(receivedData.movementCode);
  Serial.println(")");
  
  Serial.print("Dominant Accel: ");
  Serial.print(receivedData.dominantAccel, 2);
  Serial.println(" m/s²");
  
  Serial.print("Accelerations - X: ");
  Serial.print(receivedData.ax, 2);
  Serial.print(", Y: ");
  Serial.print(receivedData.ay, 2);
  Serial.print(", Z: ");
  Serial.print(receivedData.az, 2);
  Serial.println(" m/s²");
  
  Serial.print("Gyroscope - X: ");
  Serial.print(receivedData.gx, 2);
  Serial.print(", Y: ");
  Serial.print(receivedData.gy, 2);
  Serial.print(", Z: ");
  Serial.print(receivedData.gz, 2);
  Serial.println(" deg/s");
  
  Serial.print("Euler Angles - Yaw: ");
  Serial.print(receivedData.yaw, 1);
  Serial.print(", Pitch: ");
  Serial.print(receivedData.pitch, 1);
  Serial.print(", Roll: ");
  Serial.print(receivedData.roll, 1);
  Serial.println(" deg");
  
  Serial.print("Quaternion - W: ");
  Serial.print(receivedData.qw, 3);
  Serial.print(", X: ");
  Serial.print(receivedData.qx, 3);
  Serial.print(", Y: ");
  Serial.print(receivedData.qy, 3);
  Serial.print(", Z: ");
  Serial.print(receivedData.qz, 3);
  Serial.println();
  
  Serial.print("Temperature: ");
  Serial.print(receivedData.temperature, 1);
  Serial.println(" °C");
  Serial.println();
}

// Function to update LCD display based on current mode
void updateLCDDisplay() {
  lcd.clear();
  
  switch (displayMode) {
    case 0: // Movement information
      lcd.setCursor(0, 0);
      lcd.print("Move: ");
      if (receivedData.movementCode <= 6) {
        lcd.print(movementNames[receivedData.movementCode]);
      } else {
        lcd.print("UNKNOWN");
      }
      
      lcd.setCursor(0, 1);
      if (receivedData.movementCode == 0) {
        lcd.print("Accel: 0.00 m/s2");
      } else {
        lcd.print("Accel: ");
        lcd.print(receivedData.dominantAccel, 2);
        lcd.print(" m/s2");
      }
      break;
      
    case 1: // Accelerations
      lcd.setCursor(0, 0);
      lcd.print("Accel X,Y,Z m/s2");
      lcd.setCursor(0, 1);
      lcd.print(receivedData.ax, 1);
      lcd.print(",");
      lcd.print(receivedData.ay, 1);
      lcd.print(",");
      lcd.print(receivedData.az, 1);
      break;
      
    case 2: // Gyroscope
      lcd.setCursor(0, 0);
      lcd.print("Gyro X,Y,Z deg/s");
      lcd.setCursor(0, 1);
      lcd.print(receivedData.gx, 1);
      lcd.print(",");
      lcd.print(receivedData.gy, 1);
      lcd.print(",");
      lcd.print(receivedData.gz, 1);
      break;
      
    case 3: // Euler Angles
      lcd.setCursor(0, 0);
      lcd.print("Y,P,R degrees");
      lcd.setCursor(0, 1);
      lcd.print(receivedData.yaw, 0);
      lcd.print(",");
      lcd.print(receivedData.pitch, 0);
      lcd.print(",");
      lcd.print(receivedData.roll, 0);
      break;
      
    case 4: // Quaternions
      lcd.setCursor(0, 0);
      lcd.print("Quat W,X,Y,Z");
      lcd.setCursor(0, 1);
      lcd.print(receivedData.qw, 2);
      lcd.print(",");
      lcd.print(receivedData.qx, 2);
      lcd.print(",");
      lcd.print(receivedData.qy, 1);
      break;
      
    case 5: // Temperature
      lcd.setCursor(0, 0);
      lcd.print("Temperature");
      lcd.setCursor(0, 1);
      lcd.print(receivedData.temperature, 1);
      lcd.print(" Celsius");
      break;
  }
}

// Optional: Add a function to display connection status
void displayConnectionStatus(bool connected) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Black Box:");
  lcd.setCursor(0, 1);
  if (connected) {
    lcd.print("CONNECTED");
  } else {
    lcd.print("DISCONNECTED");
  }
}

// ================================================================
// I2C COMMUNICATION
// ================================================================

// I2C RECEIVE CALLBACK FUNCTION (WHEN THE MASTER SENDS DATA)
void receiveEvent(int bytesReceived) {
  bufferIndex = 0;
  
  // Read all available bytes
  while (Wire.available() && bufferIndex < sizeof(dataBuffer)) {
    dataBuffer[bufferIndex] = Wire.read();
    bufferIndex++;
  }
  
  // If we received the expected amount of data
  if (bufferIndex >= sizeof(MPUData)) {
    // Copy the received bytes into the MPU data structure
    memcpy((void*)&receivedData, (void*)dataBuffer, sizeof(MPUData));
    
    // Set flag that new data is available
    newDataReceived = true;
  }
}