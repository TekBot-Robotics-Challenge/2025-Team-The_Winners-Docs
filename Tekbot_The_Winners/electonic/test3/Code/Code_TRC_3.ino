#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// PWM range for servos (adjust according to your model)
#define SERVO_MIN 100   // 0° position
#define SERVO_MAX 500   // 180° position

int angleOn = 90;   // Angle to activate a segment
int angleOff = 0;   // Angle to deactivate a segment

// PCA9685 channels used (0 to 6 for the 7 segments A to G)
const int segments[7] = {0, 1, 2, 3, 4, 5, 6};
// LED channels (8 to 14 for the 7 LED trios)
/*LED_segment 1 -- 14
  LED_segment 2 -- 15
  LED_segment 3 -- 9
  LED_segment 4 -- 11
  LED_segment 5 -- 13
  LED_segment 6 -- 12
  LED_segment 7 -- 10
*/

const int ledChannels[7] = {14,15,9,11,13,12,10};

// Battery level LEDs on PCA9685 channels
const int batteryLEDs[4] = {12, 9, 10, 11};

// Digit table for servos 0–11 (1 = active segment)
const bool servos[12][7] = {
  {1,0,0,1,0,1,1},
  {0,1,1,0,1,0,1}, // 0
  {1,1,1,1,0,1,1}, // 1
  {0,1,0,0,1,1,0}, // 2
  {0,1,1,0,0,1,0}, // 3
  {1,1,1,1,0,0,0}, // 4
  {0,0,1,0,0,0,0}, // 5
  {0,0,1,0,1,0,0}, // 6
  {0,1,1,1,0,1,1}, // 7
  {0,1,1,0,1,0,0}, // 8
  {0,1,1,0,0,0,0}, // 9
  {1,0,0,1,0,1,1}
};



// LED table for digits 0–11 (1 = LED on)
const bool leds[12][7] = {
  {0,0,0,0,0,0,0},
  {1,1,1,1,1,1,0}, // 0
  {0,1,1,0,0,0,0}, // 1
  {1,1,0,1,1,0,1}, // 2
  {1,1,1,1,0,0,1}, // 3
  {0,1,1,0,0,1,1}, // 4
  {1,0,1,1,0,1,1}, // 5
  {1,0,1,1,1,1,1}, // 6
  {1,1,1,0,0,0,0}, // 7
  {1,1,1,1,1,1,1}, // 8
  {1,1,1,1,0,1,1}, // 9
  {0,0,0,0,0,0,0}
};

// Timing without delay()
unsigned long previousMillis = 0;
const long interval = 1000; // 1 seconds
int currentDigit = 0;
int direction = 1; // +1 = up ; -1 = down

// LED fade variables
unsigned long fadeStartTime = 0;
const long fadeDuration = 1000; // 1 second fade
bool fadeActive = false;
bool previousSegmentState[7] = {false}; // Previous state of each segment
bool currentSegmentState[7] = {false};  // Current state of each segment

// Battery monitoring variables
const int batteryPin = A0;  // ADC0 pin
const float maxVoltage = 8.4;  // Maximum voltage (100%)
// const int ratio = 1.7142 ; // Resistor ratio 
unsigned long lastBatteryCheck = 0;
const long batteryCheckInterval = 500; // Check battery every 500ms

void setup() {
  pwm.begin();
  pwm.setPWMFreq(50); // PWM frequency for servos

  for (int i = 0; i< 4; i++){
    pinMode(batteryLEDs[i], OUTPUT);
    digitalWrite(batteryLEDs[i], LOW);
  }

  // Deactivate all segments at startup
  // for (int i = 0; i < 7; i++) {
  //   setServoAngle(segments[i], digits[0][i]);
  // }
  
  // Initialize segment states
  for (int i = 0; i < 7; i++) {
    currentSegmentState[i] = leds[currentDigit][i];
    previousSegmentState[i] = currentSegmentState[i];
  }
  
  displayDigit(currentDigit);
  currentDigit +=1;
  Serial.begin(9600);
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Handle digit change
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Store previous segment states
    for (int i = 0; i < 7; i++) {
      previousSegmentState[i] = currentSegmentState[i];
      currentSegmentState[i] = leds[currentDigit][i];
    }

    // Display current digit (servos)
    displayDigit(currentDigit);
    
    // Start LED fade
    fadeStartTime = currentMillis;
    fadeActive = true;

    // Prepare next digit
    currentDigit += direction;
    if (currentDigit > 11) {
      currentDigit = 11;
      direction = -1;
    } else if (currentDigit < 0) {
      currentDigit = 0;
      direction = 1;
    }
  }
  
  // Handle LED fade
  if (fadeActive) {
    updateLEDFade(currentMillis);
  }
  
  // Check battery level
  if (currentMillis - lastBatteryCheck >= batteryCheckInterval) {
    lastBatteryCheck = currentMillis;
    updateBatteryLevel();
  }
}

// Function to display a digit between 0 and 11
void displayDigit(int number) {
  for (int i = 0; i < 7; i++) {
    if (servos[number][i]) {
      setServoAngle(segments[i], angleOn);
    } else {
      setServoAngle(segments[i], angleOff);
    }
  }
}

// Convert angle (°) to PWM signal for PCA9685
void setServoAngle(int channel, int angle) {
  int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
  pwm.setPWM(channel, 0, pulse);
}

// Update LED fade animation
void updateLEDFade(unsigned long currentMillis) {
  unsigned long elapsed = currentMillis - fadeStartTime;
  
  if (elapsed >= fadeDuration) {
    // Fade complete
    fadeActive = false;
    // Set final LED states
    for (int i = 0; i < 7; i++) {
      if (currentSegmentState[i]) {
        pwm.setPWM(ledChannels[i], 0, 4095); // Full brightness
      } else {
        pwm.setPWM(ledChannels[i], 0, 0);    // Off
      }
    }
  } else {
    // Calculate fade progress (0.0 to 1.0)
    float progress = (float)elapsed / fadeDuration;
    
    for (int i = 0; i < 7; i++) {
      int brightness = 0;
      
      if (currentSegmentState[i] && !previousSegmentState[i]) {
        // Fade IN: 0 to 4095
        brightness = (int)(4095 * progress);
      } else if (!currentSegmentState[i] && previousSegmentState[i]) {
        // Fade OUT: 4095 to 0
        brightness = (int)(4095 * (1.0 - progress));
      } else if (currentSegmentState[i] && previousSegmentState[i]) {
        // Stay ON
        brightness = 4095;
      } else {
        // Stay OFF
        brightness = 0;
      }
      
      pwm.setPWM(ledChannels[i], 0, brightness);
    }
  }
}

// Update battery level indicator
void updateBatteryLevel() {
  // Read ADC value (0-1023)
  int adcValue = analogRead(batteryPin);
  
  // Convert to voltage (0-5V reference)
  float voltage = (adcValue / 1023.0) * 4.9 * 1.898;
        // voltage = 1.7142 * voltage ;
  Serial.print("Voltage = "); Serial.println(voltage);
  // Calculate battery percentage based on 4.9V max
  float batteryPercent = (voltage / maxVoltage) * 100.0;
  Serial.print("Battery level = "); Serial.println(batteryPercent);
  for (int i = 0; i < 2; i++){
      digitalWrite(batteryLEDs[i], LOW);
    }
  // Constrain to 0-100%
  if (batteryPercent > 100.0) batteryPercent = 100.0;
  if (batteryPercent < 0.0) batteryPercent = 0.0;
  
  // Light up LEDs based on battery level
  if ((batteryPercent >= 75.0) && (batteryPercent <= 100.0)) {
    // 100-75%: Light up LEDs on channels 9, 10, 11, 12
    for (int i = 0; i < 4; i++){
      digitalWrite(batteryLEDs[i], HIGH);
    }
  } 
  else if ((batteryPercent >= 50.0) && (batteryPercent < 75.0)) {
    // 75-50%: Light up LEDs on channels 10, 11, 12
     for (int i = 0; i < 3; i++){
      digitalWrite(batteryLEDs[i], HIGH);
    }
  } 
  else if ((batteryPercent >= 25.0) && (batteryPercent < 50.0)) {
    // 50-25%: Light up LEDs on channels 11, 12
     for (int i = 0; i < 2; i++){
      digitalWrite(batteryLEDs[i], HIGH);
    }
  } 
  else if ((batteryPercent > 0.0) && (batteryPercent < 25.0)) {
    // 50-25%: Light up LEDs on channels 11, 12
     for (int i = 0; i < 1; i++){
      digitalWrite(batteryLEDs[i], HIGH);
    }
  } 
  else {
    // 50-25%: Light up LEDs on channels 11, 12
     for (int i = 0; i < 2; i++){
      digitalWrite(batteryLEDs[i], LOW);
    }
  } 
  // If batteryPercent == 0, all LEDs stay off
  // Turn off all battery LEDs first
  // for (int i = 0; i < 4; i++){
  //   digitalWrite(batteryLEDs[i], LOW);
  // } 
}
