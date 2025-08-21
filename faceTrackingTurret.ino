#include <ESP32Servo.h>

// Servo objects
Servo panServo;   // Pan servo (GPIO 23)
Servo tiltServo;  // Tilt servo (GPIO 27)
const int panServoPin = 23;   // Pin for pan servo
const int tiltServoPin = 27;  // Pin for tilt servo
const int laserPin = 32;      // Pin for laser (GPIO 32)

// Smoothing parameters
const float ALPHA = 0.2;      // EMA smoothing factor (0 < ALPHA < 1, lower = more smoothing)
const float MAX_ANGLE_STEP = 3.0;  // Maximum angle change per update (degrees)
const unsigned long UPDATE_INTERVAL = 100;  // Minimum time between updates (ms)

// Variables for serial communication, smoothing, and laser control
String inputString = "";
bool stringComplete = false;
float currentPanAngle = 90.0;  // Current smoothed pan angle
float currentTiltAngle = 90.0; // Current smoothed tilt angle
int lastPanAngle = 90;         // Last valid pan angle (default 90°)
int lastTiltAngle = 90;        // Last valid tilt angle (default 90°)
unsigned long lastUpdateTime = 0;  // Last servo update time
int deadzoneCount = 0;         // Counter for consecutive deadzone commands
bool laserOn = false;           // Laser state

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }

  // Initialize laser pin
  pinMode(laserPin, OUTPUT);
  digitalWrite(laserPin, LOW);  // Laser off initially

  // Initialize servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  panServo.setPeriodHertz(50);  // Standard 50Hz for servos
  if (!panServo.attach(panServoPin, 1000, 2000)) {
    Serial.println("[ERROR] Failed to attach pan servo on pin 23");
  }
  
  tiltServo.setPeriodHertz(50);
  if (!tiltServo.attach(tiltServoPin, 1000, 2000)) {
    Serial.println("[ERROR] Failed to attach tilt servo on pin 27");
  }

  // Set initial servo positions to 90 degrees
  panServo.write((int)currentPanAngle);
  tiltServo.write((int)currentTiltAngle);
  
  // Reserve space for input string to prevent memory issues
  inputString.reserve(50);
  
  Serial.println("[INFO] ESP32 Servo Control Initialized");
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void updateServo(Servo &servo, float &currentAngle, float targetAngle, int &lastAngle, int maxAngle) {
  // Apply EMA smoothing
  float smoothedAngle = ALPHA * targetAngle + (1 - ALPHA) * currentAngle;
  
  // Limit angle change to MAX_ANGLE_STEP
  float delta = smoothedAngle - currentAngle;
  if (abs(delta) > MAX_ANGLE_STEP) {
    delta = (delta > 0) ? MAX_ANGLE_STEP : -MAX_ANGLE_STEP;
  }
  currentAngle += delta;
  
  // Clamp angle within valid range
  currentAngle = constrain(currentAngle, 0, maxAngle);
  lastAngle = (int)currentAngle;
  
  // Update servo position
  servo.write(lastAngle);
}

void loop() {
  // Process serial input when a complete string is received and enough time has passed
  unsigned long currentTime = millis();
  if (stringComplete && (currentTime - lastUpdateTime >= UPDATE_INTERVAL)) {
    inputString.trim(); // Remove whitespace, including newline
    
    // Handle 'deadzone' command
    if (inputString == "deadzone") {
      deadzoneCount++;  // Increment deadzone counter
      panServo.write(lastPanAngle);   // Maintain last pan position
      tiltServo.write(lastTiltAngle); // Maintain last tilt position
      Serial.printf("[INFO][%lu] Deadzone: Servos maintained at pan: %d, tilt: %d, Deadzone count: %d\n", 
                    currentTime, lastPanAngle, lastTiltAngle, deadzoneCount);
      
      // Turn on laser after 3 consecutive deadzone commands
      if (deadzoneCount >= 3 && !laserOn) {
        digitalWrite(laserPin, HIGH);
        laserOn = true;
        Serial.printf("[INFO][%lu] Laser ON\n", currentTime);
      }
      Serial.println("ACK");
    }
    // Handle 'pan:X' command
    else if (inputString.startsWith("pan:")) {
      String angleStr = inputString.substring(4); // Extract number after "pan:"
      int angle = angleStr.toInt();
      // Validate pan angle (0-180 degrees)
      if (angle >= 0 && angle <= 180) {
        updateServo(panServo, currentPanAngle, angle, lastPanAngle, 180);
        Serial.printf("[INFO][%lu] Set pan: %d (smoothed: %.2f)\n", currentTime, lastPanAngle, currentPanAngle);
        // Turn off laser and reset deadzone counter
        if (laserOn) {
          digitalWrite(laserPin, LOW);
          laserOn = false;
          Serial.printf("[INFO][%lu] Laser OFF\n", currentTime);
        }
        deadzoneCount = 0;  // Reset deadzone counter
        Serial.println("ACK");
      } else {
        Serial.printf("[ERROR][%lu] Invalid pan angle: %d, must be 0-180\n", currentTime, angle);
        Serial.println("ERROR");
      }
    }
    // Handle 'tilt:Y' command
    else if (inputString.startsWith("tilt:")) {
      String angleStr = inputString.substring(5); // Extract number after "tilt:"
      int angle = angleStr.toInt();
      // Validate tilt angle (0-170 degrees)
      if (angle >= 0 && angle <= 170) {
        updateServo(tiltServo, currentTiltAngle, angle, lastTiltAngle, 170);
        Serial.printf("[INFO][%lu] Set tilt: %d (smoothed: %.2f)\n", currentTime, lastTiltAngle, currentTiltAngle);
        // Turn off laser and reset deadzone counter
        if (laserOn) {
          digitalWrite(laserPin, LOW);
          laserOn = false;
          Serial.printf("[INFO][%lu] Laser OFF\n", currentTime);
        }
        deadzoneCount = 0;  // Reset deadzone counter
        Serial.println("ACK");
      } else {
        Serial.printf("[ERROR][%lu] Invalid tilt angle: %d, must be 0-170\n", currentTime, angle);
        Serial.println("ERROR");
      }
    }
    // Handle invalid commands
    else {
      Serial.printf("[ERROR][%lu] Invalid command: '%s', expected 'pan:X', 'tilt:Y', or 'deadzone'\n", currentTime, inputString.c_str());
      Serial.println("ERROR");
    }
    
    // Update last update time
    lastUpdateTime = currentTime;
    
    // Clear input string and reset flag
    inputString = "";
    stringComplete = false;
  }
  
  delay(10); // Small delay to prevent CPU overload
}