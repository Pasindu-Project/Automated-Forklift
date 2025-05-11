#include <WiFi.h>
#include <WebServer.h>
#include <Firebase_ESP_Client.h>
#include <addons/TokenHelper.h>

// WiFi Credentials
#define WIFI_SSID "Dialog 4G 800"
#define WIFI_PASSWORD "62304AAD"

//#define WIFI_SSID "Dushan Wifi"
//#define WIFI_PASSWORD "EB9817CF"

// Firebase Credentials
#define FIREBASE_HOST "warehouse-35f88-default-rtdb.firebaseio.com"
#define FIREBASE_AUTH "Kksg7rG5sIHDoY9k1pAkDGNhDspnQiSwp1DJ4Ali"

// Initialize Firebase
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

WebServer server(80);

const int dirPin = TX;  // GPIO18 for direction
const int stepPin = 2; // GPIO19 for step
const int stepsPerRevolution = 200;
const int numberOfRevolutions = 5;
const int totalSteps = stepsPerRevolution * numberOfRevolutions;

// Define rotation steps
const int firstFloorRotations = 2;  // 5 rounds
const int secondFloorRotations = 15; // 10 rounds
int rotations = 0;
bool greenorder = false;
bool store = false;
bool blue_order = false;
bool blue_store = false;
bool firebaseChecked = false;

float distance1 = 100;
float distance2 = 100;
float distance3 = 100;

#define leftSensorPin    35 //front sensors
#define middleSensorPin  18
#define rightSensorPin   34

#define leftSensor_backPin    5 //back sensors
#define middleSensor_backPin  4
#define rightSensor_backPin   15

#define leftMotorForward_IN1   25 // Back Right motor
#define leftMotorForward_IN2   26
#define leftMotorForward_EN    27

#define leftMotorBackward_IN1  32  // Back Left motor
#define leftMotorBackward_IN2  33
#define leftMotorBackward_EN   14

#define rightMotorForward_IN1  19   //front left motor
#define rightMotorForward_IN2  21
#define rightMotorForward_EN   22

#define rightMotorBackward_IN1  23  //front Right motor
#define rightMotorBackward_IN2  12
#define rightMotorBackward_EN   13

// Global variables for junction and special mode handling
int junctionCount = 0;               // Count of junctions detected (all sensors HIGH)
bool junctionDetected = false;       // Debounce flag to avoid counting the same junction repeatedly
bool inSpecialMode = false;
bool inSpecialMode_back = false;         // Flag for special mode (active after first junction)
unsigned long specialModeStartTime = 0;
unsigned long specialModeStartTime_back = 0;
const unsigned long specialModeDuration_M1 = 3000;
const unsigned long specialModeDuration_M2 = 3000;
const unsigned long specialModeDuration_M3 = 1500;
const unsigned long specialModeDuration_M1_back = 2500; // Special mode duration (milliseconds)
const unsigned long specialModeDuration_M2_back = 2000;
const unsigned long specialModeDuration_M3_back = 1500;
const unsigned long specialModeDuration_B = 3500;
const unsigned long specialModeDuration_B_back = 3000;
const unsigned long specialModeDuration_B1 = 3000;
const unsigned long specialModeDuration_B1_back = 1500;
const unsigned long specialModeDuration_B2 = 2500;
const unsigned long specialModeDuration_G = 3500;
const unsigned long specialModeDuration_G_back = 3000;
const unsigned long specialModeDuration_S = 4000;
const unsigned long specialModeDuration_S_back = 3000;
const unsigned long specialModeDuration = 4000;
const unsigned long specialModeDuration_2 = 5000;
const unsigned long specialModeDuration_back = 5000; // Special mode duration (milliseconds)
const unsigned long specialModeDuration_back_2 = 5000;


bool move = true;                    // Flag to control when normal movement is allowed
bool rivers = false;
bool stepper_rotate = true;
bool firstFloorEmpty = true;
bool secondFloorEmpty = true;
String storecolor = "null";
String orderedColor = "null";

void setup() {

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);

  // Initialize sensor pins as inputs
  pinMode(leftSensorPin, INPUT);
  pinMode(middleSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  // back IR sensor
  pinMode(leftSensor_backPin, INPUT);
  pinMode(middleSensor_backPin, INPUT);
  pinMode(rightSensor_backPin, INPUT);

  // Initialize motor control pins as outputs
  pinMode(leftMotorForward_IN1, OUTPUT);
  pinMode(leftMotorForward_IN2, OUTPUT);
  pinMode(leftMotorForward_EN, OUTPUT);
  pinMode(leftMotorBackward_IN1, OUTPUT);
  pinMode(leftMotorBackward_IN2, OUTPUT);
  pinMode(leftMotorBackward_EN, OUTPUT);
  pinMode(rightMotorForward_IN1, OUTPUT);
  pinMode(rightMotorForward_IN2, OUTPUT);
  pinMode(rightMotorForward_EN, OUTPUT);
  pinMode(rightMotorBackward_IN1, OUTPUT);
  pinMode(rightMotorBackward_IN2, OUTPUT);
  pinMode(rightMotorBackward_EN, OUTPUT);
  
  // Start Serial for debugging
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  server.on("/distance", handleDistanceUpdate);
  server.begin();
  
  // Firebase Configuration
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  server.handleClient();

  if (!firebaseChecked) {
    // Check detected color node
    if (Firebase.RTDB.getString(&fbdo, "/color/detected")) {
      storecolor = fbdo.stringData();
      // Again, use the ordered color to determine which shelf to check.
      checkAndMoveStepper(storecolor);
      if (storecolor == "Green" && (firstFloorEmpty || secondFloorEmpty)) { 
        store = true;
        firebaseChecked = true;
      } else if (storecolor == "Blue" && (firstFloorEmpty || secondFloorEmpty)){
        blue_store = true;
        firebaseChecked = true;
      }else {
        store = false;
        blue_store = false;
        firebaseChecked = false;
        stopMotors();
      }
    }
    if (!firebaseChecked) {
      // Check placed color node
      if (Firebase.RTDB.getString(&fbdo, "/color/placed_color")) {
        orderedColor = fbdo.stringData();
        // Again, use the ordered color to determine which shelf to check.
        checkAndMoveStepper(orderedColor);
        if (orderedColor == "Green" && (!firstFloorEmpty || !secondFloorEmpty)) { 
          greenorder = true;
          firebaseChecked = true;
        } else if (orderedColor == "Blue" && (!firstFloorEmpty || !secondFloorEmpty)){
          blue_order = true;
          firebaseChecked = true;
        } else {
          greenorder = false;
          blue_order = false;
          stopMotors();
        }
      }
    }
  }

  
  // Execute movement based on the current flag and line following logic.
  if (store) {
    followLine();
  }else if (greenorder){
    green_order();
  }else if(blue_store){
    Blue_store();
  }else if(blue_order){
    Blue_order();
  }
}

void followLine() {

  // Read the three IR sensors
  int leftValue   = digitalRead(leftSensorPin);
  int middleValue = digitalRead(middleSensorPin);
  int rightValue  = digitalRead(rightSensorPin);
  //read back IR sensors
  int leftValue_back   = digitalRead(leftSensor_backPin);
  int middleValue_back = digitalRead(middleSensor_backPin);
  int rightValue_back  = digitalRead(rightSensor_backPin);

  if (move) {
    // Check for junction: all sensors HIGH
    if (leftValue == HIGH && middleValue == HIGH && rightValue == HIGH) {
      if (!junctionDetected){
        junctionDetected = true;
        junctionCount++;
      //Serial.print("Junction detected. Count = ");
      //Serial.println(junctionCount);
        stopMotors();
        delay(2000);
      }
                    // Pause after detecting a junction
                   // Prevent further commands until conditions change

        // If this is the first junction, enter special mode.
      if (junctionCount == 1) {
        move = false;
        moveForward(110);        // Resume motion briefly
        delay(350);
        inSpecialMode = true;
        specialModeStartTime = millis();
      }
      if (junctionCount == 2) {
        move = false;
        moveForward(110);        // Resume motion briefly
        delay(350);
        inSpecialMode = true;
        specialModeStartTime = millis();
      }

      if (junctionCount == 3) {
        junctionDetected = false;
        moveForward(110);        // Resume motion briefly
        delay(450);
      }

      if (junctionCount == 4) {
        junctionDetected = false;
        move = false;
        rotateStepperMotor(3,false);
        updateShelfStatus(storecolor);////////////////////////
        rivers = true;
      }
      if (junctionCount == 9) {
        moveForward(110);        // Resume motion briefly
        delay(400);
        move = false;
        inSpecialMode = true;
        specialModeStartTime = millis();
      }

      if (junctionCount == 10) {
        junctionDetected = false;
        moveForward(110);        // Resume motion briefly
        delay(500);
      }

      if (junctionCount == 11) {
        move = false;
        if (firstFloorEmpty) {
          rotateStepperMotor(firstFloorRotations, false);
          rotations = firstFloorRotations;
          
        } else if (secondFloorEmpty) {
          rotateStepperMotor(secondFloorRotations, false);
          rotations = secondFloorRotations;
        } 
        moveForward(110);        // Resume motion briefly
        delay(750);
        move = true;
        junctionDetected = false;
      }
      if (junctionCount == 12) {
        rotateStepperMotor(2,true);
        junctionDetected = false;
        move = false;
        rivers = true;
      }
    }
    // Normal line following conditions:
    else if (middleValue == HIGH && leftValue == LOW && rightValue == LOW) {
      if ((distance1 < 20 || distance2 < 20) && (junctionCount != 3 && junctionCount != 4 && junctionCount != 11 && junctionCount != 12)) {
        stopMotors();
      } else {
        moveForward(100);
      }
    }
    else if (leftValue == HIGH && middleValue == LOW && rightValue == LOW) {
      turnLeft(180);
    }
    else if (rightValue == HIGH && middleValue == LOW && leftValue == LOW) {
      turnRight(180);
    }
    else {
      if ((distance1 < 20 || distance2 < 20) && (junctionCount != 3 && junctionCount != 4 && junctionCount != 11 && junctionCount != 12)) {
        stopMotors();
      } else {
        moveForward(100);
      }  // Default: move forward
    }
  }

  // for backward moving
  if (rivers) {
    // Check for junction: all sensors HIGH
    if (leftValue_back == HIGH && middleValue_back == HIGH && rightValue_back == HIGH) {
      if (!junctionDetected){
        junctionDetected = true;
        junctionCount++;
      //Serial.print("Junction detected. Count = ");
      //Serial.println(junctionCount);
        stopMotors();
        delay(2000);
      }
      

        // If this is the first junction, enter special mode.
      if (junctionCount == 5) {
        moveBackward(110);        // Resume motion briefly
        delay(500);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }

      if (junctionCount == 6) {
        moveBackward(110);        // Resume motion briefly
        delay(450);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }

      if (junctionCount == 7){
        moveBackward(100);        // Resume motion briefly
        delay(500);
        junctionDetected = false;
      }
      if (junctionCount == 8){
        stopMotors();
        delay(2000);
        junctionDetected = false;
        move = true;
        rivers = false;
      }

      if (junctionCount == 15){
        moveBackward(100);        // Resume motion briefly
        delay(400);
        if(rightValue_back = HIGH){
          turnRight(180);
        }else{
          moveBackward(100);
        }
        junctionDetected = false;
      }

      if (junctionCount == 14) {
        moveBackward(100);        // Resume motion briefly
        delay(750);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }
      if (junctionCount == 13){
        rotateStepperMotor(rotations,true);
        moveBackward(100);        // Resume motion briefly
        delay(600);
        junctionDetected = false;
      }
      if (junctionCount == 16){
        stopMotors();
        rotateStepperMotor(1,true);
        junctionDetected = false;
        move = false;
        rivers = false;
        ESP.restart();
      }
    }
    // Normal line following conditions:
    else if (middleValue_back == HIGH && leftValue_back == LOW && rightValue_back == LOW) {
      if (distance3 < 20) {
        stopMotors();
      } else {
        moveBackward(100);
      }
    }
    else if (leftValue_back == HIGH && middleValue_back == LOW && rightValue_back == LOW) {
      turnLeft(180);
    }
    else if (rightValue_back == HIGH && middleValue_back == LOW && leftValue_back == LOW) {
      turnRight(180);
    }
    else {
      if (distance3 < 20) {
        stopMotors();
      } else {
        moveBackward(100);
      }  // Default: move forward
    }
  }

  // Special Mode (after first junction)
  if (inSpecialMode && (junctionCount == 1)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_M1) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue == HIGH) {
        turnLeft(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode && (junctionCount == 2)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_M2) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue == HIGH ) {
        turnLeft(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  //for backward junction
  if (inSpecialMode_back && (junctionCount == 5)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_M1_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue_back == HIGH ) {
        turnRight(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode_back && (junctionCount == 6)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_M2_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue_back == HIGH ) {
        turnRight(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode && junctionCount == 9) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_B) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue == HIGH) {
        turnRight(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode_back && junctionCount == 14) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_B_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue_back == HIGH ) {
        turnLeft(180);
      }else if (middleValue_back == LOW && leftValue_back == LOW) {
        turnRight(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
}

void green_order(){
  // Read the three IR sensors
  int leftValue   = digitalRead(leftSensorPin);
  int middleValue = digitalRead(middleSensorPin);
  int rightValue  = digitalRead(rightSensorPin);
  //read back IR sensors
  int leftValue_back   = digitalRead(leftSensor_backPin);
  int middleValue_back = digitalRead(middleSensor_backPin);
  int rightValue_back  = digitalRead(rightSensor_backPin);

  if (move) {
    // Check for junction: all sensors HIGH
    if (leftValue == HIGH && middleValue == HIGH && rightValue == HIGH) {
      if (!junctionDetected){
        junctionDetected = true;
        junctionCount++;
      //Serial.print("Junction detected. Count = ");
      //Serial.println(junctionCount);
        stopMotors();
        delay(2000);
      }
                    // Pause after detecting a junction
                   // Prevent further commands until conditions change

        // If this is the first junction, enter special mode.
      if (junctionCount == 1) {
        move = false;
        moveForward(110);        // Resume motion briefly
        delay(750);
        inSpecialMode = true;
        specialModeStartTime = millis();
         
      }
      if (junctionCount == 2) {
        moveForward(110);        // Resume motion briefly
        delay(600);
        junctionDetected = false;
      }
      if (junctionCount == 3) {
        move = false;
        if (!firstFloorEmpty) {
          rotateStepperMotor(firstFloorRotations+1, false);
          rotations = firstFloorRotations;
          
        } else if (!secondFloorEmpty) {
          rotateStepperMotor(secondFloorRotations+1, false);
          rotations = secondFloorRotations;
        } 
        moveForward(110);        // Resume motion briefly
        delay(750);
        move = true;
        junctionDetected = false;
        
      }
      if (junctionCount == 4) {
        String shelfPath1 = orderedColor + "Shelf/firstFloor";
        String shelfPath2 = orderedColor + "Shelf/secondFloor"; // dynamically create path (e.g., "/GreenShelf")
        if (!firstFloorEmpty) {
          Firebase.RTDB.setBool(&fbdo, shelfPath1.c_str(), true);
        } else if (!secondFloorEmpty) {
          Firebase.RTDB.setBool(&fbdo, shelfPath2.c_str(), true);
        }
        rotateStepperMotor(2,false);
        junctionDetected = false;
        move = false;
        rivers = true;
      }
      if (junctionCount == 8) {
        moveForward(110);        // Resume motion briefly
        delay(300);
        move = false;
        inSpecialMode = true;
        specialModeStartTime = millis();
      }
      if (junctionCount == 9) {
        moveForward(110);        // Resume motion briefly
        delay(400);
        move = false;
        inSpecialMode = true;
        specialModeStartTime = millis();
      }
      if (junctionCount == 10) {
        moveForward(110);        // Resume motion briefly
        delay(400);
        move = false;
        inSpecialMode = true;
        specialModeStartTime = millis();
      }
      if (junctionCount == 11) {
        moveForward(110);        // Resume motion briefly
        delay(600);
        move = false;
        inSpecialMode = true;
        specialModeStartTime = millis();
      }
      if (junctionCount == 12) {
        moveForward(110);        // Resume motion briefly
        delay(450);
        junctionDetected = false;
      }
      if (junctionCount == 13) {
        junctionDetected = false;
        move = false;
        Firebase.RTDB.setString(&fbdo, "/color/placed_color", "null");
        rotateStepperMotor(3,true);////////////////////////
        rivers = true;
      }
    }
    // Normal line following conditions:
    else if (middleValue == HIGH && leftValue == LOW && rightValue == LOW) {
      if ((distance1 < 20 || distance2 < 20) && (junctionCount != 3 && junctionCount != 4 && junctionCount != 12 && junctionCount != 13)) {
        stopMotors();
      } else {
        moveForward(100);
      }
    }
    else if (leftValue == HIGH && middleValue == LOW && rightValue == LOW) {
      turnLeft(180);
    }
    else if (rightValue == HIGH && middleValue == LOW && leftValue == LOW) {
      turnRight(180);
    }
    else {
      if ((distance1 < 20 || distance2 < 20) && (junctionCount != 3 && junctionCount != 4 && junctionCount != 12 && junctionCount != 13)) {
        stopMotors();
      } else {
        moveForward(100);
      }  // Default: move forward
    }
  }

  // for backward moving
  if (rivers) {
    // Check for junction: all sensors HIGH
    if (leftValue_back == HIGH && middleValue_back == HIGH && rightValue_back == HIGH) {
      if (!junctionDetected){
        junctionDetected = true;
        junctionCount++;
      //Serial.print("Junction detected. Count = ");
      //Serial.println(junctionCount);
        stopMotors();
        delay(2000);
      }
      
      if (junctionCount == 5){
        rotateStepperMotor(rotations,true);
        moveBackward(100);        // Resume motion briefly
        delay(400);
        junctionDetected = false;
      }
        // If this is the first junction, enter special mode.
      if (junctionCount == 6) {
        moveBackward(110);        // Resume motion briefly
        delay(300);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }
      if (junctionCount == 7){
        moveBackward(100);        // Resume motion briefly
        delay(400);
        if(rightValue_back = HIGH){
          turnRight(180);
        }else{
          moveBackward(100);
        }
        junctionDetected = false;
      }


      if (junctionCount == 8){
        stopMotors();
        delay(2000);
        junctionDetected = false;
        move = true;
        rivers = false;
      }
      if (junctionCount == 14) {
        moveBackward(100);        // Resume motion briefly
        delay(750);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }
      if (junctionCount == 15) {
        moveBackward(100);        // Resume motion briefly
        delay(750);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }
      if (junctionCount == 16) {
        moveBackward(100);        // Resume motion briefly
        delay(500);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }
      if (junctionCount == 17){
        moveBackward(100);        // Resume motion briefly
        delay(400);
        if(rightValue_back = HIGH){
          turnRight(180);
        }else{
          moveBackward(100);
        }
        junctionDetected = false;
      }
      
      if (junctionCount == 18){
        stopMotors();
        junctionDetected = false;
        move = false;
        rivers = false;
        ESP.restart();
      }
    }
    // Normal line following conditions:
    else if (middleValue_back == HIGH && leftValue_back == LOW && rightValue_back == LOW) {
      if (distance3 < 20) {
        stopMotors();
      } else {
        moveBackward(100);
      }
    }
    else if (leftValue_back == HIGH && middleValue_back == LOW && rightValue_back == LOW) {
      turnLeft(180);
    }
    else if (rightValue_back == HIGH && middleValue_back == LOW && leftValue_back == LOW) {
      turnRight(180);
    }
    else {
      if (distance3 < 20) {
        stopMotors();
      } else {
        moveBackward(100);
      }  // Default: move forward
    }
  }

  // Special Mode (after first junction)
  if (inSpecialMode && junctionCount == 1) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_G) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue == HIGH ) {
        turnRight(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode_back && junctionCount == 6) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_G_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue_back == HIGH ) {
        turnLeft(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode && (junctionCount == 9)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_M1) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue == HIGH ) {
        turnLeft(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode && junctionCount == 10) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_B) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue == HIGH ) {
        turnRight(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode && (junctionCount == 11)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_S) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue == HIGH ) {
        turnLeft(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  //for backward junction
  if (inSpecialMode_back && (junctionCount == 14)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_S_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue_back == HIGH ) {
        turnRight(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode_back && junctionCount == 15) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_B_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue_back == HIGH ) {
        turnLeft(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode_back && (junctionCount == 16)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_M2_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue_back == HIGH ) {
        turnRight(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
}

void Blue_store(){

  // Read the three IR sensors
  int leftValue   = digitalRead(leftSensorPin);
  int middleValue = digitalRead(middleSensorPin);
  int rightValue  = digitalRead(rightSensorPin);
  //read back IR sensors
  int leftValue_back   = digitalRead(leftSensor_backPin);
  int middleValue_back = digitalRead(middleSensor_backPin);
  int rightValue_back  = digitalRead(rightSensor_backPin);

  if (move) {
    // Check for junction: all sensors HIGH
    if (leftValue == HIGH && middleValue == HIGH && rightValue == HIGH) {
      if (!junctionDetected){
        junctionDetected = true;
        junctionCount++;
      //Serial.print("Junction detected. Count = ");
      //Serial.println(junctionCount);
        stopMotors();
        delay(2000);
      }
                    // Pause after detecting a junction
                   // Prevent further commands until conditions change

        // If this is the first junction, enter special mode.
      if (junctionCount == 1) {
        move = false;
        moveForward(110);        // Resume motion briefly
        delay(500);
        inSpecialMode = true;
        specialModeStartTime = millis();
         
      }
      if (junctionCount == 2) {
        move = false;
        moveForward(110);        // Resume motion briefly
        delay(500);
        inSpecialMode = true;
        specialModeStartTime = millis();
      }

      if (junctionCount == 3) {
        junctionDetected = false;
        moveForward(110);        // Resume motion briefly
        delay(400);
      }

      if (junctionCount == 9) {
        move = false;
        moveForward(110);        // Resume motion briefly
        delay(500);
        inSpecialMode = true;
        specialModeStartTime = millis();
      }

      if (junctionCount == 4) {
        junctionDetected = false;
        move = false;
        String shelfPath1 = "/" + storecolor + "Shelf/firstFloor";
        String shelfPath2 = "/" + storecolor + "Shelf/secondFloor"; // dynamically create path (e.g., "/GreenShelf")
        if (!firstFloorEmpty) {
          Firebase.RTDB.setBool(&fbdo, shelfPath1.c_str(), false);
        } else if (!secondFloorEmpty) {
          Firebase.RTDB.setBool(&fbdo, shelfPath2.c_str(), false);
        }
        updateShelfStatus(storecolor);
        rotateStepperMotor(3,false);////////////////////////
        rivers = true;
      }
      if (junctionCount == 10) {
        moveForward(110);        // Resume motion briefly
        delay(400);
        move = false;
        inSpecialMode = true;
        specialModeStartTime = millis();
      }
      if (junctionCount == 11) {
        moveForward(110);        // Resume motion briefly
        delay(450);
        move = false;
        inSpecialMode = true;
        specialModeStartTime = millis();
      }
      if (junctionCount == 12) {
        moveForward(110);        // Resume motion briefly
        delay(300);
        junctionDetected = false;
      }
      if (junctionCount == 13) {
        move = false;
        if (firstFloorEmpty) {
          rotateStepperMotor(firstFloorRotations, false);
          rotations = firstFloorRotations;
          
        } else if (secondFloorEmpty) {
          rotateStepperMotor(secondFloorRotations, false);
          rotations = secondFloorRotations;
        } 
        moveForward(110);        // Resume motion briefly
        delay(500);
        move = true;
        junctionDetected = false;
      }
      if (junctionCount == 14) {
        rotateStepperMotor(2,true);
        junctionDetected = false;
        move = false;
        rivers = true;
      }
    }
    // Normal line following conditions:
    else if (middleValue == HIGH && leftValue == LOW && rightValue == LOW) {
      if ((distance1 < 20 || distance2 < 20) && (junctionCount != 3 && junctionCount != 4 && junctionCount != 13 && junctionCount != 14)) {
        stopMotors();
      } else {
        moveForward(100);
      }
    }
    else if (leftValue == HIGH && middleValue == LOW && rightValue == LOW) {
      turnLeft(180);
    }
    else if (rightValue == HIGH && middleValue == LOW && leftValue == LOW) {
      turnRight(180);
    }
    else {
      if ((distance1 < 20 || distance2 < 20) && (junctionCount != 3 && junctionCount != 4 && junctionCount != 13 && junctionCount != 14)) {
        stopMotors();
      } else {
        moveForward(100);
      }  // Default: move forward
    }
  }

  // for backward moving
  if (rivers) {
    // Check for junction: all sensors HIGH
    if (leftValue_back == HIGH && middleValue_back == HIGH && rightValue_back == HIGH) {
      if (!junctionDetected){
        junctionDetected = true;
        junctionCount++;
      //Serial.print("Junction detected. Count = ");
      //Serial.println(junctionCount);
        stopMotors();
        delay(2000);
      }
      

        // If this is the first junction, enter special mode.
      if (junctionCount == 5) {
        moveBackward(110);        // Resume motion briefly
        delay(500);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }

      if (junctionCount == 6) {
        moveBackward(110);        // Resume motion briefly
        delay(500);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }

      if (junctionCount == 7){
        moveBackward(100);        // Resume motion briefly
        delay(400);
        junctionDetected = false;
      }
      if (junctionCount == 15){
         rotateStepperMotor(rotations,true);
        moveBackward(100);        // Resume motion briefly
        delay(500);
        junctionDetected = false;
      }
      if (junctionCount == 16){
        moveBackward(100);        // Resume motion briefly
        delay(750);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();
      }

      if (junctionCount == 8){
        stopMotors();
        delay(2000);
        junctionDetected = false;
        move = true;
        rivers = false;
      }
      if (junctionCount == 17) {
        moveBackward(100);        // Resume motion briefly
        delay(750);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }
      if (junctionCount == 18) {
        moveBackward(100);        // Resume motion briefly
        delay(750);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }
      if (junctionCount == 19){
        moveBackward(100);        // Resume motion briefly
        delay(400);
        if (rightValue_back == HIGH){
          turnRight(180);
        }else if(leftValue_back == HIGH){
          turnLeft(180);
        }
        junctionDetected = false;
      }
      if (junctionCount == 20){
        stopMotors();
        rotateStepperMotor(1,true);
        junctionDetected = false;
        move = false;
        rivers = false;
        ESP.restart();
      }
    }
    // Normal line following conditions:
    else if (middleValue_back == HIGH && leftValue_back == LOW && rightValue_back == LOW) {
      if (distance3 < 20) {
        stopMotors();
      } else {
        moveBackward(100);
      }
    }
    else if (leftValue_back == HIGH && middleValue_back == LOW && rightValue_back == LOW) {
      turnLeft(180);
    }
    else if (rightValue_back == HIGH && middleValue_back == LOW && leftValue_back == LOW) {
      turnRight(180);
    }
    else {
      if (distance3 < 20) {
        stopMotors();
      } else {
        moveBackward(100);
      }  // Default: move forward
    }
  }

  // Special Mode (after first junction)
  if (inSpecialMode && (junctionCount == 1)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_M1) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue == HIGH) {
        turnLeft(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode && (junctionCount == 2)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_M2) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue == HIGH ) {
        turnLeft(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  //for backward junction
  if (inSpecialMode_back && (junctionCount == 5)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_M1_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue_back == HIGH ) {
        turnRight(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode_back && (junctionCount == 6)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_M2_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue_back == HIGH ) {
        turnRight(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode && (junctionCount == 9)) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_M1) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue == HIGH ) {
        turnLeft(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode && junctionCount == 10) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_B) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue == HIGH) {
        turnRight(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode && junctionCount == 11) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_B1) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue == HIGH) {
        turnRight(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  //for backward junction
  if (inSpecialMode_back && junctionCount == 16) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_B1_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue_back == HIGH ) {
        turnLeft(180);
      }else if (leftValue_back == LOW && middleValue_back == LOW){
        turnRight(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode_back && junctionCount == 17) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_B_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue_back == HIGH ) {
        turnLeft(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode_back && junctionCount == 18) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_M2_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue_back == HIGH ) {
        turnRight(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  
}

void Blue_order(){
  // Read the three IR sensors
  int leftValue   = digitalRead(leftSensorPin);
  int middleValue = digitalRead(middleSensorPin);
  int rightValue  = digitalRead(rightSensorPin);
  //read back IR sensors
  int leftValue_back   = digitalRead(leftSensor_backPin);
  int middleValue_back = digitalRead(middleSensor_backPin);
  int rightValue_back  = digitalRead(rightSensor_backPin);

  if (move) {
    // Check for junction: all sensors HIGH
    if (leftValue == HIGH && middleValue == HIGH && rightValue == HIGH) {
      if (!junctionDetected){
        junctionDetected = true;
        junctionCount++;
      //Serial.print("Junction detected. Count = ");
      //Serial.println(junctionCount);
        stopMotors();
        delay(2000);
      }
                    // Pause after detecting a junction
                   // Prevent further commands until conditions change

        // If this is the first junction, enter special mode.
      if (junctionCount == 1) {
        move = false;
        moveForward(110);        // Resume motion briefly
        delay(500);
        inSpecialMode = true;
        specialModeStartTime = millis();
         
      }
      if (junctionCount == 2) {
        move = false;
        moveForward(110);        // Resume motion briefly
        delay(500);
        inSpecialMode = true;
        specialModeStartTime = millis();
      }
      if (junctionCount == 3) {
        move = false;
        moveForward(110);        // Resume motion briefly
        delay(300);
        inSpecialMode = true;
        specialModeStartTime = millis();
      }

      if (junctionCount == 4) {
        junctionDetected = false;
        moveForward(110);        // Resume motion briefly
        delay(350);
      }
      if (junctionCount == 5) {
        move = false;
        if (!firstFloorEmpty) {
          rotateStepperMotor(firstFloorRotations+1, false);
          rotations = firstFloorRotations;
          
        } else if (!secondFloorEmpty) {
          rotateStepperMotor(secondFloorRotations+1, false);
          rotations = secondFloorRotations;
        } 
        moveForward(110);        // Resume motion briefly
        delay(500);
        move = true;
        junctionDetected = false;
      }
      if (junctionCount == 6) {
        String shelfPath1 = orderedColor + "Shelf/firstFloor";
        String shelfPath2 = orderedColor + "Shelf/secondFloor"; // dynamically create path (e.g., "/GreenShelf")
        if (!firstFloorEmpty) {
          Firebase.RTDB.setBool(&fbdo, shelfPath1.c_str(), true);
        } else if (!secondFloorEmpty) {
          Firebase.RTDB.setBool(&fbdo, shelfPath2.c_str(), true);
        }
        rotateStepperMotor(2,false);
        junctionDetected = false;
        move = false;
        rivers = true;
      }
      if (junctionCount == 10) {
        junctionDetected = false;
        moveForward(110);        // Resume motion briefly
        delay(400);
      }

      if (junctionCount == 11) {
        junctionDetected = false;
        move = false;
        Firebase.RTDB.setString(&fbdo, "/color/placed_color", "null");
        rotateStepperMotor(3,true);////////////////////////
        rivers = true;
      }
    }
    // Normal line following conditions:
    else if (middleValue == HIGH && leftValue == LOW && rightValue == LOW) {
      if ((distance1 < 20 || distance2 < 20) && (junctionCount != 5 && junctionCount != 6 && junctionCount != 10 && junctionCount != 11)) {
        stopMotors();
      } else {
        moveForward(100);
      }
    }
    else if (leftValue == HIGH && middleValue == LOW && rightValue == LOW) {
      turnLeft(180);
    }
    else if (rightValue == HIGH && middleValue == LOW && leftValue == LOW) {
      turnRight(180);
    }
    else {
      if ((distance1 < 20 || distance2 < 20) && (junctionCount != 5 && junctionCount != 6 && junctionCount != 10 && junctionCount != 11)) {
        stopMotors();
      } else {
        moveForward(100);
      }  // Default: move forward
    }
  }

  // for backward moving
  if (rivers) {
    // Check for junction: all sensors HIGH
    if (leftValue_back == HIGH && middleValue_back == HIGH && rightValue_back == HIGH) {
      if (!junctionDetected){
        junctionDetected = true;
        junctionCount++;
      //Serial.print("Junction detected. Count = ");
      //Serial.println(junctionCount);
        stopMotors();
        delay(2000);
      }
      

        // If this is the first junction, enter special mode.
      
      if (junctionCount == 7){
        rotateStepperMotor(rotations,true);
        moveBackward(100);        // Resume motion briefly
        delay(500);
        junctionDetected = false;
      }
      if (junctionCount == 8) {
        rivers = false;
        moveBackward(110);        // Resume motion briefly
        delay(500);
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }

      if (junctionCount == 9) {
        rivers = false;
        moveForward(110);        // Resume motion briefly
        delay(600);
        inSpecialMode = true;
        specialModeStartTime = millis();  
      }

      if (junctionCount == 12){
        moveBackward(100);        // Resume motion briefly
        delay(750);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();
      }
      if (junctionCount == 13) {
        moveBackward(100);        // Resume motion briefly
        delay(750);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }
      if (junctionCount == 14) {
        moveBackward(100);        // Resume motion briefly
        delay(750);
        rivers = false;
        inSpecialMode_back = true;
        specialModeStartTime_back = millis();  
      }
      
      if (junctionCount == 15){
        moveBackward(100);        // Resume motion briefly
        delay(400);
        if (rightValue_back == HIGH){
          turnRight(180);
        }else if(leftValue_back == HIGH){
          turnLeft(180);
        }
        junctionDetected = false;
      }
      if (junctionCount == 16){
        stopMotors();
        junctionDetected = false;
        move = false;
        rivers = false;
        ESP.restart();
      }
    }
    // Normal line following conditions:
    else if (middleValue_back == HIGH && leftValue_back == LOW && rightValue_back == LOW) {
      if (distance3 < 20) {
        stopMotors();
      } else {
        moveBackward(100);
      }
    }
    else if (leftValue_back == HIGH && middleValue_back == LOW && rightValue_back == LOW) {
      turnLeft(180);
    }
    else if (rightValue_back == HIGH && middleValue_back == LOW && leftValue_back == LOW) {
      turnRight(180);
    }
    else {
      if (distance3 < 20) {
        stopMotors();
      } else {
        moveBackward(100);
      }  // Default: move forward
    }
  }

  // Special Mode (after first junction)
  if (inSpecialMode && junctionCount == 1) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_M1) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue == HIGH ) {
        turnLeft(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode && junctionCount == 2) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_B) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue == HIGH ) {
        turnRight(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode && junctionCount == 3) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_B1) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue == HIGH ) {
        turnRight(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode_back && junctionCount == 8) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_B1_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue_back == HIGH ) {
        turnLeft(180);
      }else if (leftValue_back == LOW && middleValue_back == LOW){
        turnRight(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode && junctionCount == 9) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime < specialModeDuration_B2) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue == HIGH ) {
        turnLeft(180);
      }else {
        moveForward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      move = true;
      inSpecialMode = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  
  //for backward junction
  if (inSpecialMode_back && junctionCount == 12) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_S_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue_back == HIGH ) {
        turnRight(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }

  if (inSpecialMode_back && junctionCount == 13) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_B_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (leftValue_back == HIGH ) {
        turnLeft(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }
  if (inSpecialMode_back && junctionCount == 14) {
    // Remain in special mode for the specified duration.
    if (millis() - specialModeStartTime_back < specialModeDuration_M2_back) {
      // For example, in special mode, if the right sensor is HIGH, turn right;
      // if both middle and right are LOW, turn left; otherwise, move forward.
      if (rightValue_back == HIGH ) {
        turnRight(180);
      }else {
        moveBackward(100);
      }
      // Skip the rest of the followLine function during special mode.
      return;
    }
    else {
      // End special mode once the time expires.
      rivers = true;
      inSpecialMode_back = false;
      junctionDetected = false;
      //Serial.println("Exiting Special Mode.");
    }
  }

}

// ------------------ MOTOR CONTROL FUNCTIONS ------------------

void moveForward(int speed) {
  // Left Motor Forward
  digitalWrite(leftMotorForward_IN1, LOW);
  digitalWrite(leftMotorForward_IN2, HIGH);
  analogWrite(leftMotorForward_EN, speed);
  // Left Motor Backward (set same direction as forward)
  digitalWrite(leftMotorBackward_IN1,LOW);
  digitalWrite(leftMotorBackward_IN2, HIGH);
  analogWrite(leftMotorBackward_EN, speed);
  
  // Right Motor Forward
  digitalWrite(rightMotorForward_IN1, LOW);
  digitalWrite(rightMotorForward_IN2, HIGH);
  analogWrite(rightMotorForward_EN, speed);
  // Right Motor Backward (set same direction as forward)
  digitalWrite(rightMotorBackward_IN1, LOW);
  digitalWrite(rightMotorBackward_IN2, HIGH);
  analogWrite(rightMotorBackward_EN, speed);
}
void moveBackward(int speed) {
  // Left Motor Forward
  digitalWrite(leftMotorForward_IN1, HIGH);
  digitalWrite(leftMotorForward_IN2, LOW);
  analogWrite(leftMotorForward_EN, speed);
  // Left Motor Backward (set same direction as forward)
  digitalWrite(leftMotorBackward_IN1, HIGH);
  digitalWrite(leftMotorBackward_IN2, LOW);
  analogWrite(leftMotorBackward_EN, speed);
  
  // Right Motor Forward
  digitalWrite(rightMotorForward_IN1, HIGH);
  digitalWrite(rightMotorForward_IN2, LOW);
  analogWrite(rightMotorForward_EN, speed);
  // Right Motor Backward (set same direction as forward)
  digitalWrite(rightMotorBackward_IN1, HIGH);
  digitalWrite(rightMotorBackward_IN2, LOW);
  analogWrite(rightMotorBackward_EN, speed);
}

void turnLeft(int speed) {
  // For a left turn, adjust motor speeds/directions:
  digitalWrite(leftMotorForward_IN1, LOW);
  digitalWrite(leftMotorForward_IN2, HIGH);
  analogWrite(leftMotorForward_EN, 200);
  
  digitalWrite(leftMotorBackward_IN1, HIGH);
  digitalWrite(leftMotorBackward_IN2, LOW);
  analogWrite(leftMotorBackward_EN, 200);
  
  digitalWrite(rightMotorForward_IN1, HIGH);
  digitalWrite(rightMotorForward_IN2, LOW);
  analogWrite(rightMotorForward_EN, 220);
  
  digitalWrite(rightMotorBackward_IN1, LOW);
  digitalWrite(rightMotorBackward_IN2, HIGH);
  analogWrite(rightMotorBackward_EN, 200);
}

void turnRight(int speed) {
  // For a right turn, adjust motor speeds/directions:
  digitalWrite(leftMotorForward_IN1, HIGH); // back right
  digitalWrite(leftMotorForward_IN2, LOW);
  analogWrite(leftMotorForward_EN, 200);
  
  digitalWrite(leftMotorBackward_IN1, LOW);  //back left
  digitalWrite(leftMotorBackward_IN2, HIGH);
  analogWrite(leftMotorBackward_EN, 240);
  
  digitalWrite(rightMotorForward_IN1, LOW); //front left
  digitalWrite(rightMotorForward_IN2, HIGH);
  analogWrite(rightMotorForward_EN, 200);
  
  digitalWrite(rightMotorBackward_IN1, HIGH);
  digitalWrite(rightMotorBackward_IN2, LOW);
  analogWrite(rightMotorBackward_EN, 220);
}

void stopMotors() {
  digitalWrite(leftMotorForward_IN1, LOW);
  digitalWrite(leftMotorForward_IN2, LOW);
  analogWrite(leftMotorForward_EN, 0);
  
  digitalWrite(leftMotorBackward_IN1, LOW);
  digitalWrite(leftMotorBackward_IN2, LOW);
  analogWrite(leftMotorBackward_EN, 0);
  
  digitalWrite(rightMotorForward_IN1, LOW);
  digitalWrite(rightMotorForward_IN2, LOW);
  analogWrite(rightMotorForward_EN, 0);
  
  digitalWrite(rightMotorBackward_IN1, LOW);
  digitalWrite(rightMotorBackward_IN2, LOW);
  analogWrite(rightMotorBackward_EN, 0);
}

void updateShelfStatus(String detectedColor) {
    String firstShelfPath = "/" + detectedColor + "Shelf/firstFloor";
    String secondShelfPath = "/" + detectedColor + "Shelf/secondFloor";

    Serial.print("Updating Firebase Path: ");
    Serial.println(firstShelfPath);
    Serial.println(secondShelfPath);

    if (firstFloorEmpty) {
        if (Firebase.RTDB.setBool(&fbdo, firstShelfPath.c_str(), false)) {
            Serial.println("Successfully updated firstFloor to false.");
        } else {
            Serial.print("Failed to update firstFloor. Reason: ");
            Serial.println(fbdo.errorReason());
        }
    } else if (secondFloorEmpty) {
        if (Firebase.RTDB.setBool(&fbdo, secondShelfPath.c_str(), false)) {
            Serial.println("Successfully updated secondFloor to false.");
        } else {
            Serial.print("Failed to update secondFloor. Reason: ");
            Serial.println(fbdo.errorReason());
        }
    }
}

void checkAndMoveStepper(String shelfColor) {
  // Correct the case to match your Firebase structure.
  String firstShelfPath = "/" + shelfColor + "Shelf/firstFloor";
  String secondShelfPath = "/" + shelfColor + "Shelf/secondFloor";
  
  if (Firebase.RTDB.getBool(&fbdo, firstShelfPath.c_str())) {
    firstFloorEmpty = fbdo.boolData();
  }
  if (Firebase.RTDB.getBool(&fbdo, secondShelfPath.c_str())) {
    secondFloorEmpty = fbdo.boolData();
  }
}


void rotateStepperMotor(int numRevolutions, bool clockwise) {
    digitalWrite(dirPin, clockwise ? HIGH : LOW); // Set direction

    long totalSteps = stepsPerRevolution * numRevolutions; // Calculate total steps

    for (long x = 0; x < totalSteps; x++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2000);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(2000);
    }

    delay(1000); // Pause before returning
}

void handleDistanceUpdate() {
  if (server.hasArg("distance1") && server.hasArg("distance2") && server.hasArg("distance3")) {
    distance1 = server.arg("distance1").toFloat();
    distance2 = server.arg("distance2").toFloat();
    distance3 = server.arg("distance3").toFloat();
    server.send(200, "text/plain", "Distances updated");
  } else {
    server.send(400, "text/plain", "Invalid request");
  }
}




