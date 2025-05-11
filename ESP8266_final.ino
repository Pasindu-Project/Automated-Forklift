#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

// Wi-Fi credentials
#define WIFI_SSID       "Dialog 4G 800"
#define WIFI_PASSWORD   "62304AAD"

// ESP32 server IP
const char* ESP32_SERVER = "http://192.168.8.173/distance";  // Replace with ESP32's IP

// Ultrasonic Sensor Pins
#define TRIG_PIN1 D1
#define ECHO_PIN1 D2
#define TRIG_PIN2 D3
#define ECHO_PIN2 D0
#define TRIG_PIN3 D5
#define ECHO_PIN3 D6

// Buzzer Pin
#define BUZZER_PIN D7

// Constants
#define DISTANCE_THRESHOLD 20  // Distance threshold in cm
#define HTTP_TIMEOUT 5000      // HTTP timeout in ms
#define RECONNECT_DELAY 10000  // WiFi reconnection attempt delay

bool issend = true;
unsigned long lastReconnectAttempt = 0;

void setup() {
  Serial.begin(115200);
  
  pinMode(TRIG_PIN1, OUTPUT);
  pinMode(ECHO_PIN1, INPUT);
  pinMode(TRIG_PIN2, OUTPUT);
  pinMode(ECHO_PIN2, INPUT);
  pinMode(TRIG_PIN3, OUTPUT);
  pinMode(ECHO_PIN3, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);  // Set buzzer pin as output
  
  // Initialize buzzer to OFF
  digitalWrite(BUZZER_PIN, LOW);
  
  // Connect to Wi-Fi
  connectToWiFi();
}

void loop() {
  // Check WiFi connection
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long currentMillis = millis();
    // Try to reconnect every RECONNECT_DELAY milliseconds
    if (currentMillis - lastReconnectAttempt > RECONNECT_DELAY) {
      lastReconnectAttempt = currentMillis;
      Serial.println("WiFi disconnected. Attempting to reconnect...");
      connectToWiFi();
    }
    return;  // Skip the rest of the loop if not connected
  }
  
  float distance1 = getDistance(TRIG_PIN1, ECHO_PIN1);
  float distance2 = getDistance(TRIG_PIN2, ECHO_PIN2);
  float distance3 = getDistance(TRIG_PIN3, ECHO_PIN3);
  
  Serial.println("----- Distance Readings -----");
  Serial.printf("Sensor 1: %.2f cm\n", distance1);
  Serial.printf("Sensor 2: %.2f cm\n", distance2);
  Serial.printf("Sensor 3: %.2f cm\n", distance3);
  Serial.println("----------------------------");
  
  bool frontObstacle = (distance1 < DISTANCE_THRESHOLD || distance2 < DISTANCE_THRESHOLD);
  bool backObstacle = (distance3 < DISTANCE_THRESHOLD);
  bool allClear = (distance1 > DISTANCE_THRESHOLD && distance2 > DISTANCE_THRESHOLD && distance3 > DISTANCE_THRESHOLD);
  
  // Control buzzer based on distance readings
  if (distance1 < DISTANCE_THRESHOLD || distance2 < DISTANCE_THRESHOLD || distance3 < DISTANCE_THRESHOLD) {
    // If any sensor detects obstacle within threshold, turn buzzer ON
    digitalWrite(BUZZER_PIN, HIGH);
    Serial.println("Buzzer ON - Obstacle detected within threshold");
  } else {
    // If all sensors show clear path, turn buzzer OFF
    digitalWrite(BUZZER_PIN, LOW);
    Serial.println("Buzzer OFF - No obstacles detected");
  }
  
  if (issend) {
    // Send data when front sensors detect obstacle but back is clear, or vice versa
    if ((frontObstacle && !backObstacle) || (!frontObstacle && backObstacle)) {
      if (sendToESP32(distance1, distance2, distance3)) {
        issend = false;  // Only change state if send was successful
        Serial.println("State changed: issend = false");
      }
    }
  } else {  // !issend
    // When all sensors detect no obstacle, send data and reset state
    if (allClear) {
      if (sendToESP32(distance1, distance2, distance3)) {
        issend = true;  // Only change state if send was successful
        Serial.println("State changed: issend = true");
      }
    }
  }
  
  delay(500); // Delay before next reading
}

// Function to connect to WiFi
void connectToWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to WiFi");
  
  // Try for about 20 seconds (40 * 500ms)
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to WiFi");
  }
}

// Function to get distance from an ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Timeout after 23200 microseconds (approximately 4m range)
  long duration = pulseIn(echoPin, HIGH, 23200);
  
  // Check if the pulse timed out
  if (duration == 0) {
    Serial.printf("Warning: Sensor (%d, %d) timed out\n", trigPin, echoPin);
    return 400.0;  // Return a large value to indicate "very far"
  }
  
  float distance = duration * 0.034 / 2;  // Convert to cm
  
  // Basic sanity check
  if (distance < 2.0 || distance > 400.0) {
    Serial.printf("Warning: Unusual reading (%d, %d): %.2f cm\n", trigPin, echoPin, distance);
  } else {
    Serial.printf("Sensor (%d, %d): %.2f cm\n", trigPin, echoPin, distance);
  }
  
  return distance;
}

// Function to send distance values to ESP32
bool sendToESP32(float distance1, float distance2, float distance3) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Error: WiFi not connected. Can't send data.");
    return false;
  }
  
  WiFiClient client;
  HTTPClient http;
  
  String url = String(ESP32_SERVER) + "?distance1=" + String(distance1) + 
                                    "&distance2=" + String(distance2) + 
                                    "&distance3=" + String(distance3);
  Serial.println("Sending Data to ESP32: " + url);
  
  http.begin(client, url);
  http.setTimeout(HTTP_TIMEOUT);  // Set timeout to 5000 ms (5 seconds)
  
  int httpCode = http.GET();
  if (httpCode > 0) {
    String payload = http.getString();
    Serial.printf("HTTP Response code: %d\n", httpCode);
    Serial.println("ESP32 Response: " + payload);
    http.end();
    return true;
  } else {
    Serial.printf("HTTP Error: %s\n", http.errorToString(httpCode).c_str());
    http.end();
    return false;
  }
}