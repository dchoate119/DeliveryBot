#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>  // Include Wire library for I2C
#include <MPU9250.h>
#include <VL53L0X.h>
#include <ESP32Servo.h>




// WiFi and MQTT setup
const char* ssid = "Tufts_Robot";  // Replace with your WiFi SSID
const char* mqttServer = "test.mosquitto.org";  // MQTT broker address
const int mqttPort = 1883;          // MQTT broker port
int currentServoPosition = 0; 

WiFiClient espClient;
PubSubClient client(espClient);
Servo myServo;

// Motor control pins
const int motorPin1 = 19; // IN1 to GPIO19
const int motorPin2 = 18; // IN2 to GPIO18
const int motorPin3 = 17; // IN3 to GPIO17
const int motorPin4 = 16; // IN4 to GPIO16


// Enable pins for motor speed control (PWM)
const int enablePin1 = 12; // Enable motor 1 (GPIO12)
const int enablePin2 = 14; // Enable motor 2 (GPIO14)

// PWM channels and settings
const int pwmFreq = 1000;      // Frequency for PWM signal
const int pwmResolution = 8;   // Resolution for PWM signal (0-255)
const int pwmChannel1 = 0;     // PWM channel for enablePin1
const int pwmChannel2 = 1;     // PWM channel for enablePin2

// VL53L0X sensor setup
VL53L0X sensor;

// Motor control variables
bool stopMotors = false;  // Flag to indicate whether to stop motors based on distance
float lastAngleX = 0, lastAngleY = 0;  // Last received angles from MQTT

// Timing variables
unsigned long lastMillis = 0; // Timestamp for LiDAR reading interval
const long interval = 100;     // Interval to read LiDAR (in milliseconds)

// Function declarations
void connectToWiFi();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);
void setupVL53L0X();
void stopMotorsNow();

void setup() {
    Serial.begin(115200);

    // Initialize motor control pins for PWM
    ledcSetup(pwmChannel1, pwmFreq, pwmResolution); // Motor 1
    ledcSetup(pwmChannel2, pwmFreq, pwmResolution); // Motor 2

    // Attach the PWM signals to the GPIO pins
    ledcAttachPin(enablePin1, pwmChannel1);  // Motor 1 enable
    ledcAttachPin(enablePin2, pwmChannel2);  // Motor 2 enable  

    // Initialize motor control pins
    pinMode(motorPin1, OUTPUT);
    pinMode(motorPin2, OUTPUT);
    pinMode(motorPin3, OUTPUT);
    pinMode(motorPin4, OUTPUT);
    
    // Connect to WiFi
    connectToWiFi();

    // Setup MQTT
    client.setServer(mqttServer, mqttPort);
    client.setCallback(callback); // Set the callback function for incoming messages
    reconnect();

    // Initialize the VL53L0X
    setupVL53L0X();
    // Initialize servo (GIOP 10)
    myServo.attach(27);
    myServo.write(70);
    currentServoPosition = 70;
}

// Function to move the servo to 90 degrees
void moveServoTo90() {
    if (currentServoPosition != 170) {
        myServo.write(170);  // Move the servo to 90 degrees
        currentServoPosition = 170;
    } else {
        Serial.println("Servo is already at 90");
    }
}

void loop() {
    // Process MQTT messages
    client.loop();

    // Periodically read LiDAR data (non-blocking)
    unsigned long currentMillis = millis();
    if (currentMillis - lastMillis >= interval) {
        lastMillis = currentMillis;

        // Read LiDAR distance continuously
        int distance = sensor.readRangeContinuousMillimeters();
        Serial.print("LiDAR Distance: ");
        Serial.print(distance);
        Serial.println(" mm");

        if (sensor.timeoutOccurred()) {
            Serial.println("Sensor timeout!");
        }

        // Stop motors if distance is less than 50 mm
        if (distance < 250) {
            if (!stopMotors) {
                stopMotors = true;
                Serial.println("Distance is less than 50 mm. Stopping motors.");
                stopMotorsNow();
            }

        } else {
            // If distance is greater than 50 mm and motors were stopped, resume movement
            if (stopMotors) {
                stopMotors = false;
                Serial.println("Resuming motor movement.");
            }

            // Control motors based on angle values (received via MQTT)
            if (lastAngleX != 0 && lastAngleY != 0) {

                // New logic to set speedX to 0 when X angle is between -10 and 10 degrees
                int speedX;
                if (lastAngleX > -10 && lastAngleX < 10) {
                    speedX = 0;
                } else {
                    speedX = map(constrain(abs(lastAngleX), 10, 55), 10, 55, 100, 255);
                }

                // int speedX = map(constrain(abs(lastAngleX), 10, 55), 10, 55, 100, 255);

                if (abs(lastAngleY) >= 5) {
                    // Map the angles to motor speeds
                    int speedY = map(constrain(abs(lastAngleY), 5, 45), 5, 45, 100, 200);

                // // Map the angles to motor speeds
                // int speedY = map(constrain(abs(lastAngleY), 0, 45), 5, 45, 50, 100);
                // int speedX = map(constrain(abs(lastAngleX), 0, 45), 0, 45, 0, 255);
                

                    // Set motor direction and speed for forward/backward movement (X-axis)
                    if (lastAngleX > 0) {  // Forward
                        digitalWrite(motorPin1, HIGH);  // Motor 1 forward
                        digitalWrite(motorPin2, LOW);
                        digitalWrite(motorPin3, LOW);  // Motor 2 forward
                        digitalWrite(motorPin4, HIGH);

                        // If Y-angle is negative, add speedY to speedX for Motor 1
                        if (lastAngleY > 0) {
                            ledcWrite(pwmChannel1, speedX + speedY);  // Apply to motor 1
                        } else {
                            ledcWrite(pwmChannel1, speedX);  // Apply only speedX to motor 1
                        }

                        // If Y-angle is positive, apply speedY to motor 2
                        if (lastAngleY < 0) {
                            ledcWrite(pwmChannel2, speedX + speedY);  // Apply to motor 2
                        } else {
                            ledcWrite(pwmChannel2, speedX);  // Apply only speedX to motor 2
                        }
                        Serial.print("Speed X: ");
                        Serial.println(speedX);
                        Serial.print("Speed Y: ");
                        Serial.println(speedY);

                    } else {  // Backward
                        digitalWrite(motorPin1, LOW);   // Motor 1 backward
                        digitalWrite(motorPin2, HIGH);
                        digitalWrite(motorPin3, HIGH);   // Motor 2 backward
                        digitalWrite(motorPin4, LOW);

                        // If Y-angle is negative, add speedY to speedX for Motor 1
                        if (lastAngleY > 0) {
                            ledcWrite(pwmChannel1, speedX + speedY);  // Apply to motor 1
                        } else {
                            ledcWrite(pwmChannel1, speedX);  // Apply only speedX to motor 1
                        }

                        // If Y-angle is positive, apply speedY to motor 2
                        if (lastAngleY < 0) {
                            ledcWrite(pwmChannel2, speedX + speedY);  // Apply to motor 2
                        } else {
                            ledcWrite(pwmChannel2, speedX);  // Apply only speedX to motor 2
                        }

                        Serial.print("Speed X: ");
                        Serial.println(speedX);
                        Serial.print("Speed Y: ");
                        Serial.println(speedY);
                    }
                } else {
                    // When lastAngleY is between -5 and 5, only apply speedX
                    if (lastAngleX > 0) {  // Forward
                        digitalWrite(motorPin1, HIGH);  // Motor 1 forward
                        digitalWrite(motorPin2, LOW);
                        digitalWrite(motorPin3, LOW);  // Motor 2 forward
                        digitalWrite(motorPin4, HIGH);

                        // Apply only speedX to both motors
                        ledcWrite(pwmChannel1, speedX);
                        ledcWrite(pwmChannel2, speedX);
                        Serial.print("Speed X: ");
                        Serial.println(speedX);
                    } else {  // Backward
                        digitalWrite(motorPin1, LOW);   // Motor 1 backward
                        digitalWrite(motorPin2, HIGH);
                        digitalWrite(motorPin3, HIGH);   // Motor 2 backward
                        digitalWrite(motorPin4, LOW);

                        // Apply only speedX to both motors
                        ledcWrite(pwmChannel1, speedX);
                        ledcWrite(pwmChannel2, speedX);
                        Serial.print("Speed X: ");
                        Serial.println(speedX);



                // // Apply X-angle speed to both motors
                // ledcWrite(pwmChannel1, speedX);
                // ledcWrite(pwmChannel2, speedX);

                // Serial.print("Speed X: ");
                // Serial.println(speedX);
                    }
                }
            }
        }
    }
}

// Function to connect to WiFi
void connectToWiFi() {
    WiFi.begin(ssid, "");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi.");
}

// Function to reconnect to the MQTT broker
void reconnect() {
    while (!client.connected()) {
        Serial.println("Attempting MQTT connection...");
        if (client.connect("Device2")) { // Use a unique client ID for Device 2
            Serial.println("Connected to MQTT broker");
            client.subscribe("device1/control"); // Subscribe to the topic
            client.subscribe("bot/motors");
        } else {
            Serial.print("Failed to connect, return code: ");
            Serial.print(client.state());
            delay(2000);
        }
    }
}


void callback(char* topic, byte* payload, unsigned int length) {
    // Convert the payload to a string
    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }


    Serial.print("Message received: ");
    Serial.println(message);

    // Check if the message is "DETECTED"
    if (message.equals("DETECTED")) {
        Serial.println("matched: DETECTED");
        moveServoTo90();
        return; // No need to process further for "DETECTED"
    }

    // Parse the X and Y angles from the received message
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
   
    if (error) {
        Serial.print("JSON parse failed: ");
        Serial.println(error.c_str());
        return;
    }

    // Extract angleY and angleX from the JSON message
    lastAngleY = doc["angleY"];
    lastAngleX = doc["angleX"];
   
    Serial.print("Y-axis angle: ");
    Serial.println(lastAngleY);
    Serial.print("X-axis angle: ");
    Serial.println(lastAngleX);
}


// Function to initialize the VL53L0X sensor
void setupVL53L0X() {

    Wire.begin(21,22);

    sensor.setTimeout(500);

    if (!sensor.init()) {
        Serial.println("Failed to initialize VL53L0X!");
        while (1); // Stay stuck here if the sensor fails to initialize
    }

    sensor.startContinuous(); // Start continuous measurement mode
}
    
// Function to stop the motors immediately
void stopMotorsNow() {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    digitalWrite(motorPin3, LOW);
    digitalWrite(motorPin4, LOW);
    ledcWrite(pwmChannel1, 0);
    ledcWrite(pwmChannel2, 0);
    Serial.println("Motors stopped.");
}
