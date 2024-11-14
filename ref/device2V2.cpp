// Device 2: contains DC motors and LIDAR
// Receives angle measurements from device 1 over MQTT


#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>  // Include Wire library for I2C
#include <MPU9250.h>


// WiFi and MQTT setup
const char* ssid = "Tufts_Robot";  // Replace with your WiFi SSID
const char* mqttServer = "test.mosquitto.org";  // MQTT broker address
const int mqttPort = 1883;          // MQTT broker port


WiFiClient espClient;
PubSubClient client(espClient);


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


// Function declarations
void connectToWiFi();
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);


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
}


void loop() {
    // Process MQTT messages
    client.loop();
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
        } else {
            Serial.print("Failed to connect, return code: ");
            Serial.print(client.state());
            delay(2000);
        }
    }
}


// Callback function to handle incoming messages
void callback(char* topic, byte* payload, unsigned int length) {
    // Convert the payload to a string
    String message = "";
    for (int i = 0; i < length; i++) {
        message += (char)payload[i];
    }


    Serial.print("Message received: ");
    Serial.println(message);


    // Parse the X and Y angles from the received message
    // Parse JSON data
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, message);
   
    if (error) {
        Serial.print("JSON parse failed: ");
        Serial.println(error.c_str());
        return;
    }


    // Extract angleY and angleX from the JSON message
    float angleY = doc["angleY"];
    float angleX = doc["angleX"];
   
    Serial.print("Y-axis angle: ");
    Serial.println(angleY);
    Serial.print("X-axis angle: ");
    Serial.println(angleX);


    // Map the angles to motor speeds
    int speedY = map(constrain(abs(angleY), 0, 45), 0, 45, 0, 255);
    int speedX = map(constrain(abs(angleX), 0, 45), 0, 45, 0, 255);
   
    // Set motor direction and speed for forward/backward movement (X-axis)
    if (angleX > 0) {  // Forward
        digitalWrite(motorPin1, LOW);  // Motor 1 forward
        digitalWrite(motorPin2, HIGH);
        digitalWrite(motorPin3, HIGH);  // Motor 2 forward
        digitalWrite(motorPin4, LOW);
    } else {  // Backward
        digitalWrite(motorPin1, HIGH);   // Motor 1 backward
        digitalWrite(motorPin2, LOW);
        digitalWrite(motorPin3, LOW);   // Motor 2 backward
        digitalWrite(motorPin4, HIGH);
    }


    // Apply X-angle speed to both motors
    ledcWrite(pwmChannel1, speedX);
    ledcWrite(pwmChannel2, speedX);


    // // Set motor direction for turning based on X-axis
    // if (angleY != 0) {
    //     if (angleY > 0) {  // Right turn
    //         digitalWrite(motorPin3, HIGH);  // Motor 2 forward
    //         digitalWrite(motorPin4, LOW);
    //     } else {  // Left turn
    //         digitalWrite(motorPin3, LOW);   // Motor 2 backward
    //         digitalWrite(motorPin4, HIGH);
    //     }
    //     // Apply turn speed based on X-angle (only on Motor 2)
    //     ledcWrite(pwmChannel2, speedY);


   
    Serial.print("SPEED");
    Serial.println(speedX);
    // // Set motor direction and speed based on angle signs
    // digitalWrite(motorPin1, angleY > 0 ? HIGH : LOW);
    // digitalWrite(motorPin2, angleY > 0 ? LOW : HIGH);
    // digitalWrite(motorPin3, angleX > 0 ? HIGH : LOW);
    // digitalWrite(motorPin4, angleX > 0 ? LOW : HIGH);


    // // Apply PWM speed values
    // ledcWrite(pwmChannel1, speedY);
    // ledcWrite(pwmChannel2, speedX);
   
}


