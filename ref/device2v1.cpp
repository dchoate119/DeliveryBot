// GOAL: recieve the simple message over mqtt protocol
// Using a public broker
// Upon receival of the message, spin the motors

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>


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


    // Spin motors if the message is "spin_motors"
    if (message == "spin_motors") {
        Serial.println("Spinning motors...");
       
        // Turn on the motors (set the correct input pins to HIGH)
        digitalWrite(motorPin1, HIGH);  // IN1 = HIGH
        digitalWrite(motorPin2, LOW);   // IN2 = LOW
        digitalWrite(motorPin3, HIGH);  // IN3 = HIGH
        digitalWrite(motorPin4, LOW);   // IN4 = LOW
       
        // Control motor speed using PWM on enable pins
        ledcWrite(pwmChannel1, 255);  // Enable motor 1 with full speed (255)
        ledcWrite(pwmChannel2, 255);  // Enable motor 2 with full speed (255)


        delay(2000); // Spin motors for 2 seconds
       
        // Stop motors
        ledcWrite(pwmChannel1, 0);    // Disable motor 1 (no speed)
        ledcWrite(pwmChannel2, 0);    // Disable motor 2 (no speed)


        // Stop the motors
        digitalWrite(motorPin1, LOW);   // IN1 = LOW
        digitalWrite(motorPin2, LOW);   // IN2 = LOW
        digitalWrite(motorPin3, LOW);   // IN3 = LOW
        digitalWrite(motorPin4, LOW);   // IN4 = LOW
    }
}


