// Testing wireless connection using bluetooth serial 
// Device 1: contains IMU-based controller
// ESP-32 with arudino connection 

#include <Arduino.h>
#include <Wire.h>
#include <BluetoothSerial.h>  // Bluetooth Serial library for ESP32


// Define pins for motor control
const int ENA = 14;  // Pin for controlling the speed of the left motor (A)
const int IN1 = 19;  // Pin for direction of the left motor (A)
const int IN2 = 18;  // Pin for direction of the left motor (A)


const int ENB = 12;  // Pin for controlling the speed of the right motor (B)
const int IN3 = 17;  // Pin for direction of the right motor (B)
const int IN4 = 16;  // Pin for direction of the right motor (B)


// PWM setup
const int ledcChannelA = 0;  // PWM channel for left motor
const int ledcChannelB = 1;  // PWM channel for right motor
const int ledcFreq = 5000;    // PWM frequency
const int ledcResolution = 8; // 8-bit resolution (0-255)


BluetoothSerial SerialBT; // Bluetooth Serial object
bool isConnected = false;  // Connection status


void setup() {
  // Initialize Serial for debugging
  Serial.begin(115200);


  // Initialize Bluetooth Serial
  SerialBT.begin("ESP32_Motor_Control");  // Bluetooth device name
  Serial.println("Bluetooth device started, you can pair with ESP32_Motor_Control");


  // Initialize motor control pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


  // Configure PWM channels
  ledcSetup(ledcChannelA, ledcFreq, ledcResolution);
  ledcSetup(ledcChannelB, ledcFreq, ledcResolution);
 
  // Attach the PWM channels to the motor speed pins
  ledcAttachPin(ENA, ledcChannelA);
  ledcAttachPin(ENB, ledcChannelB);


  // Set initial motor speeds to zero
  ledcWrite(ledcChannelA, 0);
  ledcWrite(ledcChannelB, 0);
}


// Function to stop the motors
void stopMotors() {
  ledcWrite(ledcChannelA, 0);
  ledcWrite(ledcChannelB, 0);
}


// Function to move motors forward at specified speed
void moveForward(int speed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ledcChannelA, speed);
  ledcWrite(ledcChannelB, speed);
}


// Function to move motors backward at specified speed
void moveBackward(int speed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  ledcWrite(ledcChannelA, speed);
  ledcWrite(ledcChannelB, speed);
}


// Function to turn left by stopping the left motor and running the right motor
void turnLeft(int speed) {
  ledcWrite(ledcChannelA, 0);  // Stop left motor
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  ledcWrite(ledcChannelB, speed);  // Right motor forward
}


// Function to turn right by stopping the right motor and running the left motor
void turnRight(int speed) {
  ledcWrite(ledcChannelB, 0);  // Stop right motor
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  ledcWrite(ledcChannelA, speed);  // Left motor forward
}


void loop() {
  // Check if the device is connected
  if (SerialBT.connected()) {
    if (!isConnected) {
      isConnected = true;
      Serial.println("Client connected");
    }


    if (SerialBT.available()) {
      char command = SerialBT.read();  // Read the incoming Bluetooth command
      int speed = 128;  // Set a default speed of half (128 out of 255)


      Serial.print("Received: ");
      Serial.println(command);


      switch (command) {
        case 'F':  // Forward
          moveForward(speed);
          Serial.println("Moving Forward");
          break;
        case 'B':  // Backward
          moveBackward(speed);
          Serial.println("Moving Backward");
          break;
        case 'L':  // Left turn
          turnLeft(speed);
          Serial.println("Turning Left");
          break;
        case 'R':  // Right turn
          turnRight(speed);
          Serial.println("Turning Right");
          break;
        case 'S':  // Stop
          stopMotors();
          Serial.println("Stopping Motors");
          break;
        default:
          Serial.println("Unknown command");
          break;
      }
    }
  } else {
    // Reset connection status if disconnected
    if (isConnected) {
      isConnected = false;
      Serial.println("Client disconnected");
    }
  }
}
