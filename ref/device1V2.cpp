// Device 1 contains MPU 9250 which 
// Records input movements from user
// Sends angles to user
// Used for video submission on 11/07 - IT WORKS!
// Basic forward/backward wireless motion using MQTT

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <MPU9250.h>
#include <math.h>  // For defining PI
#include <Wire.h>  // Include Wire library for I2C




MPU9250 mpu;


// Function prototypes
void connectToWiFi();
void reconnect();


const char* ssid = "Tufts_Robot";  // Wi-Fi SSID
const char* password = "";          // No password
const char* mqttServer = "test.mosquitto.org";  // MQTT broker


const int mqttPort = 1883;
const char* publishTopic = "device1/control"; // Topic for sending commands to Device 2


WiFiClient espClient;
PubSubClient client(espClient);


float angularVelocityY = 0;  // Y-axis angular velocity in deg/s
float angularVelocityX = 0;  // X-axis angular velocity in deg/s
float angleY = 0;  // Y-axis angle in degrees (integrated value)
float angleX = 0;  // X-axis angle in degrees (integrated value)
unsigned long lastTime = 0;


// Define a variable to store the gyroscope Y-axis bias
float gyroYBias = 0.0;
float gyroXBias = 0.0;
const char resetKey = '\r';  // Enter key (Carriage Return)




// Function to reset the angle
void resetAngle() {
    angleY = 0.0;  // Reset angle to 0
    angleX = 0.0;
    Serial.println("Angle reset to zero");
}


// // Function to map the angle to motor speed
// int mapAngleToSpeed(float angle) {
//     // Map angle range (0-45) to speed range (0-255)
//     return map(angle, 0, 45, 0, 255);
// }




void setup() {
    Serial.begin(115200);
    connectToWiFi();
    client.setServer(mqttServer, mqttPort);


    // Initialize I2C communication with custom pins
    Wire.begin(21, 22);  // SDA = 21, SCL = 22 (change these if necessary)




    // Initialize the MPU9250 sensor
    if (!mpu.setup(0x68)) {  // Use I2C address 0x68 for MPU9250
        Serial.println("Failed to initialize MPU9250");
        while (1);  // Hang if initialization fails
    }
    Serial.println("MPU9250 initialized");


    // Get the initial gyroscope bias (average of several readings)
    float gyroYSum = 0.0;
    float gyroXSum = 0.0;


    const int sampleCount = 100;  // Take 100 samples to average
    for (int i = 0; i < sampleCount; i++) {
        mpu.update();
        gyroYSum += mpu.getGyroY();  // Sum up the Y-axis gyro readings
        gyroXSum += mpu.getGyroX();  // Sum up the X-axis gyro readings


        delay(10);  // Wait a little between readings
    }
    gyroYBias = gyroYSum / sampleCount;  // Calculate the average value
    gyroXBias = gyroXSum / sampleCount;


    Serial.print("Gyro Y-axis bias: ");
    Serial.println(gyroYBias);
    Serial.print("Gyro X-axis bias: ");
    Serial.println(gyroXBias);




    // Initial message to indicate setup completion
    Serial.println("Setup complete, ready to send messages");
}


unsigned long lastLoopTime = 0;  // Store the last loop time
const unsigned long loopInterval = 10;  // 100 Hz = 10 ms per loop




void loop() {
    unsigned long currentTime = millis();
   
    // Check if enough time (10 ms) has passed since the last loop iteration
    if (currentTime - lastLoopTime >= loopInterval) {
        lastLoopTime = currentTime;  // Update the last loop time


        if (!client.connected()) {
            reconnect();
        }
        client.loop();


        // Get gyro data from MPU9250
        mpu.update();  // Update sensor data


        // Calculate the time difference (in seconds) for integration
        float deltaTime = loopInterval / 1000.0;  // Convert milliseconds to seconds


        // Get the Y-axis angular velocity in degrees per second
        angularVelocityY = mpu.getGyroY() - gyroYBias;  // Y-axis angular velocity
        angularVelocityX = mpu.getGyroX() - gyroXBias;  // X-axis angular velocity




        // Integrate the angular velocity to get the angle (in degrees)
        angleY += angularVelocityY * deltaTime;
        angleX += angularVelocityX * deltaTime;


        // Print the X and Y angles
        // Serial.print("Angle Y: ");
        // Serial.print(angleY);
        // Serial.print(" degrees, Angle X: ");
        // Serial.print(angleX);
        // Serial.println(" degrees");






        // Check if the user presses Enter to reset the angle
        if (Serial.available() > 0) {
            char incomingChar = Serial.read();
            if (incomingChar == resetKey) {
                resetAngle();  // Reset the angle to zero if Enter is pressed
            }
        }
        // Publish the angles to Device 2 as a JSON message
        char message[50];
        snprintf(message, sizeof(message), "{\"angleY\": %.2f, \"angleX\": %.2f}", angleY, angleX);
       
        if (client.publish(publishTopic, message)) {
            Serial.print("Message successfully sent: ");
            Serial.println(message);
        } else {
            Serial.println("Failed to send message");
            Serial.print("MQTT state: ");
            Serial.println(client.state());  // Print the MQTT connection state
        }
    }
}    


//         // Send the "spin_motors" command to Device 2
//         if (client.publish(publishTopic, "spin_motors")) {
//             Serial.println("Message successfully sent: spin_motors");
//         } else {
//             Serial.println("Failed to send message: spin_motors");
//             Serial.print("MQTT state: ");
//             Serial.println(client.state());  // Print the MQTT connection state
//         }
//     }
// }    
   


void connectToWiFi() {
    Serial.print("Connecting to WiFi...");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("Connected to WiFi");
}


void reconnect() {
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Use a unique client ID
        String clientId = "Device1-";
        clientId += String(random(0xffff), HEX);
        Serial.print("Client ID: ");
        Serial.println(clientId);


        if (client.connect(clientId.c_str())) {
            Serial.println("Connected to MQTT broker");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" trying again in 2 seconds...");
            delay(2000);
        }
    }
}


