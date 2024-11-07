// GOAL: send a simple message over MQTT protocol 
// Using a public broker 


#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>


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


void setup() {
    Serial.begin(115200);
    connectToWiFi();
    client.setServer(mqttServer, mqttPort);


    // Initial message to indicate setup completion
    Serial.println("Setup complete, ready to send messages");
}


void loop() {
    if (!client.connected()) {
        reconnect();
    }
    client.loop();


    // Send message every 2 seconds for testing
    static unsigned long lastSendTime = 0;
    if (millis() - lastSendTime > 2000) {
        Serial.println("Attempting to send message: spin_motors");


        // Send the "spin_motors" command to Device 2
        if (client.publish(publishTopic, "spin_motors")) {
            Serial.println("Message successfully sent: spin_motors");
        } else {
            Serial.println("Failed to send message: spin_motors");
            Serial.print("MQTT state: ");
            Serial.println(client.state());  // Print the MQTT connection state
        }
        lastSendTime = millis();
    }


    // Add a small delay to avoid flooding the Serial output too quickly
    delay(200);
}


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
