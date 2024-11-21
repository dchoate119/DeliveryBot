# DeliveryBot
**Wireless, hand motion-controlled delivery robot with facial recognition security and lidar-based collision avoidance**

**Project length:** 3 weeks

**Team Members:** Daniel Choate (myself), Vivian Becker

## How it works: 
Using an IMU-based wearable controller, the user inputs hand movements which map to vehicle motion (pitch and roll angles). The input movements are sent through wireless communication using MQTT protocol and are mapped as input speed for the DC motors to drive the vehicle forwards, backwards, left, and right. The vehicle contains Lidar-based collision avoidance.

### Facial Recognition: 
The lock box on the delivery vehicle is opened through facial recognition software (yolo v11). The user places their phone in the mount on the front of the vehicle, and can use visual keys as a way of unlocking the box. 


---

## Video Demonstration
![Alt Text](Media/delivery_demo.gif)

---

## Run this code: 

**Scripts:** 

1. **<device1.cpp>** uses ESP-32 micro-controller and MQTT wireless communication to send pitch and roll angles from MPU 9250 device to '<device2.cpp>'

2. **<key_detection.py>** Connects to user IPhone through bluetooth, constantly checking for the 'key' set by the user. Once the 'key' is detected (initially set as a 'cell phone' found by the camera), a "DETECTED" message is sent to '<device2.cpp>' through MQTT

3. **<device2.cpp>** Also uses ESP-32 micro-controller to receive angle measurements from '<device1.cpp>'. Angles are mapped as input speed for the DC motors to drive the vehicle forwards, backwards, left, and right. When a "DETECTED" message is received, the servo motor operates a linkage system to open the lock box. Similarly, Lidar distance measurements are continuously read, and the vehicle motors are stopped once the Lidar detects a potential collision. 

---

## Requirements:
'<device1.cpp>' and '<device2.cpp>' can be uploaded to each individual ESP-32 through platformIO in visual studio code with '<PIO_requirements.ini>' pasted in the '.ini' file


## Materials Used:

- **(2)** ESP-32 micro-controller 
- **(1)** MPU-9250
- **(1)** VL53L1X Lidar Sensor
- **(2)** DC motors
- **(1)** L298N motor driver
- **(1)** Servo motor
- **(1)** 12-V battery
- **(1)** Personal phone or camera

*All other structural components were 3D print or laser-cut* 


