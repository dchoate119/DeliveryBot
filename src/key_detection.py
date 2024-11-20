import cv2
import json
import time
import logging
from ultralytics import YOLO
import paho.mqtt.client as mqtt
import numpy as np

# Configurations and Constants
class Config:
    MQTT_BROKER = "test.mosquitto.org"  # Replace with actual broker IP
    MQTT_PORT = 1883
    CONTROL_TOPIC = "bot/motors"
    # Kp = 0.1  # Proportional gain
    # Kd = 0.05  # Derivative gain
    TARGET_IMAGE_WIDTH = 640  # Consistent image width
    SAFE_ZONE_RADIUS = 50  # Safe zone radius in pixels
    TARGET_FPS = 10  # Target FPS for frame processing

# Logger setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


# Vision System Class
class VisionSystem:
    def __init__(self, mqtt_client):
        self.current_model = None
        self.current_model_name = ""
        self.PERSON_ID = 0  # Assuming detection of 'person'
        self.OBJECT_ID = 39  # Assuming detection of 'bottle'
        self.CELL_PHONE_ID = 67  # Assuming detection of 'cell phone', confirm the correct class ID for your model
        self.prev_wrist_position = None # To store the previous wrist position for motion 
        self.mqtt_client = mqtt_client


    def load_model(self, model_name):
        if self.current_model_name != model_name:
            model_path = f"yolo_models/{model_name}.pt"
            self.current_model = YOLO(model_path, verbose=False)
            self.current_model_name = model_name
            logger.info(f"Model '{model_name}' loaded successfully.")

    def process_detection_frame(self, frame):
        target_height = int(Config.TARGET_IMAGE_WIDTH * frame.shape[0] / frame.shape[1])
        resized_frame = cv2.resize(frame, (Config.TARGET_IMAGE_WIDTH, target_height))
        results = self.current_model(resized_frame, conf=0.6, verbose=False)

        logger.info("Processing detection frame")


        people_boxes = [box for box in results[0].boxes if box.cls == self.PERSON_ID]
        bottle_boxes = [box for box in results[0].boxes if box.cls == self.OBJECT_ID]
        cell_phone_boxes = [box for box in results[0].boxes if box.cls == self.CELL_PHONE_ID]  # Check for cell phones

        target_box = None
        if people_boxes and bottle_boxes:
            bottle_center_x = int((bottle_boxes[0].xyxy[0][0] + bottle_boxes[0].xyxy[0][2]) / 2)
            closest_person = min(
                people_boxes,
                key=lambda person_box: abs(int((person_box.xyxy[0][0] + person_box.xyxy[0][2]) / 2) - bottle_center_x)
            )
            target_box = closest_person  # The closest person to the bottle
            
        # Check if cell phone is found
        if cell_phone_boxes:
            logger.info("Cell phone found")  # Print message if a cell phone is detected
            self.mqtt_client.publish(Config.CONTROL_TOPIC, "DETECTED")
            logger.info("Sent 'DETECTED' over MQTT")

        return results[0].plot(), people_boxes, bottle_boxes, target_box

    def process_pose_frame(self, frame):
        # Ensure `self.model` is properly defined or replace with self.current_model if intended
        if self.current_model is None:
            logger.error("Model is not loaded for pose processing")
            return frame, None  # Return the frame with no processing


    
# Additional Helper Functions
def calculate_angle(p1, p2, p3):
    a = np.array(p1)
    b = np.array(p2)
    c = np.array(p3)
    ba = a - b
    bc = c - b
    norm_product = np.linalg.norm(ba) * np.linalg.norm(bc)
    
    # Check for zero denominator to avoid invalid value for cosine
    if norm_product == 0:
        return 0  # Return 0 or another default value if points are collinear or overlapping

    cosine_angle = np.dot(ba, bc) / norm_product
    angle = np.degrees(np.arccos(np.clip(cosine_angle, -1.0, 1.0)))  # Clip to handle any floating-point precision errors
    return angle


# MQTT Handler Class
class MQTTHandler:
    def __init__(self, broker, port):
        self.client = mqtt.Client()
        self.client.connect(broker, port)
        self.client.loop_start()

# Main loop function
def main():
    vid = cv2.VideoCapture(0)  # Open the camera
    mqtt_client = mqtt.Client()
    mqtt_client.connect(Config.MQTT_BROKER, Config.MQTT_PORT)  # Connect to the MQTT broker
    mqtt_client.loop_start()

    vision_system = VisionSystem(mqtt_client)

    # mqtt_handler = MQTTHandler(Config.MQTT_BROKER, Config.MQTT_PORT)


    # Start in detection mode instead of pose mode
    mode = 'detection'  # Start in detection mode
    vision_system.load_model('yolo11s')  # Load the detection model initially

    previous_time = time.time()  # Initialize FPS calculation

    while True:
        ret, frame = vid.read()
        if not ret:
            break
            # logger.error("Failed to capture frame from camera.")
            # continue

        processed_frame = frame  # Default to the original frame
        target_box = None
        if mode == 'detection':
            try:
                processed_frame, people_boxes, bottle_boxes, target_box = vision_system.process_detection_frame(frame)
                # logger.info(f"People detected: {len(people_boxes)}, Bottles detected: {len(bottle_boxes)}")
            except Exception as e:
                logger.error(f"Error during detection processing: {e}")

        # elif mode == 'pose':
        #     try:
        #         processed_frame, person_with_raised_hand = vision_system.process_pose_frame(frame)
        #         logger.info(f"Person with raised hand: {bool(person_with_raised_hand)}")
        #     except Exception as e:
        #         logger.error(f"Error during pose processing: {e}")

        # Calculate FPS
        current_time = time.time()
        fps = 1 / (current_time - previous_time)
        previous_time = current_time

        # Display FPS on the processed frame
        cv2.putText(processed_frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            logger.info("Exiting...")
            break
        # elif key == ord('t'):
        #     mode = 'detection'  # Switch to detection mode
        #     vision_system.load_model('yolo11s')  # Switch to detection model
        #     logger.info("Switched to detection mode.")
        # elif key == ord('p'):
        #     mode = 'pose'  # Switch to pose mode
        #     vision_system.load_model('yolo11s-pose')  # Switch to pose detection model
        #     logger.info("Switched to pose mode.")

        # Show the processed frame
        cv2.imshow("Processed Frame", processed_frame)

    vid.release()
    cv2.destroyAllWindows()
    mqtt_client.disconnect()


if __name__ == '__main__':
    main()
