import cv2
import json
import time
import logging
from ultralytics import YOLO
import paho.mqtt.client as mqtt
import numpy as np
import tensorflow as tf

# Configurations and Constants
class Config:
    MQTT_BROKER = "test.mosquitto.org"  # Replace with actual broker IP
    CONTROL_TOPIC = "bot/motors"
    Kp = 0.1  # Proportional gain
    Kd = 0.05  # Derivative gain
    TARGET_IMAGE_WIDTH = 640  # Consistent image width
    SAFE_ZONE_RADIUS = 50  # Safe zone radius in pixels
    TARGET_FPS = 10  # Target FPS for frame processing

# Logger setup
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Motor Control Class
class MotorControl:
    def __init__(self, mqtt_client, control_topic):
        self.client = mqtt_client
        self.control_topic = control_topic

    def _publish_command(self, pwm_values):
        payload = json.dumps({"pwm": pwm_values})
        self.client.publish(self.control_topic, payload)
        logger.info(f"Published motor command: {payload}")

    def move(self, left_pwm, right_pwm):
        self._publish_command([left_pwm, right_pwm])

# Vision System Class
class VisionSystem:
    def __init__(self):
        self.current_model = None
        self.current_model_name = ""
        self.PERSON_ID = 0  # Assuming detection of 'person'
        self.OBJECT_ID = 39  # Assuming detection of 'bottle'
        self.CELL_PHONE_ID = 67  # Assuming detection of 'cell phone', confirm the correct class ID for your model
        self.prev_wrist_position = None # To store the previous wrist position for motion 


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

        # # Debugging the detected boxes and class IDs
        # print("Detected objects:")
        # for box in results[0].boxes:
        #     print(f"Class: {box.cls}, Coordinates: {box.xyxy}, Confidence: {box.conf}")

        people_boxes = [box for box in results[0].boxes if box.cls == self.PERSON_ID]
        bottle_boxes = [box for box in results[0].boxes if box.cls == self.OBJECT_ID]
        cell_phone_boxes = [box for box in results[0].boxes if box.cls == self.CELL_PHONE_ID]  # Check for cell phones
        # Debug: Print the detected number of each object
        # print(f"Detected {len(people_boxes)} people")
        # print(f"Detected {len(bottle_boxes)} bottles")
        # print(f"Detected {len(cell_phone_boxes)} cell phones")

        target_box = None
        if people_boxes and bottle_boxes:
            bottle_center_x = int((bottle_boxes[0].xyxy[0][0] + bottle_boxes[0].xyxy[0][2]) / 2)
            closest_person = min(
                people_boxes,
                key=lambda person_box: abs(int((person_box.xyxy[0][0] + person_box.xyxy[0][2]) / 2) - bottle_center_x)
            )
            target_box = closest_person  # The closest person to the bottle
            
        # Check if cell phone is found
        # if cell_phone_boxes:
        #     logger.info("Cell phone found")  # Print message if a cell phone is detected

        return results[0].plot(), people_boxes, bottle_boxes, target_box

    def process_pose_frame(self, frame):
        # Ensure `self.model` is properly defined or replace with self.current_model if intended
        if self.current_model is None:
            logger.error("Model is not loaded for pose processing")
            return frame, None  # Return the frame with no processing

        results = self.current_model(frame)  # Use self.current_model instead of self.model
        person_with_raised_hand = None

        # Check if keypoints exist in the results
        if hasattr(results[0], 'keypoints') and results[0].keypoints is not None:
            for keypoints in results[0].keypoints:
                if is_hand_raised(keypoints):  # Define this function based on your criteria
                    person_with_raised_hand = keypoints
                    break

        return results[0].plot(), person_with_raised_hand


    
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

# def is_hand_raised(elbow, shoulder, wrist, elbow_angle, shoulder_angle, angle_offset=0):
#     hand_above_shoulder = wrist[1] < shoulder[1]
#     elbow_flexed = elbow_angle < (90 + angle_offset)
#     shoulder_positioned_for_raise = shoulder_angle < (60 + angle_offset)
#     return hand_above_shoulder and elbow_flexed and shoulder_positioned_for_raise

# def is_hand_raised(pose_results):
#     try:
#         print(f"Number of keypoints detected: {len(pose_results)}")
#         if len(pose_results) > 9:  # Check if the expected number of keypoints is returned
#             print(f"Detected keypoints: {pose_results}")
#             wrist_keypoint = pose_results[9]  # Assuming index 9 is for the wrist
            
#             if wrist_keypoint and wrist_keypoint['confidence'] > 0.5:
#                 if wrist_keypoint['y'] < 0.5:  # Example condition for hand raised
#                     return True  # Hand is raised
#                 else:
#                     return False  # Hand is not raised
#             else:
#                 print("Wrist keypoint not detected or confidence too low.")
#                 return False
#         else:
#             print(f"Pose result has fewer keypoints than expected: {len(pose_results)}")
#             return False
#     except Exception as e:
#         print(f"Error during pose processing: {e}")
#         return False

def is_hand_raised(pose_results):
    """
    Check if the hand is raised based on keypoint data.
    
    Parameters:
    - hand_keypoints: A tensor of shape (N, 2), where N is the number of keypoints, 
                      and each keypoint is a 2D point (x, y).
    
    Returns:
    - A tensor indicating whether the hand is raised (True/False).
    """

    # Assuming keypoint 0 is the wrist (or base of the hand) and keypoint 1 is a finger.
    head = tf.gather(pose_results, 0, axis=0)  # Get the wrist keypoint
    hand = tf.gather(pose_results, 9, axis=0)  # Get the finger keypoint (example)

    head_y = head[1]  # Assuming the second element is the y-coordinate
    hand_y = hand[1]

    # Check if the finger is above the wrist (hand raised)
    is_raised = tf.greater(hand_y, head_y)  # True if the finger is higher than the wrist

    return is_raised

# MQTT Handler Class
class MQTTHandler:
    def __init__(self, broker):
        self.client = mqtt.Client()
        self.client.connect(broker)
        self.client.loop_start()

# Main loop function
def main():
    vid = cv2.VideoCapture(0)  # Open the camera
    mqtt_client = mqtt.Client()
    mqtt_client.connect(Config.MQTT_BROKER)  # Connect to the MQTT broker
    motor_control = MotorControl(mqtt_client, Config.CONTROL_TOPIC)
    vision_system = VisionSystem()
    mqtt_handler = MQTTHandler(Config.MQTT_BROKER)

    mode = 'pose'  # Start in pose mode
    vision_system.load_model('yolo11s-pose')  # Load the pose detection model initially

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

        elif mode == 'pose':
            try:
                processed_frame, person_with_raised_hand = vision_system.process_pose_frame(frame)
                logger.info(f"Person with raised hand: {bool(person_with_raised_hand)}")
            except Exception as e:
                logger.error(f"Error during pose processing: {e}")

        # Calculate FPS
        current_time = time.time()
        fps = 1 / (current_time - previous_time)
        previous_time = current_time

        # Display FPS on the processed frame
        cv2.putText(processed_frame, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # Send MQTT commands based on detection
        if target_box:
            logger.info("Target box detected, sending motor commands...")
            # Add logic to calculate and send motor commands based on target_box
            motor_control.move(50, 50)  # Example command
            pass
        else:
            print(" ") # No target box found 

        # Check for key presses without blocking frame capture
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            logger.info("Exiting...")
            break
        elif key == ord('t'):
            mode = 'detection'  # Switch to detection mode
            vision_system.load_model('yolo11s')  # Switch to detection model
            logger.info("Switched to detection mode.")
        elif key == ord('p'):
            mode = 'pose'  # Switch to pose mode
            vision_system.load_model('yolo11s-pose')  # Switch to pose detection model
            logger.info("Switched to pose mode.")

        # Show the processed frame
        cv2.imshow("Processed Frame", processed_frame)

    vid.release()
    cv2.destroyAllWindows()
    mqtt_client.disconnect()


if __name__ == '__main__':
    main()
