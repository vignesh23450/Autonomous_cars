import cv2
import numpy as np
from picamera2 import Picamera2
import RPi.GPIO as GPIO
import time

# GPIO pin setup
GPIO_PINS = {'IN1': 17, 'IN2': 27, 'IN3': 22, 'IN4': 23, 'ENA': 12, 'ENB': 13}
GPIO.setmode(GPIO.BCM)
GPIO.setup(list(GPIO_PINS.values()), GPIO.OUT)

# Setup PWM for motor speed control
pwmA, pwmB = GPIO.PWM(GPIO_PINS['ENA'], 1000), GPIO.PWM(GPIO_PINS['ENB'], 1000)
pwmA.start(0)
pwmB.start(0)

def set_motor_speed(motor, speed):
    """Set the speed of the specified motor."""
    (pwmA if motor == 'A' else pwmB).ChangeDutyCycle(speed)

def motor_control(motor, direction, speed):
    """Control the direction and speed of the specified motor."""
    in1, in2 = (GPIO_PINS['IN1'], GPIO_PINS['IN2']) if motor == 'A' else (GPIO_PINS['IN3'], GPIO_PINS['IN4'])
    GPIO.output(in1, GPIO.HIGH if direction == 'forward' else GPIO.LOW)
    GPIO.output(in2, GPIO.LOW if direction == 'forward' else GPIO.HIGH)
    set_motor_speed(motor, speed)

# Lane and camera parameters
height, width = 480, 640
lane_coordinates = np.float32([(115, 358), (500, 358), (639, 410), (9, 410)])
lane_coordinates2 = np.float32([(140, 0), (500, 0), (500, 480), (140, 480)])
M = cv2.getPerspectiveTransform(lane_coordinates, lane_coordinates2)

# Initialize PiCamera2 object
piCam = Picamera2()
piCam.preview_configuration.main.size = (width, height)
piCam.preview_configuration.main.format = "RGB888"
piCam.preview_configuration.align()
piCam.configure("preview")
piCam.start()

def load_detection_files():
    """Load object detection files."""
    class_file_path = "/home/vignesh/Desktop/Object_Detection_Files/coco.names"
    config_path = "/home/vignesh/Desktop/Object_Detection_Files/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt"
    weights_path = "/home/vignesh/Desktop/Object_Detection_Files/frozen_inference_graph.pb"
    with open(class_file_path, "rt") as f:
        class_names = f.read().rstrip("\n").split("\n")
    return class_names, config_path, weights_path

classNames, configPath, weightsPath = load_detection_files()
net = cv2.dnn_DetectionModel(weightsPath, configPath)
net.setInputSize(300, 300)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

calculate_center = lambda x, w: x + (w // 2)
calculate_distance = lambda centerY: (0.3) * (height - centerY) + (-48.6)

def detect_lane(frame):
    """Detect lane lines and calculate lane deviation."""
    warped = cv2.warpPerspective(frame, M, (width, height))
    gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    histogram_lane = np.sum(edges, axis=0)

    left_lane_position = np.argmax(histogram_lane[:width // 2])
    right_lane_position = np.argmax(histogram_lane[width // 2:]) + width // 2
    lane_center = (left_lane_position + right_lane_position) // 2
    frame_center = width // 2
    deviation = lane_center - frame_center

    # Draw lane lines
    cv2.line(frame, (left_lane_position, 0), (left_lane_position, height), (0, 255, 0), 2)
    cv2.line(frame, (right_lane_position, 0), (right_lane_position, height), (0, 255, 0), 2)
    cv2.line(frame, (lane_center, 0), (lane_center, height), (255, 0, 0), 2)

    return frame, deviation, left_lane_position, right_lane_position

def detect_objects(frame, process_frame_counter, left_x, right_x):
    """Detect objects and return modified frame with object bounding boxes."""
    if process_frame_counter % 5 != 0:
        return frame, []

    classIds, confs, bbox = net.detect(frame, confThreshold=0.4, nmsThreshold=0.4)
    objectInfo = []

    if isinstance(classIds, np.ndarray):
        for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
            x, y, w, h = box
            centerX, centerY = calculate_center(x, w), calculate_center(y, h)
            dist_object = calculate_distance(centerY)

            # Determine color and motor action based on distance
            if dist_object <= 20:
                box_color = (0, 0, 255)  # Red
                motor_control('A', 'stop', 0)
                motor_control('B', 'stop', 0)
            elif 20 < dist_object <= 50:
                box_color = (0, 255, 255)  # Yellow
                motor_control('A', 'forward', 50)
                motor_control('B', 'forward', 50)
            else:
                box_color = (0, 255, 0)  # Green
                motor_control('A', 'forward', 80)
                motor_control('B', 'forward', 80)

            cv2.rectangle(frame, box, color=box_color, thickness=2)
            cv2.putText(frame, "Obstacle", (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Dist: {dist_object:.1f} cm", (x + 5, y + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            objectInfo.append((box, classNames[classId - 1], (centerX, centerY), dist_object, box_color))

    return frame, objectInfo

process_frame_counter = 0

try:
    while True:
        frame = piCam.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        frame, lane_deviation, left_x, right_x = detect_lane(frame)
        frame, _ = detect_objects(frame, process_frame_counter, left_x, right_x)

        # Motor control based on lane deviation
        if lane_deviation == 0:
            motor_control('A', 'forward', 50)
            motor_control('B', 'forward', 50)
        elif 0 < lane_deviation < 10:
            motor_control('A', 'forward', 50)
            motor_control('B', 'backward', 20)
        elif 10 <= lane_deviation < 20:
            motor_control('A', 'forward', 50)
            motor_control('B', 'backward', 50)
        elif lane_deviation > 20:
            motor_control('A', 'forward', 80)
            motor_control('B', 'backward', 80)
        elif -10 < lane_deviation < 0:
            motor_control('A', 'backward', 20)
            motor_control('B', 'forward', 50)
        elif -20 <= lane_deviation < -10:
            motor_control('A', 'backward', 50)
            motor_control('B', 'forward', 50)
        elif lane_deviation < -20:
            motor_control('A', 'backward', 80)
            motor_control('B', 'forward', 80)

        cv2.putText(frame, f"Lane Deviation: {lane_deviation}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.imshow("Lane and Object Detection", frame)

        process_frame_counter += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

except KeyboardInterrupt:
    pass

finally:
    GPIO.cleanup()
    pwmA.stop()
    pwmB.stop()
    cv2.destroyAllWindows()