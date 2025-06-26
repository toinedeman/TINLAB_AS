import cv2
import numpy as np
import paho.mqtt.client as mqtt
from collections import deque
import math
import time
import gc
import json
from heapq import heappush, heappop

# MQTT Setup
MQTT_BROKER = "192.168.4.1"
CONTROL_TOPICS = {1: "robot/control/1", 2: "robot/control/2", 3: "robot/control/3"}
STATUS_TOPICS = {1: "robot/status/1", 2: "robot/status/2", 3: "robot/status/3"}
POSITION_TOPICS = {1: "robot/position/1", 2: "robot/position/2", 3: "robot/position/3"}
FRAME_TOPIC = "robot/frame/dimensions"  # Topic for frame dimensions
TARGET_POSITION_TOPICS = {1: "robot/targetpos/1", 2: "robot/targetpos/2", 3: "robot/targetpos/3"}

# Movement Config
# MIN_DISTANCE = 20
ROBOT_LENGTH = 50
MIN_ROBOT_DISTANCE = 100
SAFETY_DISTANCE = 40

# Robot configurations
ROBOT_CONFIGS = {
    1: {
        "type": "led_pair",
        "front_color": ("green", np.array([35, 50, 50]), np.array([85, 255, 255])),
        "back_color": ("red", [np.array([0, 50, 50]), np.array([10, 255, 255])], 
                      [np.array([170, 50, 50]), np.array([180, 255, 255])]),
        "speeds": {"rotate": 20, "forward": 30, "max": 40},
        "rotate_threshold": 10,
        "min_distance": 10,
        "rotation_threshold": 15
    },
    2: {
        "type": "blue_square",
        "body_color": ("blue", np.array([100, 50, 50]), np.array([130, 255, 255])),
        "front_color": ("green", np.array([35, 50, 50]), np.array([85, 255, 255])),
        "back_color": ("red", [np.array([0, 50, 50]), np.array([10, 255, 255])], 
                      [np.array([170, 50, 50]), np.array([180, 255, 255])]),
        "speeds": {"rotate": 15, "forward": 30, "max": 35},
        "rotate_threshold": 10,
        "min_distance": 30,
        "rotation_threshold": 15
    },
    3: {
        "type": "blue_split_rectangle",
        "front_color": ("green", np.array([35, 50, 50]), np.array([85, 255, 255])),
        "back_color": ("red", [np.array([0, 50, 50]), np.array([10, 255, 255])], 
                      [np.array([170, 50, 50]), np.array([180, 255, 255])]),
        "blue_color": ("blue", np.array([100, 50, 50]), np.array([130, 255, 255])),
        "speeds": {"rotate": 20, "forward": 30, "max": 35},
        "rotate_threshold": 10,
        "min_blue_area": 300,  # Minimum area for each blue segment
        "max_gap_ratio": 0.4,   # Maximum allowed gap between blue segments relative to robot size
        "min_distance": 20,
        "rotation_threshold": 20
    }
}

class RobotController:
    def __init__(self):
        self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.connect(MQTT_BROKER, 1883)
        self.client.loop_start()
        
        # Robot tracking
        self.robot_targets = {robot_id: None for robot_id in ROBOT_CONFIGS.keys()}
        self.robot_trails = {robot_id: deque(maxlen=20) for robot_id in ROBOT_CONFIGS.keys()}
        self.robot_last_seen = {robot_id: 0 for robot_id in ROBOT_CONFIGS.keys()}
        self.robot_online = {robot_id: False for robot_id in ROBOT_CONFIGS.keys()}
        self.robot_bounding_boxes = {robot_id: None for robot_id in ROBOT_CONFIGS.keys()}
        self.robot_colors = {
            1: (0, 255, 0),   # Green
            2: (255, 0, 0),    # Blue
            3: (0, 0, 255),    # Red
        }
        
        # Camera setup
        self.cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Get frame dimensions once at startup
        ret, frame = self.cap.read()
        if ret:
            self.frame_height, self.frame_width = frame.shape[:2]
            # Publish frame dimensions once at startup
            self.client.publish(FRAME_TOPIC, f"{self.frame_width},{self.frame_height}", qos=1, retain=True)
        else:
            self.frame_width, self.frame_height = 640, 480  # Default values if capture fails
        
        self.frame_count = 0
        self.last_gc_time = time.time()
        self.last_frame_time = time.time()
        self.fps = 0
        
        # Add cleanup flags
        self.running = True
        
        # UI setup
        cv2.namedWindow("Robot Control")
        cv2.setMouseCallback("Robot Control", self.mouse_callback)
    
    def on_connect(self, client, userdata, flags, rc, properties):
        print("Connected to MQTT Broker!")
        # Subscribe to status topics and target position topics
        for topic in STATUS_TOPICS.values():
            client.subscribe(topic)
        for topic in TARGET_POSITION_TOPICS.values():
            client.subscribe(topic)
    
    def on_message(self, client, userdata, msg):
        # Handle status messages from robots
        for robot_id, topic in STATUS_TOPICS.items():
            if msg.topic == topic:
                if b"online" in msg.payload:
                    self.robot_online[robot_id] = True
                    self.robot_last_seen[robot_id] = time.time()
                    print(f"Robot {robot_id} is online")
                elif b"offline" in msg.payload:
                    self.robot_online[robot_id] = False
                    print(f"Robot {robot_id} is offline")
        
        # Handle target position messages
        for robot_id, topic in TARGET_POSITION_TOPICS.items():
            print("checking target pos")
            if msg.topic == topic:
                try:
                    # Parse JSON message (expected format: {"x": 384, "y": 216})
                    data = json.loads(msg.payload)
                    print(data)
                    x = int(data["x"])
                    y = int(data["y"])
                    
                    # Validate coordinates are within frame bounds
                    if 0 <= x < self.frame_width and 0 <= y < self.frame_height:
                        self.robot_targets[robot_id] = (x, y)
                        print(f"Received target for Robot {robot_id}: {data}")
                    else:
                        print(f"Invalid target position for Robot {robot_id}: {data} - out of bounds")
                except json.JSONDecodeError:
                    print(f"Invalid JSON format for Robot {robot_id} target position")
                except KeyError:
                    print(f"Missing x/y coordinates in message for Robot {robot_id}")
                except Exception as e:
                    print(f"Error processing target position for Robot {robot_id}: {e}")
    
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # If only one robot is online, assign to that robot
            online_robots = [rid for rid in ROBOT_CONFIGS.keys() if self.robot_online.get(rid, False)]
            if len(online_robots) == 1:
                self.robot_targets[online_robots[0]] = (x, y)
                print(f"Target for Robot {online_robots[0]} set at: ({x}, {y})")
            else:
                # Existing multi-robot logic
                closest_robot = None
                min_dist = float('inf')
                for robot_id in online_robots:
                    if len(self.robot_trails[robot_id]) > 0:
                        last_pos = self.robot_trails[robot_id][-1]
                        dist = math.sqrt((x - last_pos[0])**2 + (y - last_pos[1])**2)
                        if dist < min_dist:
                            min_dist = dist
                            closest_robot = robot_id
                if closest_robot:
                    self.robot_targets[closest_robot] = (x, y)
    
    def detect_robots(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        detected_robots = {}
        
        # Only detect robots that are online
        online_robots = [rid for rid in ROBOT_CONFIGS.keys() if self.robot_online.get(rid, False)]
        
        # Initialize masks for exclusion
        exclusion_mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        
        # Detect Robot 2 (solid blue square) first if online
        if 2 in online_robots:
            config = ROBOT_CONFIGS[2]
            front, back, bbox = self.detect_blue_square_robot(frame, hsv, config)
            if front and back:
                detected_robots[2] = (front, back)
                self.robot_bounding_boxes[2] = bbox
                center = ((front[0] + back[0])//2, (front[1] + back[1])//2)
                self.robot_trails[2].append(center)
                self.robot_last_seen[2] = time.time()
                self.publish_robot_position(2, front, back)
                cv2.putText(frame, "Robot 2", (center[0]+10, center[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                
                # Add Robot 2's area to exclusion mask
                if bbox is not None:
                    cv2.drawContours(exclusion_mask, [bbox], -1, 255, -1)
        
        # Detect Robot 3 (split blue rectangle) if online, using exclusion mask
        if 3 in online_robots:
            config = ROBOT_CONFIGS[3]
            front, back, bbox = self.detect_split_blue_robot(frame, hsv, config, exclusion_mask)
            if front and back:
                detected_robots[3] = (front, back)
                self.robot_bounding_boxes[3] = bbox
                center = ((front[0] + back[0])//2, (front[1] + back[1])//2)
                self.robot_trails[3].append(center)
                self.robot_last_seen[3] = time.time()
                self.publish_robot_position(3, front, back)
                cv2.putText(frame, "Robot 3", (center[0]+10, center[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
        
        # Detect Robot 1 (LED pair) if online
        if 1 in online_robots:
            config = ROBOT_CONFIGS[1]
            front, back, bbox = self.detect_led_pair_robot(frame, hsv, config)
            if front and back:
                detected_robots[1] = (front, back)
                self.robot_bounding_boxes[1] = bbox
                center = ((front[0] + back[0])//2, (front[1] + back[1])//2)
                self.robot_trails[1].append(center)
                self.robot_last_seen[1] = time.time()
                self.publish_robot_position(1, front, back)
                cv2.putText(frame, "Robot 1", (center[0]+10, center[1]), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw bounding boxes for all detected robots
        for robot_id, bbox in self.robot_bounding_boxes.items():
            if bbox is not None and robot_id in detected_robots:
                cv2.drawContours(frame, [bbox], 0, (255, 0, 255), 2)  # Purple bounding box
        
        return detected_robots
    
    def publish_robot_position(self, robot_id, front_pos, back_pos):
        """Publish robot position to MQTT broker"""
        if not self.robot_online.get(robot_id, False):
            return
            
        # Calculate orientation angle (in degrees)
        dx = front_pos[0] - back_pos[0]
        dy = front_pos[1] - back_pos[1]
        angle = math.degrees(math.atan2(dy, dx))
        
        # Create position message
        pos_msg = f"{front_pos[0]},{front_pos[1]},{back_pos[0]},{back_pos[1]},{angle}"
        
        try:
            self.client.publish(POSITION_TOPICS[robot_id], pos_msg, qos=0)
        except Exception as e:
            print(f"Error publishing position for robot {robot_id}: {e}")
    
    def detect_split_blue_robot(self, frame, hsv, config, exclusion_mask=None):
        blue_mask = cv2.inRange(hsv, config["blue_color"][1], config["blue_color"][2])
        blue_mask = cv2.erode(blue_mask, None, iterations=1)
        blue_mask = cv2.dilate(blue_mask, None, iterations=1)
        
        # Apply exclusion mask if provided
        if exclusion_mask is not None:
            blue_mask = cv2.bitwise_and(blue_mask, cv2.bitwise_not(exclusion_mask))
        
        contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_regions = []
        
        for cnt in contours:
            if cv2.contourArea(cnt) > config["min_blue_area"]:
                rect = cv2.minAreaRect(cnt)
                box = cv2.boxPoints(rect)
                blue_regions.append((rect, np.int32(box)))
        
        # Look for two blue regions that could form Robot 3
        for i, (rect1, box1) in enumerate(blue_regions):
            for j, (rect2, box2) in enumerate(blue_regions[i+1:], i+1):
                # Check if these could be part of the same robot
                center1 = rect1[0]
                center2 = rect2[0]
                dist = np.linalg.norm(np.array(center1) - np.array(center2))
                
                # Maximum allowed distance between segments
                max_dist = max(rect1[1][0], rect1[1][1], 
                            rect2[1][0], rect2[1][1]) * 1.5
                
                if dist < max_dist:
                    # Potential Robot 3 found - look for LEDs
                    front_led = self.find_led_near_region(frame, hsv, box1, config["front_color"])
                    back_led = self.find_led_near_region(frame, hsv, box2, config["back_color"])
                    
                    if front_led and back_led:
                        # Create a combined bounding box for both segments
                        combined_box = np.vstack((box1, box2))
                        rect = cv2.minAreaRect(combined_box)
                        box = cv2.boxPoints(rect)
                        return front_led, back_led, np.int32(box)
                    
                    # Try reverse assignment
                    front_led = self.find_led_near_region(frame, hsv, box2, config["front_color"])
                    back_led = self.find_led_near_region(frame, hsv, box1, config["back_color"])
                    
                    if front_led and back_led:
                        # Create a combined bounding box for both segments
                        combined_box = np.vstack((box1, box2))
                        rect = cv2.minAreaRect(combined_box)
                        box = cv2.boxPoints(rect)
                        return front_led, back_led, np.int32(box)
        
        # Return None for all values if detection fails
        return None, None, None

    def find_led_near_region(self, frame, hsv, box, color_config):
        # Create a mask around the blue region
        mask = np.zeros(frame.shape[:2], dtype=np.uint8)
        cv2.drawContours(mask, [box], -1, 255, -1)
        
        # Expand the search area slightly
        dilated_mask = cv2.dilate(mask, None, iterations=3)
        
        if isinstance(color_config[1], list):  # For red which has two ranges
            mask1 = cv2.inRange(hsv, color_config[1][0], color_config[1][1])
            mask2 = cv2.inRange(hsv, color_config[2][0], color_config[2][1])
            color_mask = cv2.bitwise_or(mask1, mask2)
        else:
            color_mask = cv2.inRange(hsv, color_config[1], color_config[2])
        
        # Combine with region mask
        search_mask = cv2.bitwise_and(color_mask, dilated_mask)
        
        # Find LED
        contours, _ = cv2.findContours(search_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
            if w * h > 20:  # Minimum LED size
                return (x + w//2, y + h//2)
        
        return None
    
    def detect_leds_in_region(self, frame, hsv, box, rect, config):
        # Warp the region to a straight rectangle to make LED detection easier
        width = int(rect[1][0])
        height = int(rect[1][1])
        
        if width == 0 or height == 0:
            return None, None
        
        dst_pts = np.array([
            [0, 0],
            [width-1, 0],
            [width-1, height-1],
            [0, height-1]
        ], dtype="float32")
        
        ordered_box = self.order_box_points(box)
        M = cv2.getPerspectiveTransform(ordered_box, dst_pts)
        warped = cv2.warpPerspective(frame, M, (width, height))
        hsv_warped = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        inverse_M = cv2.getPerspectiveTransform(dst_pts, ordered_box)
        
        # Detect front LED (green)
        green_mask = cv2.inRange(hsv_warped, config["front_color"][1], config["front_color"][2])
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        front_led = None
        if green_contours:
            x, y, w, h = cv2.boundingRect(max(green_contours, key=cv2.contourArea))
            if w * h > 20:  # Minimum LED size
                center_warped = (x + w // 2, y + h // 2)
                pt = np.array([[center_warped]], dtype='float32')
                original_pt = cv2.perspectiveTransform(pt, inverse_M)[0][0]
                front_led = (int(original_pt[0]), int(original_pt[1]))
        
        # Detect back LED (red)
        red_mask1 = cv2.inRange(hsv_warped, config["back_color"][1][0], config["back_color"][1][1])
        red_mask2 = cv2.inRange(hsv_warped, config["back_color"][2][0], config["back_color"][2][1])
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        back_led = None
        if red_contours:
            x, y, w, h = cv2.boundingRect(max(red_contours, key=cv2.contourArea))
            if w * h > 20:  # Minimum LED size
                center_warped = (x + w // 2, y + h // 2)
                pt = np.array([[center_warped]], dtype='float32')
                original_pt = cv2.perspectiveTransform(pt, inverse_M)[0][0]
                back_led = (int(original_pt[0]), int(original_pt[1]))
        
        return front_led, back_led, np.int32(box)
    
    def find_led_in_gap(self, frame, hsv, gap_center, direction, gap_size, looking_for_back, config):
        # Create a search area in the gap between blue regions
        search_size = int(gap_size * 1.5)
        half_size = search_size // 2
        
        # Define the search rectangle
        angle = np.degrees(np.arctan2(direction[1], direction[0]))
        rotation_matrix = cv2.getRotationMatrix2D((half_size, half_size), angle, 1)
        
        # Extract the search area
        x = int(gap_center[0] - half_size)
        y = int(gap_center[1] - half_size)
        search_area = frame[max(0, y):min(frame.shape[0], y + search_size),
                          max(0, x):min(frame.shape[1], x + search_size)]
        
        if search_area.size == 0:
            return None
        
        # Rotate the search area to align with the robot orientation
        rotated = cv2.warpAffine(search_area, rotation_matrix, (search_size, search_size))
        
        # Look for the LED we need
        if looking_for_back:
            # Looking for back (red) LED
            red_mask1 = cv2.inRange(hsv, config["back_color"][1][0], config["back_color"][1][1])
            red_mask2 = cv2.inRange(hsv, config["back_color"][2][0], config["back_color"][2][1])
            mask = cv2.bitwise_or(red_mask1, red_mask2)
        else:
            # Looking for front (green) LED
            mask = cv2.inRange(hsv, config["front_color"][1], config["front_color"][2])
        
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Find the largest contour in the search area
            x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
            if w * h > 20:  # Minimum LED size
                # Calculate the position in the original image
                led_x = x + max(0, x)
                led_y = y + max(0, y)
                return (led_x, led_y)
        
        return None
    
    def is_color_match(self, frame, point, lower, upper):
        # Check if a point in the image matches a color range
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        x, y = point
        if 0 <= y < hsv.shape[0] and 0 <= x < hsv.shape[1]:
            pixel = hsv[y, x]
            return lower[0] <= pixel[0] <= upper[0] and \
                   lower[1] <= pixel[1] <= upper[1] and \
                   lower[2] <= pixel[2] <= upper[2]
        return False
    
    def order_box_points(self, box):
        # Order points for perspective transform
        rect = cv2.minAreaRect(box)
        box = cv2.boxPoints(rect)
        box = np.array(sorted(box, key=lambda p: p[1]))
        top_pts = sorted(box[:2], key=lambda p: p[0])
        bottom_pts = sorted(box[2:], key=lambda p: p[0], reverse=True)
        return np.array([top_pts[0], top_pts[1], bottom_pts[0], bottom_pts[1]], dtype="float32")
    
    def detect_blue_square_robot(self, frame, hsv, config):
        # Detect blue square body
        blue_mask = cv2.inRange(hsv, config["body_color"][1], config["body_color"][2])
        blue_mask = cv2.erode(blue_mask, None, iterations=2)
        blue_mask = cv2.dilate(blue_mask, None, iterations=2)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        front, back, box = None, None, None  # Initialize all return values

        if blue_contours:
            # Find the largest blue contour (robot body)
            blue_contour = max(blue_contours, key=cv2.contourArea)
            if cv2.contourArea(blue_contour) > 500:
                rect = cv2.minAreaRect(blue_contour)
                box = cv2.boxPoints(rect)
                box = np.int32(box)

                width = int(rect[1][0])
                height = int(rect[1][1])

                if width > 0 and height > 0:  # Only proceed if we have valid dimensions
                    dst_pts = np.array([
                        [0, 0],
                        [width-1, 0],
                        [width-1, height-1],
                        [0, height-1]
                    ], dtype="float32")

                    ordered_box = self.order_box_points(box)
                    M = cv2.getPerspectiveTransform(ordered_box, dst_pts)
                    warped = cv2.warpPerspective(frame, M, (width, height))
                    hsv_warped = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
                    inverse_M = cv2.getPerspectiveTransform(dst_pts, ordered_box)

                    # Detect front LED (green)
                    green_mask = cv2.inRange(hsv_warped, config["front_color"][1], config["front_color"][2])
                    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if green_contours:
                        x, y, w, h = cv2.boundingRect(max(green_contours, key=cv2.contourArea))
                        center_warped = (x + w // 2, y + h // 2)
                        pt = np.array([[center_warped]], dtype='float32')
                        original_pt = cv2.perspectiveTransform(pt, inverse_M)[0][0]
                        front = (int(original_pt[0]), int(original_pt[1]))
                        cv2.circle(frame, front, 5, (0, 255, 0), -1)

                    # Detect back LED (red)
                    red_mask1 = cv2.inRange(hsv_warped, config["back_color"][1][0], config["back_color"][1][1])
                    red_mask2 = cv2.inRange(hsv_warped, config["back_color"][2][0], config["back_color"][2][1])
                    red_mask = cv2.bitwise_or(red_mask1, red_mask2)
                    red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if red_contours:
                        x, y, w, h = cv2.boundingRect(max(red_contours, key=cv2.contourArea))
                        center_warped = (x + w // 2, y + h // 2)
                        pt = np.array([[center_warped]], dtype='float32')
                        original_pt = cv2.perspectiveTransform(pt, inverse_M)[0][0]
                        back = (int(original_pt[0]), int(original_pt[1]))
                        cv2.circle(frame, back, 5, (0, 0, 255), -1)

        return front, back, box
    
    def detect_led_pair_robot(self, frame, hsv, config):
        # Detect front LED (green)
        green_mask = cv2.inRange(hsv, config["front_color"][1], config["front_color"][2])
        green_mask = cv2.erode(green_mask, None, iterations=2)
        green_mask = cv2.dilate(green_mask, None, iterations=2)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        front = None
        if green_contours:
            x, y, w, h = cv2.boundingRect(max(green_contours, key=cv2.contourArea))
            if w*h > 50:
                front = (x + w//2, y + h//2)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        # Detect back LED (red)
        red_mask1 = cv2.inRange(hsv, config["back_color"][1][0], config["back_color"][1][1])
        red_mask2 = cv2.inRange(hsv, config["back_color"][2][0], config["back_color"][2][1])
        red_mask = cv2.bitwise_or(red_mask1, red_mask2)
        red_mask = cv2.erode(red_mask, None, iterations=2)
        red_mask = cv2.dilate(red_mask, None, iterations=2)
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        back = None
        if red_contours:
            x, y, w, h = cv2.boundingRect(max(red_contours, key=cv2.contourArea))
            if w*h > 50:
                back = (x + w//2, y + h//2)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        
        if front and back:
            # Create a more representative bounding box for the physical robot
            # Calculate direction vector between LEDs
            dx = back[0] - front[0]
            dy = back[1] - front[1]
            length = math.sqrt(dx*dx + dy*dy)
            
            # Normalize direction vector
            if length > 0:
                dx /= length
                dy /= length
            
            # Calculate perpendicular vector
            perp_dx = -dy
            perp_dy = dx
            
            # Robot dimensions (adjust these based on your actual robot size)
            robot_length = length + 20  # Add some padding
            robot_width = 80  # Approximate width
            
            # Calculate four corners of the bounding box
            half_width = robot_width / 2
            front_left = (
                int(front[0] + perp_dx * half_width),
                int(front[1] + perp_dy * half_width)
            )
            front_right = (
                int(front[0] - perp_dx * half_width),
                int(front[1] - perp_dy * half_width)
            )
            back_left = (
                int(back[0] + perp_dx * half_width),
                int(back[1] + perp_dy * half_width)
            )
            back_right = (
                int(back[0] - perp_dx * half_width),
                int(back[1] - perp_dy * half_width)
            )
            # Create bounding box
            bbox = np.array([front_left, front_right, back_right, back_left], dtype=np.int32)
            return front, back, bbox
        
        return None, None, None
    
    def distance(self, pos1, pos2):
        return math.sqrt((pos1[0]-pos2[0])**2 + (pos1[1]-pos2[1])**2)
    
    def calculate_movement(self, robot_id, front_pos, back_pos, target_pos, other_robots):
        config = ROBOT_CONFIGS[robot_id]
        
        if not front_pos or not back_pos or not target_pos:
            return "STOP"
        
        robot_front = np.array(front_pos)
        robot_back = np.array(back_pos)
        robot_dir = robot_front - robot_back
        robot_dir_normalized = robot_dir / np.linalg.norm(robot_dir)

        target_dir = np.array(target_pos) - robot_front
        target_dist = np.linalg.norm(target_dir)
        
        # If we're very close to target, stop completely
        if target_dist < config["min_distance"]:
            return "STOP"
        
        # Only check for collisions if we're not very close to target
        if target_dist > config["min_distance"] * 1.5:
            avoid_cmd = self.advanced_collision_avoidance(robot_id, front_pos, back_pos, 
                                                        target_pos, other_robots)
            if avoid_cmd:
                return avoid_cmd
        
        # Calculate angle to target
        target_dir_normalized = target_dir / target_dist
        angle = np.arctan2(target_dir_normalized[1], target_dir_normalized[0]) - \
                np.arctan2(robot_dir_normalized[1], robot_dir_normalized[0])
        angle = (angle + np.pi) % (2 * np.pi) - np.pi
        angle_deg = np.degrees(angle)

        # Only rotate if significantly off course
        if abs(angle_deg) > config["rotate_threshold"]:
            # Reduce rotation speed when approaching target
            rotate_speed = config["speeds"]["rotate"]
            if target_dist < config["min_distance"] * 3:
                rotate_speed = max(15, rotate_speed // 2)
            return f"ROTATE_RIGHT:{rotate_speed}" if angle_deg > 0 else f"ROTATE_LEFT:{rotate_speed}"
        else:
            # Adaptive forward speed
            speed = min(config["speeds"]["max"], 
                    config["speeds"]["forward"] * min(1, target_dist/100))
            # Slow down when approaching target
            if target_dist < config["min_distance"] * 3:
                speed = max(15, speed // 2)
            return f"FORWARD:{int(speed)}"
    
    def advanced_collision_avoidance(self, robot_id, front_pos, back_pos, target_pos, other_robots):
        robot_front = np.array(front_pos)
        robot_back = np.array(back_pos)
        robot_dir = robot_front - robot_back
        robot_dir_normalized = robot_dir / np.linalg.norm(robot_dir)
        
        # Get our bounding box
        our_bbox = self.robot_bounding_boxes[robot_id]
        if our_bbox is None:
            return None
        
        # Calculate our safe zone (expanded bounding box)
        our_safe_zone = self.expand_bounding_box(our_bbox, SAFETY_DISTANCE)
        
        for other_robot in other_robots:
            other_id = other_robot[0]  # Get robot ID
            other_front, other_back = other_robot[1]  # Get positions
            
            if other_id == robot_id:
                continue
                
            other_bbox = self.robot_bounding_boxes[other_id]
            if other_bbox is None:
                # If no bounding box but we have positions, create a simple one
                if other_front and other_back:
                    # Create a simple bounding box between the two points
                    dx = other_back[0] - other_front[0]
                    dy = other_back[1] - other_front[1]
                    length = math.sqrt(dx*dx + dy*dy)
                    
                    if length > 0:
                        dx /= length
                        dy /= length
                    
                    perp_dx = -dy
                    perp_dy = dx
                    
                    # Use default robot width if we don't have better info
                    robot_width = 80
                    half_width = robot_width / 2
                    
                    front_left = (
                        int(other_front[0] + perp_dx * half_width),
                        int(other_front[1] + perp_dy * half_width))
                    front_right = (
                        int(other_front[0] - perp_dx * half_width),
                        int(other_front[1] - perp_dy * half_width))
                    back_left = (
                        int(other_back[0] + perp_dx * half_width),
                        int(other_back[1] + perp_dy * half_width))
                    back_right = (
                        int(other_back[0] - perp_dx * half_width),
                        int(other_back[1] - perp_dy * half_width))
                    
                    other_bbox = np.array([front_left, front_right, back_right, back_left], dtype=np.int32)
                else:
                    continue
                    
            # Calculate other robot's safe zone
            other_safe_zone = self.expand_bounding_box(other_bbox, SAFETY_DISTANCE)
            
            # Check for intersection between safe zones
            if self.bounding_boxes_intersect(our_safe_zone, other_safe_zone):
                # Calculate avoidance vector
                other_center = np.mean(other_safe_zone, axis=0)
                our_center = np.mean(our_safe_zone, axis=0)
                
                avoidance_vector = our_center - other_center
                avoidance_vector_normalized = avoidance_vector / np.linalg.norm(avoidance_vector)
                
                # Calculate angle between our direction and avoidance vector
                angle = np.arctan2(avoidance_vector_normalized[1], avoidance_vector_normalized[0]) - \
                        np.arctan2(robot_dir_normalized[1], robot_dir_normalized[0])
                angle = (angle + np.pi) % (2 * np.pi) - np.pi
                angle_deg = np.degrees(angle)
                
                config = ROBOT_CONFIGS[robot_id]
                
                # Determine avoidance maneuver
                if abs(angle_deg) < 90:  # Obstacle in front
                    if angle_deg > 0:
                        return f"ROTATE_RIGHT:{config['speeds']['rotate']},FORWARD:{config['speeds']['forward']//2}"
                    else:
                        return f"ROTATE_LEFT:{config['speeds']['rotate']},FORWARD:{config['speeds']['forward']//2}"
                else:  # Obstacle to the side
                    return f"FORWARD:{config['speeds']['forward']//2}"
        
        return None
    
    def expand_bounding_box(self, bbox, distance):
        """Expand bounding box by specified distance in all directions"""
        center = np.mean(bbox, axis=0)
        expanded = []
        for point in bbox:
            direction = point - center
            if np.linalg.norm(direction) > 0:
                direction_normalized = direction / np.linalg.norm(direction)
                expanded_point = point + direction_normalized * distance
                expanded.append(expanded_point)
        return np.int32(expanded)

    def bounding_boxes_intersect(self, box1, box2):
        """Check if two convex polygons intersect using Separating Axis Theorem"""
        def project(poly, axis):
            dots = [np.dot(p, axis) for p in poly]
            return min(dots), max(dots)
        
        def edges(poly):
            edges = []
            for i in range(len(poly)):
                p1 = poly[i]
                p2 = poly[(i+1)%len(poly)]
                edge = p2 - p1
                normal = np.array([-edge[1], edge[0]])
                normal = normal / np.linalg.norm(normal)
                edges.append(normal)
            return edges
        
        # Check all edge normals of both polygons
        for edge in edges(box1) + edges(box2):
            min1, max1 = project(box1, edge)
            min2, max2 = project(box2, edge)
            
            if max1 < min2 or max2 < min1:
                return False
                
        return True
    
    def draw_robot_info(self, frame, robot_id, front_pos, back_pos):
        color = self.robot_colors.get(robot_id, (255, 255, 255))
        
        # Draw robot direction
        cv2.line(frame, back_pos, front_pos, color, 2)
        
        # Draw robot ID
        cv2.putText(frame, f"Robot {robot_id}", (front_pos[0]+10, front_pos[1]), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        # Draw target if exists
        if self.robot_targets[robot_id]:
            target_pos = self.robot_targets[robot_id]
            cv2.circle(frame, target_pos, 10, color, 2)
            cv2.line(frame, front_pos, target_pos, (255, 0, 255), 1)
            
        # Draw safety zone if bounding box exists
        bbox = self.robot_bounding_boxes[robot_id]
        if bbox is not None:
            safe_zone = self.expand_bounding_box(bbox, SAFETY_DISTANCE)
            cv2.drawContours(frame, [safe_zone], 0, (255, 255, 0), 1)  # Cyan safety zone
    
    def draw_ui(self, frame):
        # Draw status information
        status_y = 20
        for robot_id in ROBOT_CONFIGS.keys():
            color = self.robot_colors.get(robot_id, (255, 255, 255))
            status = "ONLINE" if self.robot_online[robot_id] else "OFFLINE"
            target = f"Target: {self.robot_targets[robot_id]}" if self.robot_targets[robot_id] else "No target"
            cv2.putText(frame, f"Robot {robot_id}: {status} - {target}", 
                       (10, status_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            status_y += 20
        
        cv2.imshow("Robot Control", frame)
        
    def cleanup(self):
        """Clean up resources"""
        self.running = False
        time.sleep(0.1)  # Allow any pending operations to complete
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()
        self.client.disconnect()
        gc.collect()  # Force garbage collection
    
    def run(self):
        try:
            while self.running:
                start_time = time.time()
                
                # Skip frames if processing is falling behind
                if time.time() - self.last_frame_time > 0.1:  # If we're running slower than 10fps
                    ret = self.cap.grab()  # Just grab the frame without decoding
                    if not ret:
                        break
                    continue
                
                ret, frame = self.cap.read()
                if not ret:
                    break
                
                # Periodically force garbage collection
                if time.time() - self.last_gc_time > 5:  # Every 5 seconds
                    gc.collect()
                    self.last_gc_time = time.time()
                
                # Process frame
                detected_robots = self.detect_robots(frame)
                
                # Control each detected robot that is online
                for robot_id, (front, back) in detected_robots.items():
                    if self.robot_online.get(robot_id, False) and self.robot_targets[robot_id]:
                        # Create list of other robots as (robot_id, (front_pos, back_pos))
                        other_robots = [(rid, pos) for rid, pos in detected_robots.items() 
                                    if rid != robot_id and self.robot_online.get(rid, False)]
                        cmd = self.calculate_movement(
                            robot_id, front, back, 
                            self.robot_targets[robot_id], other_robots
                        )
                        try:
                            self.client.publish(CONTROL_TOPICS[robot_id], cmd, qos=0)
                        except Exception as e:
                            print(f"MQTT publish error: {e}")
                
                # Check for offline robots with timeout
                current_time = time.time()
                for robot_id in self.robot_online.keys():
                    if (current_time - self.robot_last_seen[robot_id]) > 40:  # 5 seconds timeout
                        self.robot_online[robot_id] = False
                
                # Calculate FPS
                self.frame_count += 1
                if self.frame_count % 10 == 0:
                    self.fps = 10 / (time.time() - start_time)
                    # print(f"FPS: {self.fps:.1f}")
                
                # Display UI
                self.draw_ui(frame)
                
                # Handle key presses
                key = cv2.waitKey(1)
                if key == ord('q'):
                    break
                elif key == ord(' '):  # Stop all online robots
                    for robot_id in self.robot_targets.keys():
                        if self.robot_online.get(robot_id, False):
                            self.robot_targets[robot_id] = None
                            self.client.publish(CONTROL_TOPICS[robot_id], "STOP", qos=0)
                
                self.last_frame_time = time.time()
                
        finally:
            self.cleanup()

if __name__ == "__main__":
    controller = RobotController()
    controller.run()