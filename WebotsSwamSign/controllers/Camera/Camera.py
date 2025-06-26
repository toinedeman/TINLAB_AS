from controller import Robot, Camera, Emitter
import math
import numpy as np
import paho.mqtt.client as mqtt
import json

# Initialize robot and devices
robot = Robot()
timestep = int(robot.getBasicTimeStep())

camera = robot.getDevice("camera")
camera.enable(timestep)
emitter = robot.getDevice("emitter")

# Verify camera
print(f"Camera width: {camera.getWidth()}, height: {camera.getHeight()}")
print(f"Camera FOV: {camera.getFov()}")
# Constants
GRID_SIZE = 25
DETECTION_INTERVAL = 2000  
MAX_DISTANCE = 10
FORGET_TIME = 5000  
MIN_DISTANCE = 0.5

# Navigation parameters
TARGET_TOLERANCE = 1
FORWARD_SPEED_MS = 0.1  
TURN_RATE = 0.5  
ALIGNMENT_THRESHOLD = math.cos(math.radians(18))  

# MQTT setup
mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
digital_targets = {}  
physical_targets = {}

# Physical robot mappings 
physical_robot_mapping = {
    1: 6,  # targetpos1 -> FysicalBot_6
    2: 7,  
    3: 8   
}

# Store the frame dimensions
frame_width = None
frame_height = None

def calculate_middle_point(front_x, front_y, back_x, back_y):
    """Calculate the middle point between front and back positions"""
    middle_x = (front_x + back_x) / 2
    middle_y = (front_y + back_y) / 2
    return middle_x, middle_y

def scale_to_grid(x, y, frame_width, frame_height, grid_size):
    """
    Schaal een (x, y) pixelpositie van een camera of beeld met een (0, 0) midden
    naar een gridpositie binnen een grid_size x grid_size veld.
    
    x: pixelpositie (horizontaal)
    y: pixelpositie (verticaal)
    frame_width: breedte van het frame in pixels
    frame_height: hoogte van het frame in pixels
    grid_size: aantal cellen per richting (bijv. 25 => grid van -12 t/m +12)
    """
    if frame_width is None or frame_height is None:
        return x, y  # failsafe

    # Normaliseer van pixels naar [-1, 1]
    norm_x = (2 * x / frame_width) - 1
    norm_y = (2 * y / frame_height) - 1

    # Inverteer Y-as omdat pixel-Y meestal top-down is
    norm_y *= -1
    norm_x *= -1

    # Schaal naar grid-coördinaten (van -grid_size//2 tot +grid_size//2)
    half_grid = grid_size // 2
    grid_x = int(norm_x * half_grid)
    grid_y = int(norm_y * half_grid)

    return grid_x, grid_y



# MQTT message handler
last_sent_coordinates = {}

def on_mqtt_message(client, userdata, msg):
    global frame_width, frame_height
    
    try:
        if msg.topic == "robot/frame/dimensions":
            # Handle frame dimensions message
            dimensions = msg.payload.decode().split(",")
            if len(dimensions) == 2:
                frame_width = int(dimensions[0]) - 100
                frame_height = int(dimensions[1]) - 100
                print(f"Received frame dimensions: {frame_width}x{frame_height}")
                
        elif msg.topic.startswith("robot/position/"):
            # Handle position data in format "front_x,front_y,back_x,back_y,angle"
            robot_id = int(msg.topic.split("/")[-1])
            data_parts = msg.payload.decode().split(",")
            
            if len(data_parts) == 5:
                front_x = float(data_parts[0])
                front_y = float(data_parts[2])
                back_x = float(data_parts[1])
                back_y = float(data_parts[3])
                
                # Calculate middle point
                middle_x, middle_y = calculate_middle_point(front_x, front_y, back_x, back_y)
                
                # Scale to centered grid if we have frame dimensions
                if frame_width and frame_height:
                    scaled_x, scaled_y = scale_to_grid(middle_x, middle_y, frame_width, frame_height, GRID_SIZE)
                else:
                    scaled_x, scaled_y = middle_x, middle_y
                
                position = {
                    'x': scaled_x,
                    'y': scaled_y,
                }
                
                physical_targets[robot_id] = position
                
                if robot_id in physical_robot_mapping:
                    physical_id = physical_robot_mapping[robot_id]
                    current_coords = (position['x'], position['y'])
                    
                    if physical_id not in last_sent_coordinates or last_sent_coordinates[physical_id] != current_coords:
                        msg = f"{physical_id}:GOTO:{position['x']}:{position['y']}"
                        emitter.send(msg.encode('utf-8'))
                        last_sent_coordinates[physical_id] = current_coords
                        print(f"Sent command to physical bot {physical_id}: {msg}")
                    
        elif msg.topic.startswith("digital/robot/targetpos/"):
            robot_id = msg.topic.split("/")[-1]
            position = json.loads(msg.payload.decode())
            # Convert digital target coordinates to centered system if needed
            digital_targets[robot_id] = position
            print(f"Received DIGITAL target for {robot_id}: {position}")
            
            if robot_id in physical_robot_mapping:
                physical_id = physical_robot_mapping[robot_id]
                msg = f"{physical_id}:GOTO:{position['x']}:{position['y']}"
                emitter.send(msg.encode('utf-8'))
                print(f"Sent command to physical bot {physical_id}: {msg}")
                
    except Exception as e:
        print(f"MQTT message error: {e}")

# Setup MQTT client
mqtt_client.on_message = on_mqtt_message
mqtt_client.connect("192.168.4.1", 1883)
mqtt_client.subscribe("robot/position/#")
mqtt_client.subscribe("digital/robot/targetpos/#")
mqtt_client.subscribe("robot/frame/dimensions")
mqtt_client.loop_start()

# Initialize tracking structures
tracked_robots = {}
next_robot_id = 1
last_commands = {} 
turn_end_times = {} 
robot_states = {} 

# Function to detect blobs in the camera image
def detect_blobs(image, width, height):
    blobs = []
    visited = set()

    for i in range(width):
        for j in range(height):
            if (i, j) not in visited:
                r = Camera.imageGetRed(image, width, i, j)
                g = Camera.imageGetGreen(image, width, i, j)
                b = Camera.imageGetBlue(image, width, i, j)

                # Detect blue blobs
                color = None
                if b > 100 and b > 1.5 * r and b > 1.5 * g:
                    color = "blue"

                # When a blob is detected, flood fill to find all connected pixels
                if color:
                    blob_pixels = []
                    queue = [(i, j)]
                    while queue:
                        x, y = queue.pop()
                        if (x, y) in visited:
                            continue
                        if 0 <= x < width and 0 <= y < height:
                            r2 = Camera.imageGetRed(image, width, x, y)
                            g2 = Camera.imageGetGreen(image, width, x, y)
                            b2 = Camera.imageGetBlue(image, width, x, y)

                            if (color == "blue" and b2 > 100 and b2 > 1.5 * r2 and b2 > 1.5 * g2):
                                visited.add((x, y))
                                blob_pixels.append((x, y))
                                queue.extend([(x+1,y), (x-1,y), (x,y+1), (x,y-1)])

                    if len(blob_pixels) > 5:
                        avg_x = sum(p[0] for p in blob_pixels) / len(blob_pixels)
                        avg_y = sum(p[1] for p in blob_pixels) / len(blob_pixels)
                        x_scaled = round(avg_x * GRID_SIZE / width)
                        y_scaled = round(avg_y * GRID_SIZE / height)

                        SumRed = [0, 0]
                        SumGreen = [0, 0]
                        CountRed = 0
                        CountGreen = 0
                        search_radius = 10
                        x_center, y_center = int(avg_x), int(avg_y)

                        # Search around the center of the blob for red and green pixels
                        for dx in range(-search_radius, search_radius + 1):
                            for dy in range(-search_radius, search_radius + 1):
                                nx, ny = x_center + dx, y_center + dy
                                if 0 <= nx < width and 0 <= ny < height:
                                    r3 = Camera.imageGetRed(image, width, nx, ny)
                                    g3 = Camera.imageGetGreen(image, width, nx, ny)
                                    b3 = Camera.imageGetBlue(image, width, nx, ny)

                                    # Check for red and green pixels
                                    if r3 > 200 and g3 < 50 and b3 < 50:
                                        SumRed = [SumRed[0]+dx, SumRed[1]+dy]
                                        CountRed += 1
                                    elif g3 > 150 and g3 > 1.5 * r3 and g3 > 1.5 * b3:
                                        SumGreen = [SumGreen[0]+dx, SumGreen[1]+dy]
                                        CountGreen += 1
                        # Normalize the sums to the center of the blob                    
                        if CountGreen == 0 or CountRed == 0:
                            return []

                        # Calculate the direction vector from the red and green sums                    
                        deltaS = [SumRed[0]/CountRed-SumGreen[0]/CountGreen, 
                                 SumRed[1]/CountRed-SumGreen[1]/CountGreen]
                        
                        # Ensure deltaS is never zero by adding a small epsilon if needed
                        deltaS_norm = np.linalg.norm(deltaS)
                        if deltaS_norm < 1e-6:  # If deltaS is effectively zero
                            deltaS = [1e-6, 1e-6]  # Small arbitrary vector
                        else:
                            deltaS = deltaS/deltaS_norm  # Normalize as before
                            
                        north_vector = [0, -1]
                        angle_rad = np.arctan2(north_vector[1], north_vector[0]) - np.arctan2(deltaS[1], deltaS[0])
                        angle_deg = np.degrees(angle_rad)
                        if angle_deg < 0:
                            angle_deg = 360 + angle_deg

                        blobs.append((x_scaled, y_scaled, color, angle_deg))
    return blobs

# Function to update the tracking of robots based on new positions
def update_tracking(new_positions, current_time):
    global next_robot_id, tracked_robots

    for rid in tracked_robots:
        tracked_robots[rid]["matched"] = False

    for pos_x, pos_y, color, angle_deg in new_positions:
        pos = (pos_x, pos_y)
        matched = False
        closest_id = None
        min_distance = float('inf')

        # Check if the position matches an existing robot
        for rid, data in tracked_robots.items():
            if not data["matched"] and data["color"] == color:
                dist = math.sqrt((pos[0]-data["position"][0])**2 + (pos[1]-data["position"][1])**2)
                if dist < MAX_DISTANCE and dist < min_distance:
                    min_distance = dist
                    closest_id = rid

        # If a match is found, update the robot's position 
        if closest_id is not None:
            tracked_robots[closest_id].update({
                "position": pos,
                "last_seen": current_time,
                "matched": True,
                "true_angle": angle_deg
            })
            matched = True

        # If no match is found, create a new robot entry
        if not matched:
            tracked_robots[next_robot_id] = {
                "position": pos,
                "last_seen": current_time,
                "matched": True,
                "color": color,
                "true_angle": angle_deg          
            }
            next_robot_id += 1

    # Remove robots that haven't been seen for a while
    to_remove = [rid for rid, data in tracked_robots.items() 
                if not data["matched"] and current_time - data["last_seen"] > FORGET_TIME]
    for rid in to_remove:
        del tracked_robots[rid]
        if rid in turn_end_times:
            del turn_end_times[rid]
        if rid in last_commands:
            del last_commands[rid]
        if rid in robot_states:
            del robot_states[rid]

# Function to assign targets to robots based on their positions
def assign_targets_to_robots(tracked_robots, digital_targets):
    """Assign each robot to its closest digital target"""
    robot_assignments = {}
    
    if not digital_targets:
        return robot_assignments
    
    # Convert dictionary values to list of (x,y) tuples
    targets = [(pos['x'], pos['y']) for pos in digital_targets.values()]
    robot_ids = list(tracked_robots.keys())
    
    # Create a distance matrix between all robots and targets
    distance_matrix = []
    for rid in robot_ids:
        robot_pos = tracked_robots[rid]["position"]
        distances = []
        for target in targets:
            dist = math.sqrt((robot_pos[0]-target[0])**2 + (robot_pos[1]-target[1])**2)
            distances.append(dist)
        distance_matrix.append(distances)
    
    # Find optimal assignment 
    if len(robot_ids) == 2 and len(targets) == 2:
        option1_dist = distance_matrix[0][0] + distance_matrix[1][1]
        option2_dist = distance_matrix[0][1] + distance_matrix[1][0]
        
        # Assign robots to targets based on the minimum distance
        if option1_dist <= option2_dist:
            robot_assignments[robot_ids[0]] = targets[0]
            robot_assignments[robot_ids[1]] = targets[1]
        else:
            robot_assignments[robot_ids[0]] = targets[1]
            robot_assignments[robot_ids[1]] = targets[0]
    else:
        available_targets = targets.copy()
        for rid in robot_ids:
            robot_pos = tracked_robots[rid]["position"]
            min_dist = float('inf')
            best_target = None
            for target in available_targets:
                dist = math.sqrt((robot_pos[0]-target[0])**2 + (robot_pos[1]-target[1])**2)
                if dist < min_dist:
                    min_dist = dist
                    best_target = target
            if best_target:
                robot_assignments[rid] = best_target
                available_targets.remove(best_target)
    
    return robot_assignments

# Function to calculate movement commands for a robot
def calculate_movement(robot_id, current_pos, target_pos, true_angle):
    if not target_pos:
        return ("stop", 0, 0, 0)

    # Round positions to integers
    current_pos = (round(current_pos[0]), round(current_pos[1]))
    target_pos = (round(target_pos[0]), round(target_pos[1]))
    
    # Check if positions are exactly the same (integer coordinates)
    if current_pos == target_pos:
        return ("stop", 0, 0, 0)
    
    current_pos_np = np.array([current_pos[0], current_pos[1], 0])
    target_pos_np = np.array([target_pos[0], target_pos[1], 0])
    
    distance = np.linalg.norm(target_pos_np - current_pos_np)

    orientation_vec = (-np.sin(np.deg2rad(true_angle)), -np.cos(np.deg2rad(true_angle)), 0)
    target_vec = target_pos_np - current_pos_np

    if np.linalg.norm(target_vec) > 0:
        target_vec = target_vec / np.linalg.norm(target_vec)
    
    dot_product = np.dot(orientation_vec, target_vec)
    cross_product = np.cross(orientation_vec, target_vec)
    
    # Determine the command 
    if dot_product > ALIGNMENT_THRESHOLD:
        return ("forward", int(1000 * distance/FORWARD_SPEED_MS), 0, distance)
    elif cross_product[2] > 0:  
        angle = np.arccos(np.clip(dot_product, -1, 1))
        return ("left", int(1000 * angle/TURN_RATE), np.degrees(angle), distance)
    else:
        angle = np.arccos(np.clip(dot_product, -1, 1))
        return ("right", int(1000 * angle/TURN_RATE), np.degrees(angle), distance)
        
# Function to send commands to the robot
def send_command_to_robot(robot_id, command_type, duration_ms=100):
    if command_type == "forward":
        msg = f"{robot_id}:FORWARD:{duration_ms}"
    elif command_type == "left":
        msg = f"{robot_id}:LEFT:{duration_ms}"
    elif command_type == "right":
        msg = f"{robot_id}:RIGHT:{duration_ms}"
    else:
        msg = f"{robot_id}:STOP"
    emitter.send(msg.encode('utf-8'))

# Main loop
last_print_time = 0

while robot.step(timestep) != -1:
    current_time = int(robot.getTime() * 1000)

    # Image processing and tracking (only for blue robots)
    image = camera.getImage()
    blobs = detect_blobs(image, camera.getWidth(), camera.getHeight())
    update_tracking(blobs, current_time)

    # Command blue robots based on digital targets
    if tracked_robots and digital_targets:
        target_assignments = assign_targets_to_robots(tracked_robots, digital_targets)
        
        for rid, data in tracked_robots.items():
            current_state = robot_states.get(rid, 'idle')
            target_pos = target_assignments.get(rid)
            
            if not target_pos:
                if current_state != 'idle':
                    send_command_to_robot(rid, "stop")
                    robot_states[rid] = 'idle'
                    last_commands[rid] = "STOP"
                continue

            current_pos = data["position"]
            distance = math.sqrt((target_pos[0]-current_pos[0])**2 + (target_pos[1]-current_pos[1])**2)
            
            if distance < TARGET_TOLERANCE:
                if current_state != 'idle':
                    send_command_to_robot(rid, "stop")
                    robot_states[rid] = 'idle'
                    last_commands[rid] = "STOP"
                continue

            if current_state == 'turning':
                if current_time < turn_end_times.get(rid, 0):
                    continue
                else:
                    send_command_to_robot(rid, "forward")
                    robot_states[rid] = 'moving'
                    last_commands[rid] = "FORWARD"

            elif current_state == 'moving':
                (cmd, dur, angle, dist) = calculate_movement(rid, current_pos, target_pos, data["true_angle"])
                if abs(angle) > math.degrees(math.acos(ALIGNMENT_THRESHOLD)):
                    send_command_to_robot(rid, "stop")
                    robot_states[rid] = 'idle'
                    last_commands[rid] = "STOP"

            elif current_state == 'idle':
                (command, duration, angle, dist) = calculate_movement(rid, current_pos, target_pos, data["true_angle"])
                
                if command == "stop":
                    send_command_to_robot(rid, "stop")
                elif command in ["left", "right"]:
                    send_command_to_robot(rid, command, duration)
                    robot_states[rid] = 'turning'
                    turn_end_times[rid] = current_time + duration
                    last_commands[rid] = command.upper()
                else:
                    send_command_to_robot(rid, "forward", duration)
                    robot_states[rid] = 'moving'
                    last_commands[rid] = "FORWARD"

    # Periodic status print
    if current_time - last_print_time > 5000:
        print("\n=== Current System Status ===")
        print("Physical Robots:")
        for rid, pos in physical_targets.items():
            print(f"  {rid}: {pos}")
        
        print("\nDigital Robots:")
        for rid, pos in digital_targets.items():
            print(f"  {rid}: {pos}")
        
        print("\nTracked Robots:")
        for rid, data in tracked_robots.items():
            target = target_assignments.get(rid) if 'target_assignments' in locals() else None
            print(f"  Robot {rid}:")
            print(f"    Position: {data['position']}")
            print(f"    Target: {target}")
            print(f"    Angle: {data['true_angle']:.1f}°")
            print(f"    State: {robot_states.get(rid, 'unknown')}")
            print(f"    Last command: {last_commands.get(rid, 'none')}")
        
        last_print_time = current_time