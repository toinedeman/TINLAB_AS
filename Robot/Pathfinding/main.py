import random
import time
from machine import Pin, PWM, ADC, I2C
from vl53l0x import VL53L0X
from umqtt.simple import MQTTClient
import network
import math
import json
from ulab import numpy as np

# read the setting file to get bot specific settings
def read_settings():
    try:
        with open("settings.json", "r") as f:
            settings = json.load(f)
            print("succesfully import settings")
        return settings
    except Exception as e:
        print("Failed to read settings file:", e)
        return None
    
settings = read_settings()
if settings is None:
    print("failed to get settings.")
    exit()


# Pins
BUILT_IN_LED = 25  # Built-in LED
FLED = 20  # Front LED Red
BLED = 21  # Back LED Green
PWM_LM = 6  # Left Continuous Servo
PWM_RM = 7  # Right Continuous Servo
PWM_SC = 10  # Panning Servo
LDR_PIN_ID = 27 # ldr sensor
sda_pin = Pin(0) # SDA pin
scl_pin = Pin(1) # SCL pin

# Sensors
# ldr
ldr_pin = Pin(LDR_PIN_ID, mode=Pin.IN, value=None,pull=None)
ldr = ADC(ldr_pin)
ldr_readings = []
last_spin_time = 0
ldr_min_threshold = settings["ldr_min"]
ldr_max_threshold = settings["ldr_max"]  

# distance sensor
distance_readings = []
i2c = I2C(0, sda=sda_pin, scl=scl_pin, freq=400000)
devices = i2c.scan()
print("I2C devices found:", devices)
distance_sensor = VL53L0X(i2c)

# Initialise leds
built_in_led = Pin(BUILT_IN_LED, Pin.OUT)  # Built-in LED
fled = Pin(FLED, Pin.OUT)  # Front LED
bled = Pin(BLED, Pin.OUT)  # Back LED
fled.value(True)
bled.value(True)
built_in_led.value(True)
time.sleep(1)
built_in_led.value(False)
time.sleep(1)
fled.value(False)
bled.value(False)

# Set up servos
LeftMotor = PWM(Pin(PWM_LM))
LeftMotor.freq(50)
RightMotor = PWM(Pin(PWM_RM))
RightMotor.freq(50)
PanMotor = PWM(Pin(PWM_SC))
PanMotor.freq(50)
stop_duty = 4915
duty_range = 1638

# Speed
left_velocity = settings["left_velocity"]
right_velocity = settings["right_velocity"]

# mqtt settings
broker = settings["mqtt_broker"] # change ip based on network 
port = settings["mqtt_broker_port"]
client_id = f'robot_{random.randint(0, 10000)}'
client = MQTTClient(client_id, broker, port)
topic_register = "swarm/register"
topics = {}
counter = 0
subscribed_topics = set()
update_interval = 50  # Update interval in milliseconds

notfinished = True
target_position = None

target_tolerance = 20

# positioning
radius = 80
radius_correction = 60
robot_data = {}
pos_updated = False
started = False
# wifi
ssid = settings["wifi_ssid"]
password = settings["wifi_password"]
#=========================MOVES============================
# set servo speed of the left motor
# -100 to 100, 0 for stop
def set_servo_speed_left(speed):
    # if speed > 100:
    #    speed = 100
    # elif speed < -100:
    #      speed = -100
     
    duty = stop_duty + int(speed * duty_range / 100)  
    LeftMotor.duty_u16(duty)        

# set servo speed of the right motor
# -100 to 100, 0 for stop
def set_servo_speed_right(speed):
    # if speed > 100:
    #     speed = 100
    # elif speed < -100:
    #     speed = -100
    duty = stop_duty + int((speed * (-1)) * duty_range / 100)  
    RightMotor.duty_u16(duty)
    
# move forward  
def MoveForward(duration):
    global left_velocity
    global right_velocity
    print("MoveForward called with duration:", duration)
    set_servo_speed_left(left_velocity)
    set_servo_speed_right(right_velocity)
    end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        pass
    set_servo_speed_left(0)
    set_servo_speed_right(0)

# move back
def MoveBack(duration):
    global left_velocity
    global right_velocity
    print("MoveBack called with duration:", duration)
    set_servo_speed_left(-left_velocity)
    set_servo_speed_right(-right_velocity)
    end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        pass
    set_servo_speed_left(0)
    set_servo_speed_right(0)

# spin left
def SpinLeft(duration):
    global left_velocity
    global right_velocity
    set_servo_speed_left(0)
    set_servo_speed_right(right_velocity)
    end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        pass
    set_servo_speed_left(0)
    set_servo_speed_right(0)

# spin right
def SpinRight(duration):
    global left_velocity
    global right_velocity
    set_servo_speed_left(left_velocity)
    set_servo_speed_right(0)
    end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        pass
    set_servo_speed_left(0)
    set_servo_speed_right(0)

# move forward continiously
def MoveForwardCont():
    global left_velocity
    global right_velocity
    print("MoveForwardCont called")
    set_servo_speed_left(left_velocity)
    set_servo_speed_right(right_velocity)

# stop the servos
def Stop():
    print("Stop")
    set_servo_speed_left(0)
    set_servo_speed_right(0)


def SpinTop(speed, duration):
    '''
    Spins the sensor mount and reads at certain angles. 
    param speed: set speed of servo, not used 
    param duration: delay between moving the servos
    returns: list of ldr and distance sensor readings and angles
    '''
    Stop()
    ldr_readings = []
    distance_readings = []
    angles = []
    steps = 10
    duty_min = settings["duty_0"]  # all the way right
    duty_mid = settings["duty_mid"] # middle
    duty_max = settings["duty_180"]  # all the way left
    angle_min = 0
    angle_max = 180
    
    step_size = (duty_max - duty_min) // (steps-1)
    #duty_cycles = [2000, 3000, 4000, 5000, 6000, 7000, 8000]
    for i in range(steps):
        duty = duty_min + step_size * i
        angle = angle_min + (angle_max - angle_min) * (duty - duty_min) / (duty_max - duty_min)
        PanMotor.duty_u16(duty)
        end_time = time.ticks_add(time.ticks_ms(), int(duration * 1000))  # Convert duration to milliseconds
        while time.ticks_diff(end_time, time.ticks_ms()) > 0:
            pass
        first_reading = ldr.read_u16()
        second_reading = ldr.read_u16()
        difference = abs(first_reading - second_reading)
        print(f"ldr dif: {difference}")
        while not (difference < 1000):
            first_reading = ldr.read_u16()
            second_reading = ldr.read_u16()
            difference = abs(first_reading - second_reading)
            print(f"ldr dif: {difference}")
        ldr_readings.append(first_reading)
        try:
            distance_value = distance_sensor.read()
        except Exception as e:
            print(e)
            distance_value = 999
        distance_readings.append(distance_value)
        angles.append(angle)
    PanMotor.duty_u16(duty_mid)
    distance_sensor.stop()
    return ldr_readings, distance_readings, angles

# reboots the device
def reboot():
    print('Rebooting...')
    machine.reset()

def euclidian_distance(vector1, vector2):
    return np.linalg.norm(np.array(vector1) - np.array(vector2))

def calculate_intersection_points(coord1, coord2, radius):
    """
    Calculate the intersection points of two bots given their positions and direction vectors.
    param coord1: coordinates of first point
    param coord2: coordinates of second point
    param radius: radius to check in
    returns: if intersect: coordinates of intersection, if not returns None
    """
    coord1 = np.array(coord1)
    coord2 = np.array(coord2)
    
    d = euclidian_distance(coord1, coord2)
    
    # No intersection if distance is greater than 2 times the radius or zero
    if d > 2 * radius or d == 0:
        return None
    
    radius_squared = radius**2
    a = d / 2
    h = math.sqrt(radius_squared - a**2)
    
    midpoint = (coord1 + coord2) / 2
    
    direction = (coord2 - coord1) / d
    
    perpendicular = np.array([-direction[1], direction[0]])
    
    intersection1 = midpoint + h * perpendicular
    intersection2 = midpoint - h * perpendicular
    
    # Calculating the midpoint
    intersection_midpoint = (intersection1 + intersection2) / 2
    
    return tuple(intersection_midpoint)

def check_intersections(current_position, current_vector, radius):
    """
    Get the points of intersection with other bots.
    param current_position: the position to compare other points against
    param current_vector: not used right now
    param radius: radius to check in
    returns: list of intersection coordinates
    """
    global robot_data
    global client_id
    intersections = []
    for robot_id, info in robot_data.items():
        if robot_id == client_id:
            continue
        other_position = info['position']
        other_vector = info['vector']
        points = calculate_intersection_points(current_position, other_position, radius)
        if points:
            intersections.append(points)
    return intersections

def check_border_intersection(current_position, radius, width, height):
    """
    Check if a circle intersects with the borders of the image.
    param current_position: the current position of the chariot
    param radius: radius to check within
    param width: width of the area
    param height: height of the area
    returns: list of intersections with border
    """
    x, y = current_position
    intersections = []
    # augments the detection radius for the border
    corrected_radius = radius + radius_correction
    
    # Check intersection with the left border (x = 0)
    if x < corrected_radius:
        intersections.append((0, y))
    
    # Check intersection with the right border (x = width)
    if x > width - corrected_radius:
        intersections.append((width, y))
    
    # Check intersection with the top border (y = 0)
    if y < corrected_radius:
        intersections.append((x, 0))
    
    # Check intersection with the bottom border (y = height)
    if y > height - corrected_radius:
        intersections.append((x, height))
    
    return intersections

def calc_cross_product(v1, v2):
        return v1[0] * v2[1] - v1[1] * v2[0]

def calc_dot_product(v1, v2):
    return v1[0] * v2[0] + v1[1] * v2[1]

def normalize_vector(vector):
    norm = math.sqrt(vector[0]**2 + vector[1]**2)
    if norm == 0:
        return vector  # Return the original vector if its norm is 0 (to avoid division by zero)
    return (vector[0] / norm, vector[1] / norm)

def avoid_collisions(current_position, normalized_vector, intersections, border_intersections):
     """
     Adjust movements to avoid collisions.
     param current_position: current position of the bot
     param normalized_vector: direction vector
     param intersections: list of intersections
     param border_intersections: list of border intersections     
     """
     # Process intersections with other bots
     for point in intersections:
         vector_to_point = (point[0] - current_position[0], point[1] - current_position[1])
         normalized_vector_to_point = normalize_vector(vector_to_point)
       
         dot = calc_dot_product(normalized_vector, normalized_vector_to_point)
        
         if dot < 0:
             print("Collision behind, move forward")
             MoveForward(0.1)
             return
         else:
             print("Collision in front, avoid to the side")
             # Decide to spin left or right based on cross product sign
             cross = calc_cross_product(normalized_vector, normalized_vector_to_point)
             if cross > 0:
                 print("Avoid to the left")
                 SpinLeft(0.3)
             else:
                 print("Avoid to the right")
                 SpinRight(0.3)
             return
   
     # Process intersections with borders
     for border in border_intersections:
         vector_to_border = (border[0] - current_position[0], border[1] - current_position[1])
         normalized_vector_to_border = normalize_vector(vector_to_border)
       
         dot = calc_dot_product(normalized_vector, normalized_vector_to_border)
       
         if dot < 0:
             print("Border behind, move forward")
             MoveForward(0.1)
             return
         else:
             print("Border in front, avoid to the side")
             # Decide to spin left or right based on cross product sign
             cross = calc_cross_product(normalized_vector, normalized_vector_to_border)
             if cross > 0:
                 print("Spin to the left to avoid border")
                 SpinLeft(0.3)
             else:
                 print("Spin to the right to avoid border")
                 SpinRight(0.3)
             return

     print("No collisions detected")

def pathing_light():
    '''
    Tries to find the light by using the ldr and and moving forward unless it has to avoid a border or other chariot.
    '''
    global robot_data
    global client_id
    global last_spin_time
    global notfinished
    global target_position  # New global to hold target position
    global target_tolerance
    global radius

    # get out if already at end position
    if not notfinished:
        return

    try:
        current_position = robot_data[client_id]['position']
        current_vector = robot_data[client_id]['vector']
    except KeyError:
        print(f"No key with that name: {client_id}")
        return
    
    # get the intersections to other bots or borders
    intersections = check_intersections(current_position, current_vector, radius)
    border_intersections = check_border_intersection(current_position, radius, 1280, 720)
    # if there are intersections, avoid them
    if intersections or border_intersections:
        avoid_collisions(current_position, current_vector, intersections, border_intersections)
    # if a target position has been set, move towards it.
    elif target_position is not None:  # If a target position is set, move towards it
        if not move_to_position(current_position, current_vector, target_position, target_tolerance):
            notfinished = False
            Stop()
            print(f"Arrived at target position: {target_position}")
        return
    # scan if interval is reached, move based on ldr value if it reaches the threshold
    else:
        if time.ticks_diff(last_spin_time, time.ticks_ms()) < 0:
            ldr_readings, distance_readings, angles = SpinTop(10, 0.1)
            highest_ldr_value = max(ldr_readings)
            highest_ldr_index = ldr_readings.index(highest_ldr_value)
            highest_ldr_angle = angles[highest_ldr_index]
            distance = distance_readings[highest_ldr_index]
            print(distance)
            print(highest_ldr_value)
            # if the ldr value is high enough, send position to server
            if highest_ldr_value > ldr_max_threshold and distance < 170:
                print("Found it!")
                client.publish(topics['send'], f"foundit {current_position}")
                MoveBack(1)

            elif highest_ldr_value >= ldr_min_threshold:
                steer_to_angle(highest_ldr_angle)  
            else:
                print(f"LDR values to low ({ldr_min_threshold})")
            last_spin_time = time.ticks_add(time.ticks_ms(), int(300))
            
        else:
            MoveForwardCont()
        
def steer_to_vector(current_vector, target_vector):
    '''
    steer the robot towards the target vector, move forward if the vector is within margin
    param current_vector: current vector of the chariot
    param target_vector: target vector to aim for
    '''
    current_vector = normalize_vector(current_vector)
    target_vector = normalize_vector(target_vector)

    dot_product = calc_dot_product(current_vector, target_vector)
    cross_product = calc_cross_product(current_vector, target_vector)
    
    turn_rate = 0.1
    speed = 0.2

    # If the dot product is close to 1, move forward
    if dot_product > 0.9659:
        MoveForward(speed)
    else:
        # Adjust direction
        if cross_product > 0:
            SpinRight(turn_rate)
        else:
            SpinLeft(turn_rate)

def move_to_position(current_position, current_vector, target_position, tolerance):
    '''
    Move the target towards the target position
    param current_position: current position of the chariot
    param current_vector: current direction vector
    param target_position: target position to aim for
    param tolerance: tolerance to hit the target position
    '''
    target_x, target_y = target_position
    current_x, current_y = current_position
    
    # calculate distance to the target position, if this is within tolerance then the chariot has reached it end position
    distance = euclidian_distance(current_position, target_position)
    
    print(f"distance: {distance}")
    if distance <= tolerance:
        print("withing distance")
        return False  
     
    target_vector = (target_x - current_x, target_y - current_y)
    steer_to_vector(current_vector, target_vector)
    return True  
    
def steer_to_angle(target_angle):
    target_angle %= 360  # Normalize the target angle to 0-359 degrees

    if target_angle > 180:
        SpinRight((360 - target_angle) / 180)
    else:
        SpinLeft(target_angle / 180)

# MQTT callbacks
def on_message(topic, msg):
    '''
    Handle the incoming messages based on their topic and payload.
    param topic: mqtt topic where the message came in
    param msg: the received message
    '''
    global topics, robot_data, pos_updated, target_position
    print("I love cheese")

    # decode topic and message
    topic = topic.decode('utf-8')
    message = msg.decode('utf-8')

    # if the topic is /config, save configuration of mqtt topics
    if topic == f"robots/{client_id}/config":
        print("Received configuration response.")
        config = message.split(',')
        if len(config) == 2:
            topics['receive'] = config[0]
            topics['send'] = config[1]
            client.subscribe(topics['receive'])
            print(f"Subscribed to {topics['receive']}")
            client.publish(topics['send'], f"{client_id} connected successfully")
            print(f"Published '{client_id} connected successfully' to {topics['send']}")
        else:
            print("Invalid configuration format received.")
    
    # incoming message to specific chariot
    elif topic == topics['receive']:
        print("Received message on the receive topic.")
        # if message contains found it, take the coordinates and set it as target position
        if message.startswith('foundit'):
            try:
                coordinates_str = message.replace("foundit {", "").replace("}", "")
                x_str, y_str = coordinates_str.split(',')
                x = int(x_str.strip())
                y = int(y_str.strip())
                print(f"Received coordinates: x={x}, y={y}")
                target_position = (x, y)  # Set the target position for the robot
            except ValueError:
                print("Error: Unable to parse coordinates from the received message.")
        # if none of those, assume its a position message, save the position of each bot
        else:
            try:
                data = json.loads(message)
                # If message is valid JSON, it is assumed to contain position data
                for robot_id, robot_info in data.items():
                    if robot_id != client_id:   
                        if robot_id in robot_data:
                            robot_data[robot_id]['position'] = robot_info['position']
                            robot_data[robot_id]['vector'] = robot_info['vector']
                        else:
                            robot_data[robot_id] = {'position': robot_info['position'], 'vector': robot_info['vector']}
                    else:
                        robot_data[robot_id] = {'position': robot_info['position'], 'vector': robot_info['vector']}
                
                print("Updated robot data:", robot_data)
                pos_updated = True
            except ValueError:
                # If message is not valid JSON, it is assumed to be a command
                print(f"Received command: {message} on {topic}")
                handle_command(message)
    else:
        print(f"Received message on unknown topic {topic}: {message}")
        handle_command(message)

def handle_command(command):
    '''
    Handles incoming commands
    '''
    global started
    command = command.strip().upper()
    print(command)
    if command == "FRONT_LED_ON":
        fled.value(True)
    elif command == "FRONT_LED_OFF":
        fled.value(False)
    elif command == "BACK_LED_ON":
        bled.value(True)
    elif command == "BACK_LED_OFF":
        bled.value(False)
    elif command == "MOVE_FORWARD":
        MoveForward(1)
    elif command == "MOVE_FORWARD_CONT":
        MoveForwardCont()
    elif command == "MOVE_BACK":
        MoveBack(1)
    elif command == "SPIN_LEFT":
        SpinLeft(0.3)
    elif command == "SPIN_RIGHT":
        SpinRight(0.3)
    elif command == "SPIN_TOP":
        SpinTop(10.0, 1)
    elif command == "START":
        started = True
    elif command == "STOP":
        Stop()
    elif command == "REBOOT":
        reboot()
    else:
        print("Invalid command.")


def connect_mqtt():
    '''
    Connect to MQTT broker and start listening for commands
    '''
    global client
    try:
        client.set_callback(on_message)
        client.connect()
        print("Robot connected to MQTT Broker!")
        client.subscribe(f"robots/{client_id}/config")
        print(f"Subscribed to robots/{client_id}/config")
        client.publish(topic_register, client_id)
        print(f"Registration message sent: {client_id}")
    except Exception as e:
        print(f"Failed to connect to MQTT Broker: {e}")
        for i in range(3):
            fled.value(True)
            time.sleep(0.2)
            fled.value(False)
            time.sleep(0.2)


#==============================SETUP============================
Stop()
#==============================WIFI=============================
# Activate the Pico LAN
network.hostname(client_id)
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
print("Hostname set to: " + str(network.hostname()))

time0 = time.time()
wlan.connect(ssid, password)
while True:
    if wlan.isconnected():
        print("\nConnected!\n")
        built_in_led.value(True)
        break
    else:
        print(".")
        time.sleep(1)
        if time.time() - time0 > 10:
            print("Connection could not be established")
            break

sta_if = network.WLAN(network.STA_IF)
print(sta_if.ifconfig()[0])  # Print the IP on the serial
last_update = time.ticks_ms()


# Setup MQTT
connect_mqtt()
distance_sensor.start()
#==============================Main loop========================
while True:
    try:
        client.check_msg()  # Check for new messages
        if time.ticks_diff(time.ticks_ms(), last_update) > update_interval and started and notfinished:
            client.publish(topics['send'], b"request_positions")
            last_update = time.ticks_ms()  # Update the last update time
        
        if pos_updated and notfinished:
            pathing_light()
            pos_updated = False
        if not notfinished:
            distance_sensor.stop()
            
    except OSError as e:
        print(f"Error in main loop: {e}")
        time.sleep(2)  # Wait before retrying
        connect_mqtt()
        time.sleep(0.01)
    except Exception as e:
       print(e)
    time.sleep(0.01)
