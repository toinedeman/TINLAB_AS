from controller import Robot
import time

robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Motor initialization
motor_left = robot.getDevice("motor1")
motor_right = robot.getDevice("motor2")
motor_left.setPosition(float('inf'))
motor_right.setPosition(float('inf'))
motor_left.setVelocity(0.0)
motor_right.setVelocity(0.0)

# Movement parameters
MOVE_SPEED = 5.0
TURN_SPEED = 1.0
ACCELERATION = 0.1

# Communication
receiver = robot.getDevice("receiver")
receiver.enable(timestep)

# State variables
target_left_speed = 0.0
target_right_speed = 0.0
last_command_received = "STOP"
command_end_time = robot.getTime() * 1000

robot_name = robot.getName()
my_id = int(robot_name.split('_')[1])

while robot.step(timestep) != -1:
    # Command processing
    if receiver.getQueueLength() > 0:
        msg = receiver.getString()
        try:
            parts = msg.split(':')
            if len(parts) >= 2:  # At least ID:COMMAND
                received_id, cmd = parts[0], parts[1]
                
                # Only process commands meant for this robot
                if received_id != str(my_id):
                    receiver.nextPacket()
                    continue
                    
                last_command_received = cmd
                
                # Only process duration for non-forward commands
                if cmd != "FORWARD" and len(parts) == 3:
                    command_end_time = robot.getTime() * 1000 + int(parts[2])
                
                # Movement logic
                if cmd == "FORWARD":
                    target_left_speed = -MOVE_SPEED
                    target_right_speed = -MOVE_SPEED
                elif cmd == "LEFT":
                    target_left_speed = TURN_SPEED
                    target_right_speed = -TURN_SPEED
                elif cmd == "RIGHT":
                    target_left_speed = -TURN_SPEED
                    target_right_speed = TURN_SPEED
                elif cmd == "STOP":
                    target_left_speed = 0.0
                    target_right_speed = 0.0
                
                #print(f"Robot {my_id} executing: {cmd}")
                
        except Exception as e:
            print(f"Error processing command: {e}")
        
        receiver.nextPacket()

    # Only stop if explicitly commanded or for non-forward movements
    if (last_command_received != "FORWARD" and 
        robot.getTime() * 1000 >= command_end_time):
        target_left_speed = 0.0
        target_right_speed = 0.0

    # Smooth acceleration
    current_left = motor_left.getVelocity()
    current_right = motor_right.getVelocity()
    
    new_left = current_left + (target_left_speed - current_left) * ACCELERATION
    new_right = current_right + (target_right_speed - current_right) * ACCELERATION
    
    motor_left.setVelocity(new_left)
    motor_right.setVelocity(new_right)