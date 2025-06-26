from controller import Robot, Emitter
import math
import time

# Initialize supervisor robot (the world supervisor)
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Initialize Emitter to send commands to robots
emitter = robot.getDevice("emitter")

# --- Supervisor Parameters ---
# These are the speeds/durations that the supervisor *expects* the robot to adhere to.
# They should correspond to the robot's physical capabilities and its controller's logic.
ROBOT_MAX_SPEED = 3.0           # Max speed the robot *can* go
ROBOT_MOVE_SPEED = 2.5          # Speed robot moves forward/backward
ROBOT_TURN_SPEED = 1.0          # Speed robot turns (angular velocity of wheels)
ROBOT_WHEEL_RADIUS = 0.02       # Example: 2 cm wheel radius
ROBOT_AXLE_LENGTH = 0.05        # Example: 5 cm distance between wheels

TURN_SPEED_DEGREES_PER_SECOND = 29.217 # From your previous tests, 90 deg / sec at TURN_SPEED=1.0

# --- State for Simple Sequence ---
test_robot_id = 1 # The ID of the robot this supervisor will control
test_sequence = ["RIGHT_90", "WAIT", "LEFT_90", "WAIT", "FORWARD_3000", "WAIT", "STOP_ALL"]
current_test_step_index = 0
step_start_time_ms = 0
robot_states = {test_robot_id: 'idle'} # Supervisor's internal state tracking for the robot

def send_command(robot_id, command_type):
    """Sends a command string to a specific robot via the emitter."""
    msg = f"{robot_id}:{command_type.upper()}"
    emitter.send(msg.encode('utf-8'))
    print(f"[SUPERVISOR] Sent command: {msg}")

print("[SUPERVISOR] Starting simple movement test sequence.")

# --- Main Supervisor Loop ---
while robot.step(timestep) != -1:
    current_time_ms = int(robot.getTime() * 1000)

    # If the current step's duration has passed, or it's the very beginning
    if current_time_ms >= step_start_time_ms:
        if current_test_step_index < len(test_sequence):
            current_action = test_sequence[current_test_step_index]

            if current_action == "RIGHT_90":
                print(f"[{current_time_ms}ms] SUPERVISOR: Executing RIGHT_90 for Robot {test_robot_id}")
                send_command(test_robot_id, "RIGHT")
                # Calculate duration needed to turn 90 degrees
                # duration_ms = (angle_degrees / degrees_per_second) * 1000
                duration_ms = int(90.0 / TURN_SPEED_DEGREES_PER_SECOND * 1000)
                step_start_time_ms = current_time_ms + duration_ms
                robot_states[test_robot_id] = 'turning'

            elif current_action == "LEFT_90":
                print(f"[{current_time_ms}ms] SUPERVISOR: Executing LEFT_90 for Robot {test_robot_id}")
                send_command(test_robot_id, "LEFT")
                duration_ms = int(90.0 / TURN_SPEED_DEGREES_PER_SECOND * 1000)
                step_start_time_ms = current_time_ms + duration_ms
                robot_states[test_robot_id] = 'turning'

            elif current_action == "FORWARD_3000":
                print(f"[{current_time_ms}ms] SUPERVISOR: Executing FORWARD for 3000ms for Robot {test_robot_id}")
                send_command(test_robot_id, "FORWARD")
                duration_ms = 3000 # Move forward for 3 seconds
                step_start_time_ms = current_time_ms + duration_ms
                robot_states[test_robot_id] = 'moving'
                
            elif current_action == "WAIT":
                # Ensure the robot stops before waiting
                if robot_states[test_robot_id] != 'idle':
                    print(f"[{current_time_ms}ms] SUPERVISOR: Stopping Robot {test_robot_id} before WAIT.")
                    send_command(test_robot_id, "STOP")
                    robot_states[test_robot_id] = 'idle'
                
                print(f"[{current_time_ms}ms] SUPERVISOR: Waiting for 1000ms.")
                duration_ms = 1000 # Wait for 1 second
                step_start_time_ms = current_time_ms + duration_ms
                
            elif current_action == "STOP_ALL":
                print(f"[{current_time_ms}ms] SUPERVISOR: All test actions complete. Sending STOP to Robot {test_robot_id}.")
                send_command(test_robot_id, "STOP")
                robot_states[test_robot_id] = 'idle'
                current_test_step_index += 1 # Advance to prevent repeating
                # Optionally, break the loop or go into an infinite idle state
                # while robot.step(timestep) != -1: pass # Freeze simulation
                continue # Stay on this step and keep sending stop if needed

            current_test_step_index += 1
        else:
            # All steps are done. Keep the robot stopped.
            if robot_states[test_robot_id] != 'idle':
                send_command(test_robot_id, "STOP")
                robot_states[test_robot_id] = 'idle'
            # print(f"[{current_time_ms}ms] SUPERVISOR: Test sequence finished. Robot {test_robot_id} is idle.") # Too verbose