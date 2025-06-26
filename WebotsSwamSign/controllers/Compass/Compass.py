from controller import Robot
import math

# Create robot instance and get time step
robot = Robot()
timestep = int(robot.getBasicTimeStep())

# Get and enable compass
compass = robot.getDevice("compass")
compass.enable(timestep)

# Function to convert heading to direction label
def get_direction_label(degrees):
    if 45 <= degrees < 135:
        return "East"
    elif 135 <= degrees < 225:
        return "South"
    elif 225 <= degrees < 315:
        return "West"
    else:
        return "North"

# Main loop
while robot.step(timestep) != -1:
    # Read compass values
    compass_values = compass.getValues()

    # Calculate heading (angle from North)
    heading_rad = math.atan2(compass_values[0], compass_values[2])
    heading_deg = (math.degrees(heading_rad) + 360) % 360

    # Get cardinal direction
    direction = get_direction_label(heading_deg)

    # Print result
    print(f"Heading: {heading_deg:.2f}°, Direction: {direction}")
