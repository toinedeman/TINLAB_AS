from controller import Supervisor

robot = Supervisor()
timestep = int(robot.getBasicTimeStep())

# Communication setup
receiver = robot.getDevice("receiver")
receiver.enable(timestep)

# Get robot info
robot_node = robot.getSelf()
translation_field = robot_node.getField("translation")
robot_name = robot.getName()
my_id = int(robot_name.split('_')[1])  # Extracts 6 from FysicalBot_6


while robot.step(timestep) != -1:
    while receiver.getQueueLength() > 0:
        msg = receiver.getString().strip()  # Get and clean message
        
        # Expected format: "ID:GOTO:X:Y"
        if msg.count(':') == 3:  # Check for exactly 3 colons
            parts = msg.split(':')
            try:
                received_id = int(parts[0])
                if received_id == my_id and parts[1] == "GOTO":
                    x = float(parts[2])
                    y = float(parts[3])
                    
                    # Update position (z=0.01 for slight elevation)
                    translation_field.setSFVec3f([x, y, 0.01])
                    print(f"{robot_name} moved to ({x}, {y})")
            except (ValueError, IndexError):
                pass  # Silently ignore malformed messages
        
        receiver.nextPacket()