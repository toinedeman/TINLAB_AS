from controller import Robot

# Stapgrootte voor simulatie
TIME_STEP = 64

# Initialiseer robot
robot = Robot()

# Camera ophalen
camera = robot.getDevice("camera")
camera.enable(TIME_STEP)

# Loop waarin het cameravenster actief blijft
while robot.step(TIME_STEP) != -1:
    pass  # Niets doen, alleen zorgen dat camera blijft draaien
