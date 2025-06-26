import requests
import threading
from controller import Robot

robot = Robot()
timestep = int(robot.getBasicTimeStep())

last_message = ""
counter = 0
check_interval = 1000 // timestep  # 1x per seconde

latest_server_message = None

lock = threading.Lock()
print_lock = threading.Lock()

# Flag om te voorkomen dat meerdere fetch threads tegelijk lopen
fetching = False

def safe_print(*args, **kwargs):
    with print_lock:
        print(*args, **kwargs)

def fetch_message():
    global latest_server_message, fetching
    try:
        response = requests.get("http://localhost:8000", timeout=1)  # 1 sec timeout i.p.v. 0.2
        data = response.json()
        message = data.get("message", "")
        with lock:
            latest_server_message = message
    except requests.exceptions.RequestException as e:
        with lock:
            latest_server_message = None
        safe_print("Fetch error:", e)
    finally:
        fetching = False  # Zet flag weer uit

while robot.step(timestep) != -1:
    counter += 1

    if counter >= check_interval:
        counter = 0
        if not fetching:  # Start alleen nieuwe fetch als vorige klaar is
            fetching = True
            threading.Thread(target=fetch_message, daemon=True).start()

    with lock:
        if latest_server_message and latest_server_message != last_message:
            safe_print("Webots ontving:", latest_server_message)
            last_message = latest_server_message
            latest_server_message = None
