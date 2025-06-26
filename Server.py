from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import random
import paho.mqtt.client as mqtt
from threading import Lock

class MyHandler(BaseHTTPRequestHandler):
    _handlers = []
    _dimensions_lock = Lock()
    _field_width = 10
    _field_height = 10
    _digital_width = 25  # Fixed digital field dimensions
    _digital_height = 25
    _border_pixels = 50  # Border size in pixels
    
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        with self._dimensions_lock:
            self._handlers.append(self)
    
    def log_message(self, format, *args):
        pass
    
    @classmethod
    def update_dimensions(cls, width, height):
        with cls._dimensions_lock:
            cls._field_width = width
            cls._field_height = height
            print(f"Updated physical dimensions to: {width}x{height}")

    def do_OPTIONS(self):
        self.send_response(200, "ok")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "POST, GET, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_POST(self):
        try:
            content_length = int(self.headers.get('Content-Length', 0))
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data)
            character = data.get("message", "").upper()
            
            if len(character) != 1:
                raise ValueError("Please enter exactly one character")
            
            print(f"\nCharacter received: {character}")
            
            # Get current dimensions
            with self._dimensions_lock:
                physical_width = self._field_width
                physical_height = self._field_height
                digital_width = self._digital_width
                digital_height = self._digital_height
                border_pixels = self._border_pixels
                print(f"Physical dimensions: {physical_width}x{physical_height}")
                print(f"Digital dimensions: {digital_width}x{digital_height}")
                print(f"Border size: {border_pixels} pixels")
            
            # Calculate positions
            normalized_positions = self.calculate_robot_positions(character)
            print("\nNormalized positions (0-10 scale):")
            print(json.dumps(normalized_positions, indent=2))
            
            # Scale positions with border for physical, without border for digital
            scaled_positions = self.scale_positions(normalized_positions)
            print("\nScaled positions:")
            print(json.dumps(scaled_positions, indent=2))
            
            # Store and send positions
            self.server.last_message = character
            self.server.last_positions = scaled_positions
            self.send_to_robots(scaled_positions)
            
            self.send_response(200)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
        except Exception as e:
            print(f"POST error: {e}")
            self.send_error(400, str(e))

    def scale_positions(self, positions):
        """Scale positions from normalized 0-10 coordinates to actual field dimensions"""
        scaled = {"physical": {}, "digital": {}}
        
        with self._dimensions_lock:
            physical_width = self._field_width
            physical_height = self._field_height
            digital_width = self._digital_width
            digital_height = self._digital_height
            border_pixels = self._border_pixels
        
        print(f"\nScaling positions using:")
        print(f"Physical dimensions: {physical_width}x{physical_height}")
        print(f"Digital dimensions: {digital_width}x{digital_height}")
        print(f"Border size: {border_pixels} pixels (for physical only)")
        
        # Calculate playable area for physical field (subtracting borders)
        physical_play_width = max(0, physical_width - 2 * border_pixels)
        physical_play_height = max(0, physical_height - 2 * border_pixels)
        
        # Scale physical robots (using physical dimensions with border)
        for robot_id, pos in positions["physical"].items():
            # Scale to playable area first (0-10 to 0-playable_size)
            play_x = pos["x"] * (physical_play_width / 10)
            play_y = pos["y"] * (physical_play_height / 10)
            
            # Add border offset
            scaled["physical"][robot_id] = {
                "x": round(play_x + border_pixels),
                "y": round(play_y + border_pixels)
            }
        
        # Scale digital robots (using full digital dimensions without border)
        for robot_id, pos in positions["digital"].items():
            # Scale directly from 0-10 to 0-digital_width/height
            scaled["digital"][robot_id] = {
                "x": round(pos["x"] * (digital_width / 10)),
                "y": round(pos["y"] * (digital_height / 10))
            }
        
        return scaled

    def calculate_robot_positions(self, character):
        """Calculate positions in normalized 0-10 coordinate system"""
        try:
            with open('formations.json') as f:
                formations = json.load(f)
        except Exception as e:
            print(f"Error loading formations: {e}")
            formations = {}
        
        if character in formations:
            formation = formations[character]
            print(f"Using formation for character: {character}")
        else:
            formation = formations.get("?", {})
            print(f"Using default formation for character: {character}")
        
        all_positions = list(formation.values())
        
        # Ensure we have exactly 8 unique positions
        if len(all_positions) > 8:
            all_positions = random.sample(all_positions, 8)
        elif len(all_positions) < 8:
            # Add slight offsets to duplicates to prevent overlapping
            needed = 8 - len(all_positions)
            duplicates = random.choices(all_positions, k=needed)
            for i, pos in enumerate(duplicates):
                offset = (i+1)*0.1  # Small offset
                all_positions.append({
                    "x": min(10, pos["x"] + offset),
                    "y": min(10, pos["y"] + offset)
                })
        
        random.shuffle(all_positions)
        
        return {
            "physical": {
                "1": all_positions[0],
                "2": all_positions[1],
                "3": all_positions[2]
            },
            "digital": {
                "1": all_positions[3],
                "2": all_positions[4],
                "3": all_positions[5],
                "4": all_positions[6],
                "5": all_positions[7]
            }
        }

    def send_to_robots(self, positions):
        print("\nSending scaled positions to robots:")
        print(json.dumps(positions, indent=2))
        
        # Send to physical robots
        for robot_id, pos in positions["physical"].items():
            topic = f"robot/targetpos/{robot_id}"
            payload = json.dumps(pos)
            print(f"Publishing to {topic}: {payload}")
            self.server.mqtt_client.publish(topic, payload)
        
        # Send to digital robots
        for robot_id, pos in positions["digital"].items():
            topic = f"digital/robot/targetpos/{robot_id}"
            payload = json.dumps(pos)
            print(f"Publishing to {topic}: {payload}")
            self.server.mqtt_client.publish(topic, payload)

    def do_GET(self):
        try:
            self.send_response(200)
            self.send_header("Content-type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            
            with self._dimensions_lock:
                response = {
                    "message": self.server.last_message,
                    "positions": getattr(self.server, "last_positions", {}),
                    "field_dimensions": {
                        "physical": {
                            "width": self._field_width,
                            "height": self._field_height
                        },
                        "digital": {
                            "width": self._digital_width,
                            "height": self._digital_height
                        }
                    },
                    "border_size": self._border_pixels
                }
            self.wfile.write(json.dumps(response).encode())
        except Exception as e:
            print(f"GET error: {e}")

class MyHTTPServer(HTTPServer):
    def __init__(self, *args, **kwargs):
        self.last_message = ""
        self.last_positions = {}
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.connect("192.168.4.1", 1883, 60)
        self.mqtt_client.loop_start()
        super().__init__(*args, **kwargs)

def on_mqtt_message(client, userdata, msg):
    if msg.topic == "robot/frame/dimensions":
        try:
            payload = msg.payload.decode('utf-8').strip()
            print(f"\nReceived dimensions payload: '{payload}'")
            
            if ',' in payload:
                width, height = map(float, payload.split(','))
            else:
                width, height = map(float, payload.split())
            
            print(f"Parsed dimensions - width: {width}, height: {height}")
            
            MyHandler.update_dimensions(width, height)
                
        except Exception as e:
            print(f"Error processing dimensions: {e}")

def run():
    server_address = ('', 8000)
    httpd = MyHTTPServer(server_address, MyHandler)
    
    # Setup MQTT
    httpd.mqtt_client.on_message = on_mqtt_message
    httpd.mqtt_client.subscribe("robot/frame/dimensions")
    httpd.mqtt_client.loop_start()
    
    print('Server running on http://localhost:8000...')
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        httpd.mqtt_client.loop_stop()
        httpd.server_close()

if __name__ == "__main__":
    run()