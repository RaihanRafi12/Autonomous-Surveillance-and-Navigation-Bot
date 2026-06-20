#!/usr/bin/env python3
import os
import json
import threading
import time
import math
import cv2
import numpy as np
import serial
import pynmea2
import smbus2 as smbus
import onnxruntime as ort
from flask import Flask, render_template, request, jsonify, Response

app = Flask(__name__)

# --- GLOBAL TELEMETRY MATRIX ---
robot_telemetry = {
    "lat": 23.7561,
    "lon": 90.4244,
    "heading": 0.0,
    "status": "Vision Engine Starting...",
    "mode": "Manual",
    "battery": 100,
    
    # Updated Environmental & Spatial Array Parameters
    "temperature": 0.0,
    "humidity": 0.0,
    "gas_level": 0,
    "ir_left": 1,
    "ir_right": 1,
    "ir_back": 1,
    "sonar_left": 400.0,
    "sonar_right": 400.0,
    "sonar_back": 400.0
}

# Global tracker for processed coordinate matrix routes
CURRENT_INDOOR_PATH = []

# --- A* NAVIGATION NODE MATRIX LOGIC ---
class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0  # Operational cost tracking path weight
        self.h = 0  # Heuristic distance metric to goal node
        self.f = 0  # Total node evaluation weight (F = G + H)

    def __eq__(self, other):
        return self.position == other.position

def astar_search(grid, start, end):
    """Calculates the shortest distance path avoiding structural threshold obstructions"""
    start_node = Node(None, start)
    end_node = Node(None, end)
    
    open_list = []
    closed_list = set()
    open_list.append(start_node)
    
    max_y, max_x = grid.shape
    
    # Fully mapped 8-way navigation layout matrix vectors (Y, X, MoveCost)
    movements = [
        (-1, 0, 1.0),  (1, 0, 1.0),  (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)
    ]
    
    while len(open_list) > 0:
        current_node = open_list[0]
        current_index = 0
        for index, item in enumerate(open_list):
            if item.f < current_node.f:
                current_node = item
                current_index = index
                
        open_list.pop(current_index)
        closed_list.add(current_node.position)
        
        # Goal node intercepted: Reconstruct coordinate trajectory trace
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]
            
        for move_y, move_x, cost in movements:
            node_position = (current_node.position[0] + move_y, current_node.position[1] + move_x)
            
            # Boundary layout safety verification check
            if node_position[0] < 0 or node_position[0] >= max_y or node_position[1] < 0 or node_position[1] >= max_x:
                continue
                
            # Obstacle impact verification check (1 = Structural wall constraint block)
            if grid[node_position[0]][node_position[1]] == 1:
                continue
                
            if node_position in closed_list:
                continue
                
            child = Node(current_node, node_position)
            child.g = current_node.g + cost
            child.h = np.sqrt(((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2))
            child.f = child.g + child.h
            
            if any(open_node for open_node in open_list if child == open_node and child.g > open_node.g):
                continue
                
            open_list.append(child)
            
    return None

def process_blueprint_grid(image_path, grid_size=(100, 100)):
    """Extracts layout structures and transforms high-res blueprint pictures into 100x100 binary matrices"""
    try:
        img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            return np.zeros(grid_size, dtype=np.uint8)
            
        resized_img = cv2.resize(img, grid_size, interpolation=cv2.INTER_AREA)
        # Luminance pixel matrix threshold limits (Pixels darker than 120 are processed as wall lines)
        _, binary_grid = cv2.threshold(resized_img, 120, 1, cv2.THRESH_BINARY_INV)
        return binary_grid
    except Exception as e:
        print(f"[WARN]: Image extraction breakdown, defaulting to open layout: {e}")
        return np.zeros(grid_size, dtype=np.uint8)


# --- HARDWARE LINK A: NATIVE GPS SERIAL CHANNEL ---
gps_serial = None
possible_ports = ['/dev/serial0', '/dev/ttyAMA0', '/dev/ttyS0']

for port in possible_ports:
    try:
        gps_serial = serial.Serial(port, baudrate=9600, timeout=0.1)
        print(f"[SUCCESS]: GPS locked and communication channel open on {port}.")
        break
    except Exception:
        continue

if not gps_serial:
    print("[WARN]: GPS Serial offline: All hardware port configurations denied permissions.")

# --- HARDWARE LINK B: NATIVE I2C COMPASS (QMC5883L) ---
bus = None
COMPASS_I2C_ADDR = 0x0D
compass_online = False
try:
    bus = smbus.SMBus(1)
    bus.write_byte_data(COMPASS_I2C_ADDR, 0x09, 0x1D)
    print("[SUCCESS]: Compass communication bus online.")
    compass_online = True
except Exception as e:
    print(f"[WARN]: Compass offline: {e}")

# --- HARDWARE LINK D: ESP32 SERIAL TELEMETRY PORT ---
esp32_serial = None
try:
    # Explicit connection channel targeted to match the 115200 baud stream configuration
    esp32_serial = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
    print("[SUCCESS]: ESP32 communication array connected natively over /dev/ttyUSB0.")
except Exception as e:
    print(f"[WARN]: Serial connection failed on /dev/ttyUSB0: {e}. Checking secondary slots...")


# --- FIXED HARDWARE LINK C: ENHANCED DIRECTORY ROUTING ENGINE ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
MODEL_PATH = os.path.join(BASE_DIR, 'detect.onnx')
LABELS_PATH = os.path.join(BASE_DIR, 'coco_labels.txt')

labels = []
ort_session = None
vision_engine_ready = False

print(f"[DIAGNOSTIC]: Looking for ONNX Model at: {MODEL_PATH}")
print(f"[DIAGNOSTIC]: Looking for Labels file at: {LABELS_PATH}")

try:
    if os.path.exists(LABELS_PATH):
        with open(LABELS_PATH, 'r') as f:
            labels = [line.strip() for line in f.readlines()]
    else:
        print(f"[CRITICAL ERROR]: labels file is missing at {LABELS_PATH}!")
            
    if os.path.exists(MODEL_PATH):
        ort_session = ort.InferenceSession(MODEL_PATH)
        print(f"[SUCCESS]: Loaded ONNX Model natively via absolute base paths.")
        vision_engine_ready = True
    else:
        print(f"[CRITICAL ERROR]: ONNX Model file completely missing at {MODEL_PATH}! AI engine will not start.")
except Exception as e:
    print(f"[WARN]: ONNX Engine initialization failed: {e}. Defaulting to standard stream.")

# --- AUTOMATED BEHAVIOR ROUTING MATRIX ---
def process_robot_reaction(label_text, confidence):
    """Intercepts live vision strings and translates them into actionable hardware state overrides"""
    global robot_telemetry
    
    label_lower = label_text.lower()
    if 'person' in label_lower:
        robot_telemetry["status"] = "EMERGENCY HALT: Human detected in tracking path!"
        send_serial_command("HALT")
    elif 'chair' in label_lower or 'couch' in label_lower:
        robot_telemetry["status"] = f"OBSTACLE WARNING: Swerving to avoid {label_text}."
    elif 'stop sign' in label_lower:
        robot_telemetry["status"] = "STOP SIGN SPOTTED: Holding position for 3 seconds..."
        send_serial_command("HALT")
    else:
        robot_telemetry["status"] = f"AI Target Engine Active | Tracking {label_text.upper()}"

# Helper utility to pass commands down to the ESP32
def send_serial_command(command_str):
    global esp32_serial
    if esp32_serial and esp32_serial.is_open:
        try:
            esp32_serial.write(f"{command_str}\n".encode('utf-8'))
        except Exception as e:
            print(f"[WARN]: Failed to route command downstream across serial link: {e}")

# --- BACKGROUND DATA PROCESSING THREADS ---
def gps_parsing_worker():
    global robot_telemetry, gps_serial
    while True:
        if gps_serial:
            try:
                if gps_serial.in_waiting > 0:
                    raw_line = gps_serial.readline().decode('utf-8', errors='ignore')
                    if raw_line.startswith('$GPGGA') or raw_line.startswith('$GPRMC'):
                        msg = pynmea2.parse(raw_line)
                        if getattr(msg, 'latitude', None) and getattr(msg, 'longitude', None):
                            if msg.latitude != 0.0 and msg.longitude != 0.0:
                                robot_telemetry["lat"] = msg.latitude
                                robot_telemetry["lon"] = msg.longitude
            except Exception:
                pass
        time.sleep(0.05)

def compass_reading_worker():
    global robot_telemetry, bus, compass_online
    while True:
        if compass_online:
            try:
                data = bus.read_i2c_block_data(COMPASS_I2C_ADDR, 0x00, 6)
                raw_x = (data[1] << 8) | data[0]
                raw_y = (data[3] << 8) | data[2]
                if raw_x >= 32768: raw_x -= 65536
                if raw_y >= 32768: raw_y -= 65536
                heading_rad = math.atan2(raw_y, raw_x)
                heading_deg = math.degrees(heading_rad)
                if heading_deg < 0: heading_deg += 360.0
                robot_telemetry["heading"] = round(heading_deg, 1)
            except Exception:
                pass
        else:
            robot_telemetry["heading"] = (robot_telemetry["heading"] + 1) % 360
        time.sleep(0.1)

# --- BACKGROUND INDUSTRIAL SERIAL PORT PROCESSING WORKER ---
def esp32_serial_receiver_worker():
    """Background loop parsing raw JSON telemetry lines directly out of the micro-controller channel"""
    global robot_telemetry, esp32_serial
    while True:
        if esp32_serial and esp32_serial.is_open:
            try:
                if esp32_serial.in_waiting > 0:
                    raw_line = esp32_serial.readline().decode('utf-8', errors='ignore').strip()
                    if raw_line:
                        parsed_data = json.loads(raw_line)
                        
                        # Unpack environmental measurements
                        robot_telemetry["temperature"] = float(parsed_data.get('temperature', robot_telemetry["temperature"]))
                        robot_telemetry["humidity"] = float(parsed_data.get('humidity', robot_telemetry["humidity"]))
                        robot_telemetry["gas_level"] = int(parsed_data.get('gas_level', robot_telemetry["gas_level"]))
                        
                        # Unpack digital IR status metrics
                        robot_telemetry["ir_left"] = int(parsed_data.get('ir_left', robot_telemetry["ir_left"]))
                        robot_telemetry["ir_right"] = int(parsed_data.get('ir_right', robot_telemetry["ir_right"]))
                        robot_telemetry["ir_back"] = int(parsed_data.get('ir_back', robot_telemetry["ir_back"]))
                        
                        # Unpack high-fidelity directional ultrasonic sonar measurements
                        robot_telemetry["sonar_left"] = float(parsed_data.get('sonar_left', robot_telemetry["sonar_left"]))
                        robot_telemetry["sonar_right"] = float(parsed_data.get('sonar_right', robot_telemetry["sonar_right"]))
                        robot_telemetry["sonar_back"] = float(parsed_data.get('sonar_back', robot_telemetry["sonar_back"]))
                        
                        # Evaluate localized security boundaries
                        if (robot_telemetry["sonar_left"] < 20.0 or 
                            robot_telemetry["sonar_right"] < 20.0 or 
                            robot_telemetry["sonar_back"] < 20.0):
                            robot_telemetry["status"] = "PROXIMITY CRITICAL: Ultrasonic obstacle stop triggered!"
                        elif (robot_telemetry["ir_left"] == 0 or 
                              robot_telemetry["ir_right"] == 0 or 
                              robot_telemetry["ir_back"] == 0):
                            robot_telemetry["status"] = "PROXIMITY CRITICAL: IR collision vector active!"
            except json.JSONDecodeError:
                pass
            except Exception as e:
                print(f"[WARN]: Error inside serial processing channel runtime: {e}")
        else:
            time.sleep(2)
            try:
                esp32_serial = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
            except Exception:
                pass
        time.sleep(0.01)

# Initialize background tasks thread maps
threading.Thread(target=gps_parsing_worker, daemon=True).start()
threading.Thread(target=compass_reading_worker, daemon=True).start()
threading.Thread(target=esp32_serial_receiver_worker, daemon=True).start()


# --- LIVE CAMERA STREAM GENERATION ENGINE ---
def generate_camera_frames():
    global vision_engine_ready, ort_session, labels, robot_telemetry
    camera = cv2.VideoCapture(0)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    frame_count = 0
    cached_boxes = []
    
    while True:
        success, frame = camera.read()
        if not success:
            time.sleep(0.03)
            continue
            
        frame_count += 1
        h_f, w_f, _ = frame.shape
        targets_found_in_frame = False
        
        # Inference Step: Execute model logic on even frames
        if vision_engine_ready and ort_session is not None and frame_count % 2 == 0:
            try:
                input_img = cv2.resize(frame, (300, 300))
                input_img = cv2.cvtColor(input_img, cv2.COLOR_BGR2RGB)
                input_data = np.expand_dims(input_img, axis=0).astype(np.uint8)
                
                input_name = ort_session.get_inputs()[0].name
                outputs = ort_session.run(None, {input_name: input_data})
                
                boxes = np.squeeze(outputs[0])
                classes = np.squeeze(outputs[1])
                scores = np.squeeze(outputs[2])
                
                cached_boxes = []
                num_detections = len(scores) if isinstance(scores, (list, np.ndarray)) else 1
                
                for i in range(num_detections):
                    confidence = scores[i] if num_detections > 1 else scores
                    
                    if confidence > 0.35:  
                        class_id = int(classes[i]) if num_detections > 1 else int(classes)
                        label_text = labels[class_id] if class_id < len(labels) else f"ID {class_id}"
                        
                        ymin, xmin, ymax, xmax = boxes[i] if num_detections > 1 else boxes
                        
                        left = int(xmin * w_f)
                        top = int(ymin * h_f)
                        right = int(xmax * w_f)
                        bottom = int(ymax * h_f)
                        
                        left, right = max(0, left), min(w_f, right)
                        top, bottom = max(0, top), min(h_f, bottom)
                        
                        cached_boxes.append({
                            "coords": (left, top, right, bottom),
                            "label": label_text,
                            "conf": confidence
                        })
            except Exception as e:
                pass

        # Rendering Step: Draw target arrays on ALL frames continuously
        if len(cached_boxes) > 0:
            targets_found_in_frame = True
            for target in cached_boxes:
                left, top, right, bottom = target["coords"]
                label_text = target["label"]
                confidence = target["conf"]
                
                display_msg = f"TARGET: {label_text.upper()} ({int(confidence * 100)}%)"
                
                cv2.rectangle(frame, (left, top), (right, bottom), (34, 197, 94), 2)
                
                text_y = top - 10 if top - 10 > 15 else top + 15
                cv2.putText(frame, display_msg, (left, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (34, 197, 94), 2)
                
                if frame_count % 2 == 0:
                    process_robot_reaction(label_text, confidence)
                    
        if not targets_found_in_frame and frame_count % 2 == 0:
            if vision_engine_ready:
                robot_telemetry["status"] = "AI Target Engine Online"
            else:
                robot_telemetry["status"] = "Standard Vision Mode (Model Offline)"

        ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 65])
        frame_bytes = buffer.tobytes()
        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')


# --- WEB APP INFRASTRUCTURE ENDPOINTS ---
@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_camera_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/telemetry', methods=['GET'])
def get_telemetry():
    global robot_telemetry, vision_engine_ready
    if robot_telemetry["status"] == "Vision Engine Starting...":
        robot_telemetry["status"] = "AI Target Engine Online" if vision_engine_ready else "Standard Vision Feed Active"
    return jsonify(robot_telemetry)

@app.route('/api/esp32_telemetry', methods=['POST'])
def receive_esp32_telemetry():
    global robot_telemetry
    data = request.get_json()
    if not data:
        return jsonify({"status": "error", "message": "No JSON matrix received"}), 400

    robot_telemetry["temperature"] = float(data.get('temperature', robot_telemetry["temperature"]))
    robot_telemetry["humidity"] = float(data.get('humidity', robot_telemetry["humidity"]))
    robot_telemetry["gas_level"] = int(data.get('gas_level', robot_telemetry["gas_level"]))
    
    robot_telemetry["ir_left"] = int(data.get('ir_left', robot_telemetry["ir_left"]))
    robot_telemetry["ir_right"] = int(data.get('ir_right', robot_telemetry["ir_right"]))
    robot_telemetry["ir_back"] = int(data.get('ir_back', robot_telemetry["ir_back"]))
    
    robot_telemetry["sonar_left"] = float(data.get('sonar_left', robot_telemetry["sonar_left"]))
    robot_telemetry["sonar_right"] = float(data.get('sonar_right', robot_telemetry["sonar_right"]))
    robot_telemetry["sonar_back"] = float(data.get('sonar_back', robot_telemetry["sonar_back"]))

    if (robot_telemetry["sonar_left"] < 20.0 or 
        robot_telemetry["sonar_right"] < 20.0 or 
        robot_telemetry["sonar_back"] < 20.0 or 
        data.get('ir_obstacle', 1) == 0):
        robot_telemetry["status"] = "PROXIMITY ALERT: Nearby physical block detected!"
        
    return jsonify({"status": "processed"})

@app.route('/api/command', methods=['POST'])
def handle_command():
    """UPDATED: Intercepts action variables from web UI and pipes them down across the serial bus line"""
    global robot_telemetry
    data = request.get_json()
    if not data:
        return jsonify({"status": "error", "message": "Missing command payload"}), 400
        
    command = data.get("command", "").upper()
    robot_telemetry["mode"] = "Manual"
    robot_telemetry["status"] = f"Executing Manual Drive Vector: {command}"
    
    # Route command to the ESP32 via serial link
    send_serial_command(command)
    return jsonify({"status": "received", "direction": command})

@app.route('/api/set_waypoint', methods=['POST'])
def set_waypoint():
    global robot_telemetry
    data = request.json
    robot_telemetry["mode"] = "Autonomous"
    robot_telemetry["status"] = f"GPS Target Locked: {data.get('lat')}, {data.get('lon')}"
    return jsonify({"status": "locked"})

@app.route('/api/set_indoor_route', methods=['POST'])
def set_indoor_route():
    global robot_telemetry, CURRENT_INDOOR_PATH
    data = request.get_json()
    ax = data.get('ax')
    ay = data.get('ay')
    bx = data.get('bx')
    by = data.get('by')
    
    robot_telemetry["mode"] = "Indoor"
    robot_telemetry["status"] = "Calculating Vector Grid Route..."
    
    blueprint_path = os.path.join(app.root_path, 'static', 'floorplan.jpg')
    
    GRID_W, GRID_H = 100, 100
    grid = process_blueprint_grid(blueprint_path, grid_size=(GRID_W, GRID_H))
    
    canvas_w = data.get('canvasWidth', 400)
    canvas_h = data.get('canvasHeight', 300)
    
    start_cell = (int((ay / canvas_h) * GRID_H), int((ax / canvas_w) * GRID_W))
    goal_cell = (int((by / canvas_h) * GRID_H), int((bx / canvas_w) * GRID_W))
    
    start_cell = (max(0, min(GRID_H-1, start_cell[0])), max(0, min(GRID_W-1, start_cell[1])))
    goal_cell = (max(0, min(GRID_H-1, goal_cell[0])), max(0, min(GRID_W-1, goal_cell[1])))
    
    calculated_path = astar_search(grid, start_cell, goal_cell)
    
    if calculated_path:
        CURRENT_INDOOR_PATH = calculated_path
        robot_telemetry["status"] = f"Indoor Route Fixed: {len(calculated_path)} path nodes active."
        
        ui_path = []
        for cell_y, cell_x in calculated_path:
            ui_x = (cell_x / GRID_W) * canvas_w
            ui_y = (cell_y / GRID_H) * canvas_h
            ui_path.append({'x': ui_x, 'y': ui_y})
            
        return jsonify({
            "status": "path_computed", 
            "path": ui_path
        })
    else:
        robot_telemetry["status"] = "Routing Error: Target path completely blocked."
        return jsonify({"status": "error", "message": "No valid structural path found between points."}), 400

@app.route('/api/upload_map', methods=['POST'])
def upload_map():
    if 'file' not in request.files: return jsonify({"error": "No file"}), 400
    file = request.files['file']
    save_path = os.path.join(app.root_path, 'static', 'floorplan.jpg')
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    file.save(save_path)
    return jsonify({"status": "upload_complete"})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)
