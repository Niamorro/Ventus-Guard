import asyncio
import json
import base64
import cv2
import numpy as np
import mediapipe as mp
from fastapi import FastAPI, WebSocket, Request
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse, JSONResponse
from datetime import datetime
import os
import threading
import socketio

app = FastAPI()

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
app.mount("/static", StaticFiles(directory=os.path.join(BASE_DIR, "client", "static")), name="static")
templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "client", "templates"))

last_range = None
DISTANCE_THRESHOLD = 0.3
TAKEOFF_HEIGHT = 1.5
takeoff_command_sent = True

drone_ws = None
rover_sio = socketio.AsyncClient()
clients = set()

# Инициализация MediaPipe
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

def process_image(image):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = pose.process(image_rgb)
    
    if results.pose_landmarks:
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        return image, True
    
    return image, False

async def broadcast_image(image_data, name, device):
    _, buffer = cv2.imencode('.jpg', image_data)
    jpg_as_text = base64.b64encode(buffer).decode('utf-8')
    for client in clients:
        await client.send_text(json.dumps({"type": "camera", "name": name, "data": jpg_as_text, "device": device}))

@app.get("/", response_class=HTMLResponse)
async def root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.post("/alice")
async def handle_alice_command(request: Request):
    global takeoff_command_sent
    data = await request.json()
    command = data['request']['command'].lower()
    response_text = "Команда не распознана."
    if "безопасность" in command:
        takeoff_command_sent = False
        response_text = "Защита включена."
    return {
        "response": {
            "text": response_text,
            "end_session": False
        },
        "version": "1.0"
    }

@app.websocket("/ws/rangefinder")
async def rangefinder_websocket(websocket: WebSocket):
    await websocket.accept()
    global rangefinder_ws
    rangefinder_ws = websocket
    try:
        while True:
            data = await websocket.receive_text()
            data = json.loads(data)
            if data.get("type") == "rangefinder":
                await process_rangefinder_data(data)
    finally:
        rangefinder_ws = None

async def process_rangefinder_data(data):
    global last_range, takeoff_command_sent
    current_range = data["data"]["range"]
    if last_range is None:
        last_range = current_range
        return
    if abs(current_range - last_range) > DISTANCE_THRESHOLD and not takeoff_command_sent:
        for client in clients:
            await client.send_text(json.dumps({
                "type": "trigger_scenario",
                "message": "Distance change detected, triggering scenario"
            }))
            print("отправленно")
            takeoff_command_sent = True
    last_range = current_range

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    clients.add(websocket)
    try:
        while True:
            data = await websocket.receive_text()
            await process_command(json.loads(data))
    finally:
        clients.remove(websocket)

async def process_command(command):
    if command.get('type') == 'command':
        if drone_ws:
            await drone_ws.send_text(json.dumps(command))
            try:
                response = await asyncio.wait_for(drone_ws.receive_text(), timeout=1.0)
                response_data = json.loads(response)
                for client in clients:
                    await client.send_text(json.dumps({
                        "type": "command_result",
                        "command": command["command"],
                        "success": response_data.get("success", False),
                        "error": response_data.get("error")
                    }))
            except asyncio.TimeoutError:
                for client in clients:
                    await client.send_text(json.dumps({
                        "type": "command_result",
                        "command": command["command"],
                        "success": False,
                        "error": "Drone response timeout"
                    }))
            except Exception as e:
                for client in clients:
                    await client.send_text(json.dumps({
                        "type": "command_result",
                        "command": command["command"],
                        "success": False,
                        "error": str(e)
                    }))
    elif command.get('type') == 'rover_command':
        if rover_sio.connected:
            try:
                response = await rover_sio.call('command', command, namespace='/vehicles')
                for client in clients:
                    await client.send_text(json.dumps({
                        "type": "command_result",
                        "command": command["command"],
                        "success": response.get("success", False),
                        "error": response.get("error")
                    }))
            except Exception as e:
                for client in clients:
                    await client.send_text(json.dumps({
                        "type": "command_result",
                        "command": command["command"],
                        "success": False,
                        "error": str(e)
                    }))

@app.websocket("/ws/drone")
async def drone_websocket(websocket: WebSocket):
    await websocket.accept()
    global drone_ws
    drone_ws = websocket
    try:
        while True:
            data = await websocket.receive_text()
            data = json.loads(data)
            if data.get("type") == "state":
                await broadcast_state(data, 'drone')
            elif data.get("type") == "camera":
                image_data = base64.b64decode(data["data"])
                nparr = np.frombuffer(image_data, np.uint8)
                image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                processed_image, human_detected = process_image(image)
                
                if human_detected:
                    save_detection(processed_image, 'drone')
                
                await broadcast_image(processed_image, data["name"], 'drone')
    finally:
        drone_ws = None

async def broadcast_state(state, device):
    for client in clients:
        await client.send_text(json.dumps({"type": "state", "data": state["data"], "device": device}))

def save_detection(image, device):
    os.makedirs("detections", exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"detection_{device}_{timestamp}.jpg"
    cv2.imwrite(f"detections/{filename}", image)

@app.post("/save-detection")
async def save_detection_endpoint(request: Request):
    data = await request.json()
    image_data = base64.b64decode(data['image'].split(',')[1])
    nparr = np.frombuffer(image_data, np.uint8)
    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    save_detection(image, data.get('device', 'unknown'))
    return JSONResponse(content={"message": "Detection saved"})

@rover_sio.event
async def connect():
    print('Connected to Rover')

@rover_sio.event
async def disconnect():
    print('Disconnected from Rover')

@rover_sio.on('state', namespace='/vehicles')
async def on_rover_state(data):
    await broadcast_state(data, 'rover')

async def process_rover_video():
    cap = cv2.VideoCapture('rtsp://192.168.10.115:8554/camera1')
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        
        processed_frame, human_detected = process_image(frame)
        
        if human_detected:
            save_detection(processed_frame, 'rover')
        
        await broadcast_image(processed_frame, 'rover_camera', 'rover')
        
        await asyncio.sleep(0.1)  # Adjust the delay as needed

async def startup_event():
    await rover_sio.connect('http://192.168.10.115:3006', namespaces=['/vehicles'])
    asyncio.create_task(process_rover_video())

@app.on_event("startup")
async def startup():
    asyncio.create_task(startup_event())

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)