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

app = FastAPI()

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
app.mount("/static", StaticFiles(directory=os.path.join(BASE_DIR, "client", "static")), name="static")
templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "client", "templates"))

last_range = None
DISTANCE_THRESHOLD = 0.3  # Пороговое значение изменения дистанции в метрах
TAKEOFF_HEIGHT = 1.5  # Высота взлета в метрах
takeoff_command_sent = True  # Флаг для отслеживания отправки команды взлета

drone_ws = None
clients = set()

# Инициализация MediaPipe
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(static_image_mode=False, min_detection_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

def process_image(image):
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    results = pose.process(image_rgb)
    
    if results.pose_landmarks:
        # Рисуем скелет на изображении
        mp_drawing.draw_landmarks(image, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        return image, True
    
    return image, False

async def broadcast_image(image_data, name):
    _, buffer = cv2.imencode('.jpg', image_data)
    jpg_as_text = base64.b64encode(buffer).decode('utf-8')
    for client in clients:
        await client.send_text(json.dumps({"type": "camera", "name": name, "data": jpg_as_text}))

@app.get("/", response_class=HTMLResponse)
async def root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.post("/alice")
async def handle_alice_command(request: Request):
    global takeoff_command_sent
    data = await request.json()

    # Извлекаем команду из запроса
    command = data['request']['command'].lower()

    # Ответ Алисе
    response_text = "Команда не распознана."

    # Обрабатываем команды
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

    # Проверяем, изменилась ли дистанция больше, чем на пороговое значение
    if abs(current_range - last_range) > DISTANCE_THRESHOLD and not takeoff_command_sent:
        # Если дистанция значительно изменилась и команда еще не была отправлена
        for client in clients:
            await client.send_text(json.dumps({
                "type": "trigger_scenario",
                "message": "Distance change detected, triggering scenario"
            }))
            print("отправленно")
            takeoff_command_sent = True
        

    # Обновляем последнее известное значение дистанции
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
    global drone_ws
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
                await broadcast_state(data)
            elif data.get("type") == "camera":
                image_data = base64.b64decode(data["data"])
                nparr = np.frombuffer(image_data, np.uint8)
                image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                
                processed_image, human_detected = process_image(image)
                
                if human_detected:
                    # Сохраняем изображение с обнаруженным человеком
                    save_detection(processed_image)
                
                await broadcast_image(processed_image, data["name"])
    finally:
        drone_ws = None

async def broadcast_state(state):
    for client in clients:
        await client.send_text(json.dumps({"type": "state", "data": state["data"]}))

def save_detection(image):
    os.makedirs("detections", exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"detection_{timestamp}.jpg"
    cv2.imwrite(f"detections/{filename}", image)

@app.post("/save-detection")
async def save_detection_endpoint(request: Request):
    data = await request.json()
    image_data = base64.b64decode(data['image'].split(',')[1])
    nparr = np.frombuffer(image_data, np.uint8)
    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
    save_detection(image)
    return JSONResponse(content={"message": "Detection saved"})

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)