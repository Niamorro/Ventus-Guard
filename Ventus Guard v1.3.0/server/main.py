from fastapi import FastAPI, WebSocket, Request
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from fastapi.responses import HTMLResponse, JSONResponse
from datetime import datetime
import base64
import asyncio
import json
import os

app = FastAPI()

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
app.mount("/static", StaticFiles(directory=os.path.join(BASE_DIR, "client", "static")), name="static")
templates = Jinja2Templates(directory=os.path.join(BASE_DIR, "client", "templates"))

drone_ws = None
clients = set()

@app.get("/", response_class=HTMLResponse)
async def root(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

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
            # Use asyncio.wait_for with a shorter timeout
            response = await asyncio.wait_for(drone_ws.receive_text(), timeout=1.0)
            response_data = json.loads(response)
            # Send the result back to the client
            for client in clients:
                await client.send_text(json.dumps({
                    "type": "command_result",
                    "command": command["command"],
                    "success": response_data.get("success", False),
                    "error": response_data.get("error")
                }))
        except asyncio.TimeoutError:
            # If we don't receive a response in time, send an error to the client
            for client in clients:
                await client.send_text(json.dumps({
                    "type": "command_result",
                    "command": command["command"],
                    "success": False,
                    "error": "Drone response timeout"
                }))
        except Exception as e:
            # Handle any other exceptions
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
                await broadcast_image(data)
    finally:
        drone_ws = None

async def broadcast_state(state):
    for client in clients:
        await client.send_text(json.dumps({"type": "state", "data": state["data"]}))

async def broadcast_image(image_data):
    for client in clients:
        await client.send_text(json.dumps({"type": "camera", "name": image_data["name"], "data": image_data["data"]}))

@app.post("/save-detection")
async def save_detection(request: Request):
    data = await request.json()
    image_data = data['image'].split(',')[1]
    image_bytes = base64.b64decode(image_data)
    
    # Создаем папку для сохранения изображений, если она не существует
    os.makedirs("detections", exist_ok=True)
    
    # Сохранение изображения
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"detection_{timestamp}.jpg"
    with open(f"detections/{filename}", "wb") as f:
        f.write(image_bytes)
    
    return JSONResponse(content={"message": "Detection saved"})

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)