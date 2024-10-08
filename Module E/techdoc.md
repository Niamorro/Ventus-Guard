# **VentusGuard Drone & Rover Control - Полная Документация**

## **1. О проекте**

VentusGuard Drone & Rover Control — это многофункциональное веб-приложение, разработанное для удаленного управления дроном и ровером с использованием современного стек технологий, таких как FastAPI, WebSocket, ROS1 и Clover. Этот проект ориентирован на решение задач, связанных с автоматизированным управлением беспилотниками и наземными роботами, предоставляя пользователю возможность создавать сценарии для автономных миссий, управлять устройствами вручную и отслеживать их состояние в реальном времени.

### **1.1. Основные функции**
- **Управление дроном и ровером**: Взлет и посадка дрона, передвижение ровера, управление светодиодами, настройка высоты и скорости.
- **Просмотр видеопотоков**: Прямой видеопоток с камеры дрона и USB камеры для мониторинга обстановки.
- **Автономные сценарии**: Пользователи могут создавать сложные сценарии, задавая последовательность действий для дрона и ровера, и выполнять их автоматически.
- **Двусторонняя связь с устройствами**: Использование WebSocket для передачи команд и получения информации от дрона и ровера без задержек.
- **Интерактивный интерфейс**: Веб-интерфейс позволяет пользователю управлять устройствами через кнопки, предоставляя удобный способ контроля.

### **1.2. Интересные задачи, решенные в проекте**
- **Реализация управления реального времени**: Использование WebSocket для непрерывной передачи данных между клиентом и сервером позволяет пользователю управлять дронами и роверами без задержек.
- **Интеграция с ROS**: Система использует ROS (Robot Operating System) для управления Clover дроном, что упрощает взаимодействие с реальными роботами и дронами в полевых условиях.
- **Автоматизация полетов и перемещений**: Возможность заранее задавать маршруты и действия для дронов и роботов открывает возможности для использования в спасательных операциях, наблюдении и других задачах.

## **2. Технологии**

### **2.1 FastAPI**
FastAPI — это современный, быстрый (high-performance), веб-фреймворк для создания API на Python 3.6+ на основе стандартов Python type hints. В проекте VentusGuard он используется для создания RESTful API и WebSocket-сервера, который обрабатывает запросы от клиента и отправляет команды на дрон и ровер.

**Основные преимущества FastAPI**:
- **Высокая скорость работы**: В основе FastAPI лежит библиотека Starlette, которая делает сервер быстрым и отзывчивым, что критично для управления устройствами в реальном времени.
- **Поддержка WebSocket**: FastAPI поддерживает WebSocket «из коробки», что позволяет легко организовать двустороннюю передачу данных между клиентом и сервером.
- **Простота разработки**: FastAPI использует аннотации типов, что упрощает процесс разработки и повышает читаемость кода.

```python
from fastapi import FastAPI, WebSocket

app = FastAPI()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        data = await websocket.receive_text()
        await websocket.send_text(f"Received: {data}")
```
В приведённом коде реализован сервер WebSocket, который принимает сообщения от клиента и отправляет обратно подтверждение. Эта структура используется для обработки команд управления дронами и роверами.

### **2.2 WebSocket**
WebSocket — это протокол связи, который обеспечивает постоянное двустороннее соединение между клиентом и сервером. В проекте WebSocket используется для передачи данных реального времени между веб-клиентом и сервером FastAPI, что позволяет управлять дроном и ровером мгновенно, а также получать актуальные данные от устройств, такие как видеопотоки и состояние батареи.

**Преимущества WebSocket**:
- **Поддержка реального времени**: WebSocket идеально подходит для приложений, которые требуют мгновенной реакции, таких как управление беспилотниками и передача видеопотоков.
- **Эффективное использование ресурсов**: В отличие от HTTP, который использует запросы и ответы, WebSocket поддерживает постоянное соединение, что снижает задержки и потребление ресурсов.

### **2.3 ROS (Robot Operating System)**
ROS — это набор фреймворков и инструментов для разработки программного обеспечения для роботов. В данном проекте ROS1 используется для управления дроном через Clover, предоставляя возможность взаимодействия с реальными устройствами.

**Основные компоненты ROS**:
- **Ноды**: Ноды — это программы, которые взаимодействуют друг с другом, передавая сообщения. В проекте каждая нода отвечает за определённую задачу, такую как управление движением дрона или получение данных с сенсоров.
- **Топики**: Топики используются для обмена данными между нодами. Например, камера дрона передает данные через топик, который может быть прочитан другими нодами.
- **Службы (Services)**: Службы в ROS позволяют нодам выполнять синхронные вызовы. В данном проекте это используется для таких команд, как взлет и посадка.

Пример вызова службы для взлета дрона:
```python
import rospy
from clover import srv

rospy.init_node('drone_controller')
takeoff_srv = rospy.ServiceProxy('takeoff', srv.Takeoff)
takeoff_srv(height=1.5)
```

### **2.4 Clover**
Clover — это программная платформа для дронов на базе ROS, которая упрощает создание автономных систем. Clover поддерживает такие функции, как автоматизированное управление дроном, работа с камерой, интеграция с ROS и многое другое. В проекте Clover используется для работы с дроном, предоставляя возможности по управлению полетом и съемке.

---

## **3. Структура проекта**

Проект VentusGuard состоит из нескольких ключевых файлов и директорий, каждая из которых отвечает за свою часть функциональности.

- **static/**: Эта директория содержит статические файлы проекта, такие как CSS стили и JavaScript файлы, которые используются для отображения интерфейса и реализации клиентской логики.
  - **css/style.css**: Файл со стилями для пользовательского интерфейса.
  - **js/app.js**: Основной JavaScript файл, который управляет взаимодействием с сервером через WebSocket и обновлением интерфейса.
  
- **templates/**: Директория для HTML шаблонов, использующихся FastAPI для отображения веб-страниц.
  - **index.html**: Главный шаблон страницы, который включает все элементы управления дроном и ровером.

- **drone.py**: Python файл, отвечающий за управление дроном с использованием ROS. В этом файле вызываются сервисы ROS для выполнения действий, таких как взлет, посадка и изменение положения дрона.

- **rover.py**: Файл, отвечающий за управление ровером, включая функции передвижения и манипуляции светодиодами.

- **main.py**: Основной серверный файл FastAPI, который отвечает за обработку запросов от клиента, создание WebSocket соединений и маршрутизацию страниц.

---

## **4. Объяснение кода**

### **4.1 main.py**

Файл `main.py` отвечает за настройку и запуск FastAPI сервера, обработку запросов и реализацию WebSocket соединений.

Пример:
```python
from fastapi import FastAPI, WebSocket, Request
from fastapi.templating import Jinja2Templates

app = FastAPI()
templates = Jinja2Templates(directory="templates")

@app.get("/")
async def get(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})
```

Эта часть кода настраивает сервер для отображения главной страницы по URL `/`. Когда пользователь заходит на сайт, сервер возвращает шаблон `index.html`.

### **4.2 WebSocket обработчик**

Для обеспечения управления дроном в реальном времени, используется WebSocket:
```python
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    while True:
        data = await websocket.receive_text()
        await websocket.send_text(f"Received: {data}")
```

Этот код принимает данные от клиента и отправляет обратно подтверждение. В реальном приложении через WebSocket передаются команды на управление дроном и ровером, а также видеопоток и данные с устройств.

### **4.3 Управление дроном через ROS**

Файл `drone.py` взаимодействует с ROS, чтобы управлять дроном. Например, взлет на заданную высоту:
```python
import rospy
from clover import srv

rospy.init_node('drone_controller')
takeoff_srv = rospy.ServiceProxy('takeoff', srv.Takeoff)

def takeoff(height):
    takeoff_srv(height=height)
```

Эта функция

 вызывает сервис взлета дрона, передавая высоту в качестве параметра.

---

## **5. Использование WebSocket для управления в реальном времени**

Основной причиной выбора WebSocket для управления дронами и роверами является необходимость постоянного двустороннего соединения между клиентом и сервером, что позволяет минимизировать задержки при отправке команд и получении данных.

---

## **Заключение**

Проект VentusGuard демонстрирует, как можно эффективно использовать современный стек технологий для управления автономными системами, такими как дроны и роботы. FastAPI обеспечивает высокопроизводительный сервер, WebSocket предоставляет низколатентную связь в реальном времени, а интеграция с ROS и Clover позволяет взаимодействовать с реальными устройствами.