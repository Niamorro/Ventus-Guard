const mainCamera = document.getElementById('main-camera');
const usbCamera = document.getElementById('usb-camera');
const debugOutput = document.getElementById('debug-output');
const commandInput = document.getElementById('command-input');
const sendCommandBtn = document.getElementById('send-command');
const ledColorInput = document.getElementById('led-color');
const setLedBtn = document.getElementById('set-led');
const movementDistance = document.getElementById('movement-distance');
const scenarioBlocks = document.getElementById('scenario-blocks');
const addBlockBtn = document.getElementById('add-block');
const runScenarioBtn = document.getElementById('run-scenario');
const clearScenarioBtn = document.getElementById('clear-scenario');
const blockType = document.getElementById('block-type');
const blockValue = document.getElementById('block-value');
const blockDirection = document.getElementById('block-direction');
const detectionStatus = document.getElementById('detection-status').querySelector('span');

let isScenarioRunning = false;

const ws = new WebSocket(`ws://${window.location.host}/ws`);

function updateDroneState(state) {
    document.getElementById('battery').textContent = state.battery.toFixed(2);
    document.getElementById('altitude').textContent = state.altitude.toFixed(2);
    document.getElementById('gps').textContent = `${state.gps.lat.toFixed(6)}, ${state.gps.lon.toFixed(6)}`;
    document.getElementById('status').textContent = state.status;
}

function addDebugMessage(message) {
    const messageElement = document.createElement('div');
    messageElement.textContent = message;
    debugOutput.appendChild(messageElement);
    debugOutput.scrollTop = debugOutput.scrollHeight;
}

function addScenarioBlock() {
    const block = document.createElement('div');
    block.className = 'scenario-block';
    const type = blockType.value;
    const value = blockValue.value;
    const direction = blockDirection.value;

    if (type === 'takeoff') {
        block.textContent = `Takeoff: ${value}m`;
    } else if (type === 'move') {
        block.textContent = `Move ${direction}: ${value}m`;
    } else if (type === 'land') {
        block.textContent = 'Land';
    }

    scenarioBlocks.appendChild(block);
}

async function runScenario() {
    if (isScenarioRunning) return;
    isScenarioRunning = true;
    disableManualControls();

    const blocks = Array.from(scenarioBlocks.children);
    for (let block of blocks) {
        const [action, value] = block.textContent.split(':');
        try {
            if (action.startsWith('Takeoff')) {
                await sendCommand({command: 'takeoff', height: parseFloat(value)});
            } else if (action.startsWith('Move')) {
                const [_, direction] = action.split(' ');
                const distance = parseFloat(value);
                let x = 0, y = 0, z = 0;
                if (direction === 'forward') x = distance;
                if (direction === 'backward') x = -distance;
                if (direction === 'left') y = -distance;
                if (direction === 'right') y = distance;
                if (direction === 'up') z = distance;
                await sendCommand({command: 'navigate', x, y, z, frame_id: 'body'});
            } else if (action === 'Land') {
                await sendCommand({command: 'land'});
            }
            await new Promise(resolve => setTimeout(resolve, 5000));
        } catch (error) {
            console.error('Error executing command:', error);
            addDebugMessage(`Error executing command: ${error.message}`);
            break;
        }
    }

    isScenarioRunning = false;
    enableManualControls();
}

async function sendCommand(command) {
    return new Promise((resolve, reject) => {
        ws.send(JSON.stringify({type: "command", ...command}));
        addDebugMessage(`Sent: ${JSON.stringify(command)}`);
        
        const timeout = setTimeout(() => {
            reject(new Error('Command timeout'));
        }, 4000);

        const handleResponse = (event) => {
            const response = JSON.parse(event.data);
            if (response.type === 'command_result' && response.command === command.command) {
                clearTimeout(timeout);
                ws.removeEventListener('message', handleResponse);
                if (response.success) {
                    resolve(response);
                } else {
                    reject(new Error(response.error || 'Command failed'));
                }
            }
        };

        ws.addEventListener('message', handleResponse);
    });
}

function clearScenario() {
    scenarioBlocks.innerHTML = '';
}

function disableManualControls() {
    document.querySelectorAll('#basic-controls button, #movement-controls button').forEach(btn => btn.disabled = true);
}

function enableManualControls() {
    document.querySelectorAll('#basic-controls button, #movement-controls button').forEach(btn => btn.disabled = false);
}

ws.onmessage = function(event) {
    const data = JSON.parse(event.data);
    if (data.type === 'state') {
        updateDroneState(data.data);
        if (isScenarioRunning) {
            disableManualControls();
        } else {
            enableManualControls();
        }
    } else if (data.type === 'camera') {
        const img = new Image();
        img.onload = function() {
            if (data.name === 'main_camera') {
                mainCamera.src = img.src;
            } else if (data.name === 'usb_camera') {
                usbCamera.src = img.src;
                detectionStatus.textContent = 'Active';
                detectionStatus.style.color = 'green';
            }
        };
        img.src = `data:image/jpeg;base64,${data.data}`;
    }
    addDebugMessage(`Received: ${JSON.stringify(data)}`);
};

['takeoff', 'land'].forEach(action => {
    document.getElementById(action).addEventListener('click', () => {
        if (action === 'takeoff') {
            const height = prompt("Enter takeoff height (in meters):", "1.5");
            if (height !== null) {
                sendCommand({command: action, height: parseFloat(height)});
            }
        } else {
            sendCommand({command: action});
        }
    });
});

['forward', 'backward', 'left', 'right'].forEach(direction => {
    document.getElementById(`move-${direction}`).addEventListener('click', () => {
        const distance = parseFloat(movementDistance.value);
        let x = 0, y = 0;
        if (direction === 'forward') x = distance;
        if (direction === 'backward') x = -distance;
        if (direction === 'left') y = -distance;
        if (direction === 'right') y = distance;
        sendCommand({command: 'navigate', x, y, z: 0, frame_id: 'body'});
    });
});

sendCommandBtn.addEventListener('click', () => {
    const command = commandInput.value;
    if (command) {
        try {
            const commandObj = JSON.parse(command);
            sendCommand(commandObj);
            commandInput.value = '';
        } catch (error) {
            addDebugMessage(`Error: Invalid JSON - ${error.message}`);
        }
    }
});

setLedBtn.addEventListener('click', () => {
    const color = ledColorInput.value;
    const r = parseInt(color.substr(1,2), 16);
    const g = parseInt(color.substr(3,2), 16);
    const b = parseInt(color.substr(5,2), 16);
    const leds = Array(72).fill().map((_, i) => ({r, g, b}));
    sendCommand({command: "set_led", leds});
});

document.querySelectorAll('.tab-button').forEach(button => {
    button.addEventListener('click', () => {
        const tabId = button.getAttribute('data-tab');
        document.querySelectorAll('.tab-button').forEach(btn => btn.classList.remove('active'));
        document.querySelectorAll('.tab-content').forEach(content => content.classList.remove('active'));
        button.classList.add('active');
        const tabContent = document.getElementById(tabId);
        if (tabContent) {
            tabContent.classList.add('active');
        } else {
            console.error(`Tab content not found for id: ${tabId}`);
        }
    });
});

addBlockBtn.addEventListener('click', addScenarioBlock);
runScenarioBtn.addEventListener('click', runScenario);
clearScenarioBtn.addEventListener('click', clearScenario);

ws.onopen = () => addDebugMessage("WebSocket connection opened");
ws.onclose = () => addDebugMessage("WebSocket connection closed");
ws.onerror = (error) => addDebugMessage(`WebSocket error: ${error.message}`);