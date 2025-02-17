// 1. Command Handling
function sendCommand(action, value = 1.0) {
    fetch(`/send_command/?command=${action}&value=${value}`)
        .then(response => response.json())
        .then(data => {
            console.log(`Command sent: ${data.command}, Status: ${data.status}`);
            addCommandToHistory(`Command: ${data.command}, Value: ${value}`);
        })
        .catch(error => console.error("Error sending command:", error));
}

function sendControllerCommand(command, value = 1.0) {
    fetch(`/send_controller_command/?command=${command}&value=${value}`)
        .then(response => response.json())
        .then(data => {
            console.log(`Controller Command sent: ${data.command}, Value: ${data.value}`);
            addCommandToHistory(`Controller: ${data.command}, Value: ${value}`);
        })
        .catch(error => console.error("Error sending controller command:", error));
}

function addCommandToHistory(commandText) {
    const historyBox = document.getElementById("command-history");
    const newCommand = document.createElement("li");
    newCommand.textContent = commandText;
    historyBox.appendChild(newCommand);
    
    // Auto-scroll to the latest command
    const container = document.getElementById("command-history-box");
    container.scrollTop = container.scrollHeight;
}

// 2. Keyboard Controls
function updateKeyDisplay(key, isPressed) {
    const keyElement = document.getElementById(`key${key}`);
    if (keyElement) {
        keyElement.classList.toggle('active', isPressed);
    }
}

document.addEventListener('keydown', (e) => {
    switch (e.key.toLowerCase()) {
        case 'w': sendCommand('forward'); updateKeyDisplay('W', true); break;
        case 's': sendCommand('backward'); updateKeyDisplay('S', true); break;
        case 'a': sendCommand('left'); updateKeyDisplay('A', true); break;
        case 'd': sendCommand('right'); updateKeyDisplay('D', true); break;
        case 'x': sendCommand('stop'); break;
        case 'c': sendCommand('center'); break;
    }
});

document.addEventListener('keyup', (e) => {
    switch (e.key.toLowerCase()) {
        case 'w':
        case 's':
        case 'a':
        case 'd':
            updateKeyDisplay(e.key.toUpperCase(), false);
            break;
    }
});

// 3. Gamepad Handling
let controllerConnected = false;
let drivingMode = "forward";
const deadzone = 0.1;

function detectGamepad() {
    const gamepads = navigator.getGamepads();
    let gamepadConnected = false;

    for (const gamepad of gamepads) {
        if (gamepad) {
            gamepadConnected = true;
            processGamepadInput(gamepad);
            break;
        }
    }

    // Update UI
    const gamepadStatus = document.getElementById("status-controller");
    if (gamepadConnected) {
        gamepadStatus.textContent = "Connected";
        gamepadStatus.style.color = "green";
    } else {
        gamepadStatus.textContent = "No Gamepad Detected";
        gamepadStatus.style.color = "red";
    }
}

// Function to update the Forward/Reverse Toggle Switch
function updateDriveModeIndicator() {
    const driveModeSwitch = document.getElementById("driveModeSwitch");
    if (drivingMode === "forward") {
        driveModeSwitch.classList.add("drive-mode-forward");
        driveModeSwitch.classList.remove("drive-mode-reverse");
    } else {
        driveModeSwitch.classList.add("drive-mode-reverse");
        driveModeSwitch.classList.remove("drive-mode-forward");
    }
}

function processGamepadInput(gamepad) {
    if (!gamepad) return;

    let acceleratorRaw = gamepad.axes[2];
    let brakeRaw = gamepad.axes[5];

    let accelerator = (1 - acceleratorRaw) / 2;
    let brake = (1 - brakeRaw) / 2;

    if (accelerator < deadzone) accelerator = 0;
    if (brake < deadzone) brake = 0;

    let steering = gamepad.axes[0] * 360;

    // Change drive mode with buttons
    if (gamepad.buttons[12].pressed) {
        drivingMode = "forward";
        updateDriveModeIndicator();
    }
    if (gamepad.buttons[13].pressed) {
        drivingMode = "reverse";
        updateDriveModeIndicator();
    }

    updateSteeringWheel(steering);
    updatePedals(accelerator, brake);
}

function handleGamepadInput() {
    const gamepads = navigator.getGamepads();
    if (!gamepads) return;

    const gp = gamepads[0];
    if (!gp) return;

    let steering = gp.axes[0];
    let accelerator = gp.axes[2]; 
    let brake = gp.axes[5];      

    // Use the global drivingMode instead of local carDirection
    let direction = drivingMode;

    // Steering from [-1,1] => [0,180]
    let steeringAngle = Math.round(((steering + 1) / 2) * 180);

    // Convert accelerator [-1..1] => [0..1] if pressed, or 0 if released
    let accelValue = accelerator < 0 ? Math.abs(accelerator) : 0.0;

    // Convert brake [-1..1] => [0..1] if pressed, else 0
    let brakeValue = brake > 0 ? 0.0 : Math.abs((brake + 1) / 2);

    // Send steering command only if changed
    if (previousSteering !== steeringAngle) {
        sendControllerCommand("steering", steeringAngle);
        previousSteering = steeringAngle;
    }

    // Handle accelerator release (any direction)
    if (accelValue < 0.3 && previousAccelerator !== accelValue) {
        sendControllerCommand("accelerator", 0.0); // Always send 0.0 when releasing the pedal
        previousAccelerator = accelValue;
    }

    // Apply acceleration based on global drivingMode
    if (accelValue > 0.3 && previousAccelerator !== accelValue) {
        if (direction === "forward") {
            sendControllerCommand("accelerator", accelValue);
        } else if (direction === "reverse") {
            // Apply the -0.1 adjustment only when actively reversing
            let reverseValue = -(accelValue - 0.1);
            sendControllerCommand("accelerator", reverseValue);
        }
        previousAccelerator = accelValue;
    }

    // Handle brake
    if (brakeValue > 0 && previousBrake !== brakeValue) {
        sendControllerCommand("brake", brakeValue);
        previousBrake = brakeValue;
    }

    requestAnimationFrame(handleGamepadInput);
}

// Polling + handleGamepadInput
let gamepadPolling = setInterval(() => {
    const gamepads = navigator.getGamepads();
    if (gamepads[0]) {
        try {
            processGamepadInput(gamepads[0]);
        } catch (error) {
            console.error("Gamepad polling stopped due to server error:", error);
            clearInterval(gamepadPolling);
        }
    }
    handleGamepadInput();
}, 100);

function pollGamepad() {
    const gamepads = navigator.getGamepads();
    if (gamepads && gamepads[0]) {
        processGamepadInput(gamepads[0]);
    }
    requestAnimationFrame(pollGamepad);
}

// Gamepad Events
window.addEventListener("gamepadconnected", (event) => {
    console.log("Gamepad connected:", event.gamepad.id);
    previousSteering = null;
    previousAccelerator = null;
    previousBrake = null;
    requestAnimationFrame(handleGamepadInput);
});

window.addEventListener("gamepaddisconnected", () => console.log("Gamepad disconnected."));

window.addEventListener("beforeunload", () => {
    cancelAnimationFrame(handleGamepadInput);
});


// 4. Visual Updates
function updateSteeringWheel(angle) {
    document.getElementById("steering-wheel").style.transform = `rotate(${angle}deg)`;
}

function updatePedals(accel, brake) {
    document.getElementById("accelerator-fill").style.height = `${Math.abs(accel) * 100}%`;
    document.getElementById("brake-fill").style.height = `${Math.abs(brake) * 100}%`;
}

// 5. Telemetry Handling
function updateTelemetry() {
    fetch("/get_telemetry/")
        .then(response => response.json())
        .then(data => {
            document.getElementById("status-temperature").textContent = `${data.temperature} °C`;
            document.getElementById("status-battery").textContent = `${data.battery}%`;
            document.getElementById("status-latency").textContent = `${data.latency} ms`;
            document.getElementById("status-fps").textContent = data.fps;
            document.getElementById("status-connection").textContent = data.connection;
            document.getElementById("status-signal").textContent = `${data.signal}%`;
            document.getElementById("status-cpu").textContent = `${data.cpu_load}%`;
        })
        .catch(error => {
            console.error("Error fetching telemetry data:", error);
            clearInterval(telemetryInterval);  // Stop polling if the server is down
        });

    detectGamepad();
}

// 6. Initialize Periodic Updates
setInterval(updateTelemetry, 15000);
updateTelemetry();

// 7. DOM Content Loaded Handling
document.addEventListener("DOMContentLoaded", function() {
    document.querySelectorAll(".key").forEach(key => {
        key.addEventListener("click", () => sendCommand(key.getAttribute("data-command")));
    });
});
