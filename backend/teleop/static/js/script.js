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
        case 'w': case 's': case 'a': case 'd': updateKeyDisplay(e.key.toUpperCase(), false); break;
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

function processGamepadInput(gamepad) {
    if (!gamepad) return;

    let acceleratorRaw = gamepad.axes[2];   // -1 (pressed) to 1 (unpressed)
    let brakeRaw = gamepad.axes[5];         // -1 (pressed) to 1 (unpressed)

    let accelerator = (1 - acceleratorRaw) / 2; // 0 (unpressed) to 1 (fully pressed)
    let brake = (1 - brakeRaw) / 2;             // 0 (unpressed) to 1 (fully pressed)

    // Apply deadzone correction
    if (accelerator < deadzone) accelerator = 0;
    if (brake < deadzone) brake = 0;

    // Steering wheel mapping
    let steering = gamepad.axes[0] * 360; // Map from -90 to +90 degrees
    //if (Math.abs(gamepad.axes[0]) < deadzone) steering = 0;

    // Change drive mode with buttons
    if (gamepad.buttons[12].pressed) drivingMode = "forward";
    if (gamepad.buttons[13].pressed) drivingMode = "reverse";

    // Update visuals
    updateSteeringWheel(steering);
    updatePedals(accelerator, brake);
}

setInterval(() => {
    const gamepads = navigator.getGamepads();
    if (gamepads[0]) {
        processGamepadInput(gamepads[0]);
    }
}, 100);

function handleGamepadInput() {
    const gamepads = navigator.getGamepads();
    if (!gamepads) return;

    const gp = gamepads[0];  // Assuming only one gamepad is connected
    if (!gp) return;

    // Read steering wheel (axis 0), accelerator (axis 1), brake (axis 2)
    let steering = gp.axes[0];  // Steering wheel: -1 (left), 1 (right)
    let accelerator = gp.axes[2]; // Accelerator: -1 (pressed), 1 (released)
    let brake = gp.axes[5];  // Brake: 1 (unpressed), -1 (pressed)
    let carDirection = "forward"; // Default direction is forward

    if (gp.buttons[12] && gp.buttons[12].pressed && carDirection !== "forward") {
        carDirection = "forward";
        console.log("Button 12 pressed: Car direction set to FORWARD.");
    }
    if (gp.buttons[13] && gp.buttons[13].pressed && carDirection !== "reverse") {
        carDirection = "reverse";
        console.log("Button 13 pressed: Car direction set to REVERSE.");
    }

    // Map steering to the 0-180 range
    let steeringAngle = Math.round(((steering + 1) / 2) * 180); // Convert [-1,1] to [0,180]

    // Normalize accelerator values (already in range -1 to 1)
    let accelValue = accelerator < 0 ? Math.abs(accelerator) : 0.0; // Convert -1 (full press) to positive range

    // Normalize brake values (convert from [1, -1] to [0, 1])
    let brakeValue = brake > 0 ? 0.0 : Math.abs((brake + 1) / 2); // Normalize so 1 (unpressed) -> 0 and -1 (pressed) -> 1

    // Send steering command only if it changes
    if (previousSteering !== steeringAngle) {
        sendControllerCommand("steering", steeringAngle);
        previousSteering = steeringAngle;
    }

    // Send accelerator or brake command only if it changes
    if (accelValue > 0 && previousAccelerator !== accelValue && carDirection === "forward") {
        sendControllerCommand("accelerator", accelValue);
        previousAccelerator = accelValue;
    } else if(accelValue > 0 && previousAccelerator !== accelValue && carDirection === "reverse") {
        //sendControllerCommand("brake", 1.0);
        sendControllerCommand("accelerator", -(accelValue - 0.1));
        previousAccelerator = accelValue;
    } else if (brakeValue > 0 && previousBrake !== brakeValue) {
        sendControllerCommand("brake", brakeValue);
        previousBrake = brakeValue;
    }

    // Call the function again in the next animation frame
    requestAnimationFrame(handleGamepadInput);
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
        .catch(error => console.error("Error fetching telemetry data:", error));

    detectGamepad();
}

// 6. Initialize Periodic Updates
setInterval(updateTelemetry, 5000);
updateTelemetry();

// 7. DOM Content Loaded Handling
document.addEventListener("DOMContentLoaded", function() {
    document.querySelectorAll(".key").forEach(key => {
        key.addEventListener("click", () => sendCommand(key.getAttribute("data-command")));
    });
});
