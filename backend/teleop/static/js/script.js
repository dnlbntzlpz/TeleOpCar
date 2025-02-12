// Function to send ROS 2 commands using HTTP requests
function sendCommand(action, value = 1.0) {
    fetch(`/send_command/?command=${action}&value=${value}`)
        .then(response => response.json())
        .then(data => console.log(`Command sent: ${data.command}, Status: ${data.status}`))
        .catch(error => console.error("Error sending command:", error));
}

// Example: Hooking buttons to the sendCommand function
document.addEventListener("DOMContentLoaded", function() {
    // Select all keys that have a "data-command" attribute
    document.querySelectorAll(".key").forEach(key => {
        key.addEventListener("click", () => {
            const command = key.getAttribute("data-command");
            sendCommand(command);
        });
    });
});

// Gamepad Support
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
    if (Math.abs(gamepad.axes[0]) < deadzone) steering = 0;

    // Change drive mode with buttons
    if (gamepad.buttons[12].pressed) drivingMode = "forward";
    if (gamepad.buttons[13].pressed) drivingMode = "reverse";

    // Update visuals
    updateSteeringWheel(steering);
    updatePedals(accelerator, brake);
}

let previousSteering = null;
let previousAccelerator = null;
let previousBrake = null;

// Function to send commands to the Raspberry Pi for controller input
function sendControllerCommand(command, value = 1.0) {
    fetch(`/send_controller_command/?command=${command}&value=${value}`)
        .then(response => response.json())
        .then(data => console.log(`Controller Command sent: ${data.command}, Value: ${data.value}`))
        .catch(error => console.error("Error sending controller command:", error));
}

// Visual Updates
function updateSteeringWheel(angle) {
    document.getElementById("steering-wheel").style.transform = `rotate(${angle}deg)`;
}

function updatePedals(accel, brake) {
    document.getElementById("accelerator-fill").style.height = `${Math.abs(accel) * 100}%`;
    document.getElementById("brake-fill").style.height = `${Math.abs(brake) * 100}%`;
}

// Function to handle gamepad input
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

setInterval(() => {
    const gamepads = navigator.getGamepads();
    if (gamepads[0]) {
        processGamepadInput(gamepads[0]);
    }
}, 100);

// Gamepad Events
window.addEventListener("gamepadconnected", () => detectGamepad());
window.addEventListener("gamepaddisconnected", () => detectGamepad());

detectGamepad();

// Start listening for gamepad inputs
window.addEventListener("gamepadconnected", (event) => {
    console.log("Gamepad connected:", event.gamepad.id);
    previousSteering = null;
    previousAccelerator = null;
    previousBrake = null;
    requestAnimationFrame(handleGamepadInput);
});

window.addEventListener("gamepaddisconnected", () => {
    console.log("Gamepad disconnected.");
});

// Continuously check for gamepad input
window.addEventListener("gamepadconnected", (e) => {
    console.log("Gamepad connected:", e.gamepad);
    setInterval(detectGamepad, 100);
});

function detectGamepad() {
    const gamepads = navigator.getGamepads();
    let gamepadConnected = false;

    for (const gamepad of gamepads) {
        if (gamepad) {
            gamepadConnected = true;
            break;
        }
    }

    // Update Gamepad UI Status
    const gamepadStatus = document.getElementById("status-controller");
    if (gamepadConnected) {
        gamepadStatus.textContent = "Connected";
        gamepadStatus.style.color = "green";
    } else {
        gamepadStatus.textContent = "No Gamepad Detected";
        gamepadStatus.style.color = "red";
    }
}

// Keyboard Controls
document.addEventListener('keydown', (e) => {
    switch (e.key.toLowerCase()) {
        case 'w':
            sendCommand('forward');
            updateKeyDisplay('W', true);
            break;
        case 's':
            sendCommand('backward');
            updateKeyDisplay('S', true);
            break;
        case 'a':
            sendCommand('left');
            updateKeyDisplay('A', true);
            break;
        case 'd':
            sendCommand('right');
            updateKeyDisplay('D', true);
            break;
        case 'x':
            sendCommand('stop');
            break;
        case 'c':
            sendCommand('center');
            break;
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

function updateKeyDisplay(key, isPressed) {
    const keyElement = document.getElementById(`key${key}`);
    if (keyElement) {
        if (isPressed) {
            keyElement.classList.add('active');
        } else {
            keyElement.classList.remove('active');
        }
    }
}

function sendCommand(command) {
    fetch(`/send_command?command=${command}`)
        .then(response => response.json())
        .then(data => console.log("Command sent:", data))
        .catch(error => console.error("Error sending command:", error));
}

// Function to update telemetry status
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

    detectGamepad(); // Check if a gamepad is connected
}

// Check for gamepad connection when a controller is plugged in/out
window.addEventListener("gamepadconnected", () => detectGamepad());
window.addEventListener("gamepaddisconnected", () => detectGamepad());

// Update telemetry and gamepad status every 5 seconds
setInterval(updateTelemetry, 5000);
updateTelemetry(); // Initial call
