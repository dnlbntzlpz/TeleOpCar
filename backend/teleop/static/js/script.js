// Function to send ROS 2 commands using HTTP requests
function sendCommand(action, value = 1.0) {
    fetch(`/send_command/?command=${action}&value=${value}`)
        .then(response => response.json())
        .then(data => console.log(`Command sent: ${data.command}, Status: ${data.status}`))
        .catch(error => console.error("Error sending command:", error));
}

// Wait for the DOM to load and then add event listeners
document.addEventListener("DOMContentLoaded", function () {
    const buttonIds = ["forward", "backward", "left", "center", "right", "stop"];
    buttonIds.forEach((id) => {
        const button = document.getElementById(id);
        if (button) {
            button.addEventListener("click", () => sendCommand(id));
        } else {
            console.error(`Button with ID '${id}' not found.`);
        }
    });
});

// Keyboard control stuff
document.addEventListener("keydown", function(event) {
    if (event.key === "w") sendCommand("forward");
    if (event.key === "s") sendCommand("backward");
    if (event.key === "a") sendCommand("left");
    if (event.key === "d") sendCommand("right");
    if (event.key === "c") sendCommand("center");
    if (event.key === "x") sendCommand("stop");
});

//Controller stuff
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

// Function to handle gamepad input
function handleGamepadInput() {
    const gamepads = navigator.getGamepads();
    if (!gamepads) return;

    const gp = gamepads[0];  // Assuming only one gamepad is connected
    if (!gp) return;

    // Read steering wheel (axis 0), accelerator (axis 1), brake (axis 2)
    let steering = gp.axes[0];  // Steering wheel: -1 (left), 1 (right)
    let accelerator = gp.axes[1]; // Accelerator: -1 (pressed), 1 (released)
    let brake = gp.axes[2];  // Brake: -1 (pressed), 1 (released)

    // Map steering to the 0-180 range
    let steeringAngle = Math.round(((steering + 1) / 2) * 180); // Convert [-1,1] to [0,180]

    // Normalize accelerator and brake values
    let accelValue = accelerator < 0 ? Math.abs(accelerator) : 0.0; // Convert -1 (full press) to positive range
    let brakeValue = brake < 0 ? Math.abs(brake) : 0.0;  // Convert -1 (full press) to positive range

    // Send steering command only if it changes
    if (previousSteering !== steeringAngle) {
        sendControllerCommand("steering", steeringAngle);
        previousSteering = steeringAngle;
    }

    // Send accelerator or brake command only if it changes
    if (accelValue > 0 && previousAccelerator !== accelValue) {
        sendControllerCommand("accelerator", accelValue);
        previousAccelerator = accelValue;
    } else if (brakeValue > 0 && previousBrake !== brakeValue) {
        sendControllerCommand("brake", brakeValue);
        previousBrake = brakeValue;
    }

    // Call the function again in the next animation frame
    requestAnimationFrame(handleGamepadInput);
}

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
