//Functions
// Function to send commands to the backend
function sendCommand(action, speed) {
    fetch('/control', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ action: action, speed: speed }),
    })
        .then(response => response.json())
        .then(data => {
            console.log('Command sent successfully:', data);
        })
        .catch((error) => {
            console.error('Error sending command:', error);
        });
}

// Poll for gamepad input
let gamepadIndex = null;
let lastSteeringCommand = null; // Track the last sent steering command


function pollGamepad() {
    const gamepads = navigator.getGamepads();
    if (gamepadIndex !== null && gamepads[gamepadIndex]) {
        const gamepad = gamepads[gamepadIndex];

        // Example: Detect wheel input (axes[0] for steering)
        const steering = gamepad.axes[0]; // Range: -1 (left) to 1 (right)

        // Define thresholds
        const leftThreshold = -0.2;
        const rightThreshold = 0.2;

        // Map steering to commands
        let currentCommand;
        if (steering < leftThreshold) {
            currentCommand = 'left';
        } else if (steering > rightThreshold) {
            currentCommand = 'right';
        } else {
            currentCommand = 'center';
        }

        // Send command only if it changes
        if (currentCommand !== lastSteeringCommand) {
            sendCommand(currentCommand, 0); // Send updated command
            lastSteeringCommand = currentCommand; // Update last command
            console.log(`Steering command: ${currentCommand}`);
        }
    }
    requestAnimationFrame(pollGamepad);
}


//Events
// Listen for keyboard events
document.addEventListener('keydown', (event) => {
    switch (event.key) {
        case 'w': // Forward
            sendCommand('forward', 1.0);
            break;
        case 's': // Backward
            sendCommand('backward', 1.0);
            break;
        case 'a': // Steer Left
            sendCommand('left', 0);
            break;
        case 'd': // Steer Right
            sendCommand('right', 0);
            break;
        case 'x': // Spacebar - Stop
            sendCommand('stop', 0);
            break;
        case 'c': // Center
            sendCommand('center', 0);
            break;
    }
});

// Detect when a gamepad is connected
window.addEventListener("gamepadconnected", (event) => {
    console.log("Gamepad connected:", event.gamepad);
    gamepadIndex = event.gamepad.index;
    document.getElementById("gamepad-status").innerText = `Gamepad connected: ${event.gamepad.id}`;
    pollGamepad(); // Start polling the gamepad
});

// Detect when a gamepad is disconnected
window.addEventListener("gamepaddisconnected", (event) => {
    console.log("Gamepad disconnected:", event.gamepad);
    gamepadIndex = null;
    document.getElementById("gamepad-status").innerText = "Gamepad disconnected.";
});


