// Functions
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
let lastMotorCommand = { action: null, speed: null }; // Track the last motor command
let currentSpeed = 0.2; // Default speed
const speedLevels = [0.2, 0.4, 0.6, 0.8, 1.0]; // Speed levels for shifters

function pollGamepad() {
    const gamepads = navigator.getGamepads();
    if (gamepadIndex !== null && gamepads[gamepadIndex]) {
        const gamepad = gamepads[gamepadIndex];

        // Steering Logic
        const steering = gamepad.axes[0]; // Range: -1 (left) to 1 (right)
        const leftThreshold = -0.2;
        const rightThreshold = 0.2;
        let currentSteeringCommand;

        if (steering < leftThreshold) {
            currentSteeringCommand = 'left';
        } else if (steering > rightThreshold) {
            currentSteeringCommand = 'right';
        } else {
            currentSteeringCommand = 'center';
        }

        if (currentSteeringCommand !== lastSteeringCommand) {
            sendCommand(currentSteeringCommand, 0);
            lastSteeringCommand = currentSteeringCommand;
            console.log(`Steering command: ${currentSteeringCommand}`);
        }

        // Speed Control with Shifters
        let motorAction = null;

        // Check if button 12 (Park) is pressed
        if (gamepad.buttons[12] && gamepad.buttons[12].pressed) {
            motorAction = { action: 'stop', speed: 0 }; // Park mode
            console.log('Park mode activated');
        } else {
            // Adjust speed based on other shifter buttons (Buttons 13–16)
            for (let i = 13; i <= 16; i++) {
                if (gamepad.buttons[i] && gamepad.buttons[i].pressed) {
                    currentSpeed = speedLevels[i - 12];
                    console.log(`Speed set to: ${currentSpeed}`);
                    break;
                }
            }

            // Accelerator and Brake Logic
            const accelerator = gamepad.axes[2]; // Accelerator axis
            const acceleratorNormalized = (accelerator + 1) / 2; // Normalize -1 to 1 range
            //console.log(`Normalized Accelerator: ${acceleratorNormalized}`);
            const brake = gamepad.axes[5]; // Brake axis
            console.log(`Brake: ${brake}`);
            const reverseButton = gamepad.buttons[17]; // Button 17 for reverse

            // if (acceleratorNormalized > 0.1) {
            //     if (reverseButton && reverseButton.pressed) {
            //         motorAction = { action: 'backward', speed: currentSpeed * acceleratorNormalized };
            //         console.log(`Reverse at speed: ${currentSpeed * acceleratorNormalized}`);
            //     } else{
            //         motorAction = { action: 'forward', speed: currentSpeed * acceleratorNormalized };
            //         console.log(`Forward at speed: ${currentSpeed * acceleratorNormalized}`);
            //     }
            // } else if (brake < 0.5) {
            //     motorAction = { action: 'stop', speed: 0 }; // Stop when braking
            //     console.log('Braking: Motor stopped');
            // }


            if (reverseButton && reverseButton.pressed) {
                motorAction = { action: 'backward', speed: 0.9};
                console.log(`Reverse at speed: 0.9`);
            } else if (accelerator < 0.5){
                motorAction = { action: 'forward', speed: currentSpeed};
                console.log(`Forward at speed: ${currentSpeed}`);
            } else if (brake < 0.5) {
                motorAction = { action: 'stop', speed: 0 }; // Stop when braking
                console.log('Braking: Motor stopped');
            }
        }

        // Send motor command only if it changes
        if (
            motorAction &&
            (motorAction.action !== lastMotorCommand.action ||
                motorAction.speed !== lastMotorCommand.speed)
        ) {
            sendCommand(motorAction.action, motorAction.speed);
            lastMotorCommand = motorAction; // Update last motor command
        }
    }

    requestAnimationFrame(pollGamepad);
}

// Events
window.addEventListener('gamepadconnected', (event) => {
    console.log('Gamepad connected:', event.gamepad);
    gamepadIndex = event.gamepad.index;
    document.getElementById('gamepad-status').innerText = `Gamepad connected: ${event.gamepad.id}`;
    pollGamepad();
});

window.addEventListener('gamepaddisconnected', (event) => {
    console.log('Gamepad disconnected:', event.gamepad);
    gamepadIndex = null;
    document.getElementById('gamepad-status').innerText = 'Gamepad disconnected.';
});

// Listen for keyboard events
document.addEventListener('keydown', (event) => {
    switch (event.key.toLowerCase()) {
        case 'w': // Forward
            sendCommand('forward', 1.0);
            break;
        case 's': // Backward
            sendCommand('backward', 0.9);
            break;
        case 'a': // Steer Left
            sendCommand('left', 0);
            break;
        case 'd': // Steer Right
            sendCommand('right', 0);
            break;
        case 'x': // Stop
            sendCommand('stop', 0);
            break;
        case 'c': // Center
            sendCommand('center', 0);
            break;
    }
});