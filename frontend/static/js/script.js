// Functions
function sendCommand(action, value) {
    fetch('/control', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({ action: action, value: value }),
    })
        .then(response => response.json())
        .then(data => {
            console.log('Command sent successfully:', data);
        })
        .catch((error) => {
            console.error('Error sending command:', error);
        });
}

// Utility function to map a value from one range to another
function mapRange(value, inMin, inMax, outMin, outMax) {
    return ((value - inMin) * (outMax - outMin)) / (inMax - inMin) + outMin;
}

let smoothedAngle = 90; // Start at center
let lastSteeringAngle = 90; // Track the last sent steering angle
let lastSteeringCommand = 'center';
let lastMotorCommand = 'stop';
let deviceType = null; // Device type: "racewheel" or "controller"
let currentSpeed = 0.2; // Default speed
const speedLevels = [0.2, 0.4, 0.6, 0.8, 1.0]; // Speed levels for shifters

function smoothAngle(targetAngle, currentAngle, smoothingFactor) {
    return currentAngle + (targetAngle - currentAngle) * smoothingFactor;
}

function detectDevice(gamepad) {
    if (gamepad.id.toLowerCase().includes("logitech") || gamepad.id.toLowerCase().includes("racing")) {
        deviceType = "racewheel";
    } else if (gamepad.id.toLowerCase().includes("controller") || gamepad.id.toLowerCase().includes("gamepad") || gamepad.id.toLowerCase().includes("buffalo")) {
        deviceType = "controller";
    } else {
        deviceType = "unknown";
    }
    console.log(`Detected device: ${deviceType}`);
}

function pollGamepad() {
    const gamepads = navigator.getGamepads();
    if (gamepadIndex !== null && gamepads[gamepadIndex]) {
        const gamepad = gamepads[gamepadIndex];

        if (deviceType === "racewheel") {
            // Racing Wheel Logic

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
                //const acceleratorNormalized = (accelerator + 1) / 2; // Normalize -1 to 1 range
                //console.log(`Normalized Accelerator: ${acceleratorNormalized}`);
                const brake = gamepad.axes[5]; // Brake axis
                console.log(`Brake: ${brake}`);
                const reverseButton = gamepad.buttons[17]; // Button 17 for reverse


                if (reverseButton && reverseButton.pressed) {
                    motorAction = { action: 'backward', speed: 0.9 };
                    console.log(`Reverse at speed: 0.9`);
                } else if (accelerator < 0.5) {
                    motorAction = { action: 'forward', speed: currentSpeed };
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
        else if (deviceType === "controller") {
            // Controller Logic
    
            // Servo Control with Axis 0
            const steering = gamepad.axes[0]; // Axis 0 for steering
            const targetServoAngle = Math.round(mapRange(steering, -1, 1, 10, 170)); // Map -1 to 1 to 0 to 180
    
            // Send the servo angle only if it changes
            if (targetServoAngle !== lastSteeringAngle) {
                sendCommand("servo", targetServoAngle); // Send mapped servo angle
                lastSteeringAngle = targetServoAngle; // Update last steering angle
                console.log(`Controller Steering Servo to: ${targetServoAngle}°`);
            }

            // Accelerator and Brake Logic
            const accelerator = gamepad.axes[5]; // Accelerator axis

            if (accelerator > 0.2) {
                motorAction = { action: 'backward', speed: 0.9 };
                console.log(`Reverse at speed: 0.9`);
            } else if (accelerator < -0.2) {
                motorAction = { action: 'forward', speed: 1.0 };
                console.log(`Forward at speed: ${currentSpeed}`);
            } else {
                motorAction = { action: 'stop', speed: 0 }; // Stop when braking
                console.log('Braking: Motor stopped');
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
        } else {
            console.warn("Unknown device type. Input ignored.");
        }
    } 

    requestAnimationFrame(pollGamepad);
}



// Events
let gamepadIndex = null;

window.addEventListener("gamepadconnected", (event) => {
    console.log("Gamepad connected:", event.gamepad);
    gamepadIndex = event.gamepad.index;
    detectDevice(event.gamepad); // Detect the connected device
    pollGamepad();
});

window.addEventListener("gamepaddisconnected", (event) => {
    console.log("Gamepad disconnected:", event.gamepad);
    gamepadIndex = null;
    document.getElementById("gamepad-status").innerText = "Gamepad disconnected.";
});

// Keyboard Controls
document.addEventListener("keydown", (event) => {
    switch (event.key.toLowerCase()) {
        case "w": // Forward
            sendCommand("forward", 1.0);
            break;
        case "s": // Backward
            sendCommand("backward", 0.9);
            break;
        case "a": // Steer Left
            sendCommand("left", 0);
            break;
        case "d": // Steer Right
            sendCommand("right", 0);
            break;
        case "x": // Stop
            sendCommand("stop", 0);
            break;
        case "c": // Center
            sendCommand("center", 0);
            break;
    }
});
