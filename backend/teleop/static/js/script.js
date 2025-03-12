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

// Add this at the top level of your script
let currentController = null; // Store the current AbortController
let lastCommandSent = Date.now();
const MIN_COMMAND_INTERVAL = 50; // 50ms minimum between commands

// Replace the single controller with a queue of controllers
let pendingControllers = [];

// Command queue and processing
let commandQueue = [];
let isProcessing = false;

let selectedAcceleration = 0.25; // Default to 25%
let selectedSteeringAngle = 15; // Default to 15 degrees

function enqueueCommand(command, value = 1.0, isPriority = false) {
    if (isPriority) {
        // If it's a priority command, clear the queue and process immediately
        console.log(`Priority command received: ${command} with value ${value}`);
        commandQueue = [{ command, value }];
        processCommandQueue();
    } else {
        // Add to the queue
        commandQueue.push({ command, value });
        if (!isProcessing) {
            processCommandQueue();
        }
    }
}

function processCommandQueue() {
    if (commandQueue.length === 0) {
        isProcessing = false;
        return;
    }

    isProcessing = true;
    const { command, value } = commandQueue.shift();

    fetch(`/send_controller_command/?command=${command}&value=${value}`)
        .then(response => response.json())
        .then(data => {
            console.log(`Controller Command sent: ${data.command}, Value: ${data.value}`);
            addCommandToHistory(`Controller: ${data.command}, Value: ${value}`);
        })
        .catch(error => {
            console.error("Error sending controller command:", error);
        })
        .finally(() => {
            // Process the next command in the queue
            processCommandQueue();
        });
}

// Use enqueueCommand instead of sendControllerCommand
function sendControllerCommand(command, value = 1.0) {
    // Determine if the command is a priority
    const isPriority = (command === "brake" || (command === "accelerator" && value === 0));
    enqueueCommand(command, value, isPriority);
}

function addCommandToHistory(commandText) {
    const historyList = document.getElementById("command-history");
    if (!historyList) return; // Guard clause to prevent errors
    
    const newCommand = document.createElement("li");
    newCommand.textContent = commandText;
    historyList.appendChild(newCommand);
    
    // Keep only the last 50 commands
    while (historyList.children.length > 50) {
        historyList.removeChild(historyList.firstChild);
    }
    
    // Auto-scroll to the latest command
    const container = document.getElementById("command-history-box");
    if (container) {
        container.scrollTop = container.scrollHeight;
    }
}

// 2. Keyboard Controls
function updateKeyDisplay(key, isPressed) {
    const keyElement = document.getElementById(`key${key}`);
    if (keyElement) {
        keyElement.classList.toggle('active', isPressed);
    }
}

// Function to set the selected acceleration percentage
function setAccelerationPercentage(percentage) {
    selectedAcceleration = percentage;
    document.querySelectorAll('.acceleration-button').forEach(button => {
        button.classList.toggle('active', button.dataset.value == percentage);
    });
}

// Function to set the selected steering angle
function setSteeringAngle(angle) {
    selectedSteeringAngle = angle;
    document.querySelectorAll('.steering-button').forEach(button => {
        button.classList.toggle('active', button.dataset.value == angle);
    });
}

document.addEventListener('keydown', (e) => {
    switch (e.key.toLowerCase()) {
        case 'w': sendCommand('forward', selectedAcceleration); updateKeyDisplay('W', true); break;
        case 's': sendCommand('backward', selectedAcceleration); updateKeyDisplay('S', true); break;
        case 'a': sendCommand('left', selectedSteeringAngle); updateKeyDisplay('A', true); break;
        case 'd': sendCommand('right', selectedSteeringAngle); updateKeyDisplay('D', true); break;
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

document.addEventListener('DOMContentLoaded', function() {
    // Set initial acceleration to 25%
    setAccelerationPercentage(0.25);
    
    // Set initial steering angle to 15 degrees
    setSteeringAngle(15);
    
    // Add event listeners for the acceleration buttons
    document.querySelectorAll('.acceleration-button').forEach(button => {
        button.addEventListener('click', () => {
            setAccelerationPercentage(parseFloat(button.dataset.value));
        });
    });
    
    // Add event listeners for the steering angle buttons
    document.querySelectorAll('.steering-button').forEach(button => {
        button.addEventListener('click', () => {
            setSteeringAngle(parseFloat(button.dataset.value));
        });
    });

    // Modified event listener to exclude acceleration and steering buttons
    document.querySelectorAll(".key:not(.acceleration-button):not(.steering-button)").forEach(key => {
        key.addEventListener("click", () => {
            const command = key.getAttribute("data-command");
            if (command === "forward" || command === "backward") {
                sendCommand(command, selectedAcceleration);
            } else if (command === "left" || command === "right") {
                sendCommand(command, selectedSteeringAngle);
            } else {
                sendCommand(command);
            }
        });
    });

    // Add event listeners for resize buttons
    document.querySelectorAll('.resize-btn').forEach(button => {
        button.addEventListener('click', (e) => {
            e.preventDefault(); // Prevent any default button behavior
            const action = button.dataset.action;
            const target = button.dataset.target;
            handleVideoResize(action, target);
        });
    });
});

// Update these constants for better size control
const MIN_FEED_WIDTH = 300;  // Minimum width in pixels
const MAX_FEED_WIDTH = 1200; // Maximum width in pixels
const RESIZE_STEP = 50;      // Pixels to resize per click

// Updated resize function
function handleVideoResize(action, target) {
    const feedContainer = document.querySelector(`.video-feed.${target} .feed-container`);
    if (!feedContainer) return;

    // Get the current width, falling back to default values if not set
    const currentWidth = feedContainer.offsetWidth || 
        (target === 'primary' ? 600 : 450);
    
    let newWidth = currentWidth;
    if (action === 'increase') {
        newWidth = Math.min(currentWidth + RESIZE_STEP, MAX_FEED_WIDTH);
    } else if (action === 'decrease') {
        newWidth = Math.max(currentWidth - RESIZE_STEP, MIN_FEED_WIDTH);
    }
    
    // Apply the new width to the feed container
    feedContainer.style.width = `${newWidth}px`;
    
    // Force a reflow to ensure the change takes effect
    feedContainer.offsetHeight;
    
    // Log the resize operation
    console.log(`Resizing ${target} feed: ${currentWidth}px -> ${newWidth}px`);
    
    // Ensure parent containers adapt to the new size
    adjustContainerWidths();
}

// Add this new function to make sure containers adapt to feed sizes
function adjustContainerWidths() {
    // Get all feed containers
    const feedContainers = document.querySelectorAll('.feed-container');
    let maxWidth = 0;
    
    // Find the widest feed container
    feedContainers.forEach(container => {
        const width = container.offsetWidth;
        if (width > maxWidth) {
            maxWidth = width;
        }
    });
    
    // Add some padding
    maxWidth += 40;
    
    // Get the video column and ensure it's at least as wide as the widest feed
    const videoColumn = document.querySelector('.video-column');
    if (videoColumn) {
        videoColumn.style.minWidth = `${maxWidth}px`;
    }
}

// Video resize logic
document.addEventListener('DOMContentLoaded', function() {
    document.querySelectorAll('.resize-btn').forEach(button => {
        button.addEventListener('click', function() {
            const action = this.getAttribute('data-action');
            const target = this.getAttribute('data-target');
            resizeVideo(action, target);
        });
    });
});

const videoSizes = {
    primary: 600,
    secondary: 450
};

const MIN_VIDEO_WIDTH = 200;
const MAX_VIDEO_WIDTH = 1200;
const VIDEO_RESIZE_STEP = 50;

function resizeVideo(action, target) {
    const container = document.querySelector(`.video-feed.${target} .feed-container`);
    if (!container) return;

    let currentWidth = container.offsetWidth;

    if (action === 'increase' && currentWidth < MAX_FEED_WIDTH) {
        currentWidth += VIDEO_RESIZE_STEP;
    } else if (action === 'decrease' && currentWidth > MIN_FEED_WIDTH) {
        currentWidth -= VIDEO_RESIZE_STEP;
    }

    currentWidth = Math.min(Math.max(currentWidth, MIN_FEED_WIDTH), MAX_FEED_WIDTH);
    container.style.width = `${currentWidth}px`;
}

// 3. Gamepad Handling
let controllerConnected = false;
let drivingMode = "forward";
const deadzone = 0.1;

// Ensure previous values are initialized
let previousSteering = 0;
let previousAccelerator = 0;
let previousBrake = 0;

let lastCommandTime = 0; // Track the last time a command was sent
const commandInterval = 100; // Minimum interval between commands in milliseconds

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

    let direction = drivingMode;

    let steeringAngle = Math.round(((steering + 1) / 2) * 180);
    let accelValue = accelerator < 0 ? Math.abs(accelerator) : 0.0;
    let brakeValue = brake < 0 ? 0.0 : Math.abs((brake + 1) / 2);

    // Define deadzones
    const accelDeadzone = 0.1;
    const brakeDeadzone = 0.1;

    // Apply deadzones
    if (accelValue < accelDeadzone) accelValue = 0.0;
    if (brakeValue < brakeDeadzone) brakeValue = 0.0;

    const currentTime = Date.now();
    if (currentTime - lastCommandTime > commandInterval) {
        // Only send commands if enough time has passed
        if (previousSteering !== steeringAngle) {
            sendControllerCommand("steering", steeringAngle);
            previousSteering = steeringAngle;
        }

        // Only send accelerator command if the value has changed
        if (Math.abs(previousAccelerator - accelValue) > 0.01) {
            if (direction === "forward") {
                sendControllerCommand("accelerator", accelValue);
            } else if (direction === "reverse") {
                // Map the reverse value from 0 to 1 to -0.1 to -1
                const reverseValue = -((1 - accelValue) * 0.9 + 0.1);
                sendControllerCommand("accelerator", reverseValue);
            }
            previousAccelerator = accelValue;
        }

        // Only send brake command if the value has changed
        if (Math.abs(previousBrake - brakeValue) > 0.01) {
            sendControllerCommand("brake", brakeValue);
            previousBrake = brakeValue;
        }

        lastCommandTime = currentTime; // Update the last command time
    }

    requestAnimationFrame(handleGamepadInput);
}

// Remove redundant polling
// Polling + handleGamepadInput
// let gamepadPolling = setInterval(() => {
//     const gamepads = navigator.getGamepads();
//     if (gamepads[0]) {
//         try {
//             processGamepadInput(gamepads[0]);
//         } catch (error) {
//             console.error("Gamepad polling stopped due to server error:", error);
//             clearInterval(gamepadPolling);
//         }
//     }
//     handleGamepadInput();
// }, 100);

function pollGamepad() {
    const gamepads = navigator.getGamepads();
    if (gamepads && gamepads[0]) {
        processGamepadInput(gamepads[0]);
    }
    setTimeout(pollGamepad, 100);
}

// Start polling
pollGamepad();

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
    // No need to cancel handleGamepadInput as it's not an animation frame request ID
    // If you need to perform cleanup, do it here
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
            alert("Failed to fetch telemetry data. Please check your connection.");
            // Consider retry logic instead of stopping completely
        })
        .finally(() => {
            // Schedule the next update only after the current one completes
            setTimeout(updateTelemetry, 15000);
        });

    detectGamepad();
}

// 6. Telemetry Polling
// Remove the setInterval call
// setInterval(updateTelemetry, 15000);
updateTelemetry();
