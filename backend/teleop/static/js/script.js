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

// Make sure this is at the top of your script.js file, outside any functions
let isFullscreen = false;

// Then add a simple debug function to check its state
function checkFullscreenState() {
    console.log("Current fullscreen state:", isFullscreen);
}

function enterFullscreen() {
    console.log("Entering fullscreen mode");
    isFullscreen = true;
    
    // Get all the elements we need to modify
    const body = document.body;
    const mainContent = document.querySelector('.main-content');
    const container = document.querySelector('.container');
    const videoColumn = document.querySelector('.video-column');
    const videoFeeds = document.querySelector('.video-feeds');
    const primaryFeed = document.querySelector('.video-feed.primary');
    const primaryFeedContainer = document.querySelector('.video-feed.primary .feed-container');
    const primaryFeedImg = document.querySelector('.video-feed.primary .feed-container img');
    const primaryFeedOverlay = document.querySelector('.video-feed.primary .feed-overlay');
    const feedLabel = primaryFeedOverlay ? primaryFeedOverlay.querySelector('.feed-label') : null;
    const feedControls = primaryFeedOverlay ? primaryFeedOverlay.querySelector('.feed-controls') : null;
    const secondaryFeed = document.querySelector('.video-feed.secondary');
    const controlsColumn = document.querySelector('.controls-column');
    const statusHistoryColumn = document.querySelector('.status-history-column');
    const fullscreenBtn = document.getElementById('frontCamFullscreen');
    
    // Hide resize buttons in fullscreen mode
    const resizeButtons = primaryFeedOverlay ? primaryFeedOverlay.querySelectorAll('.resize-btn') : null;
    if (resizeButtons) {
        resizeButtons.forEach(button => {
            button.style.display = 'none';
        });
    }
    
    // Get the original background color
    const originalBgColor = getComputedStyle(body).backgroundColor || '#1e272e';
    
    if (!mainContent) {
        console.error("Main content element not found");
        return;
    }

    // Prevent scrolling on body
    body.style.overflow = 'hidden';
    
    // Add fullscreen class to main content
    mainContent.classList.add('fullscreen-mode');
    
    // Change main content to take up the full viewport
    mainContent.style.position = 'fixed';
    mainContent.style.top = '0';
    mainContent.style.left = '0';
    mainContent.style.width = '100vw';
    mainContent.style.height = '100vh';
    mainContent.style.padding = '0';
    mainContent.style.margin = '0';
    mainContent.style.backgroundColor = originalBgColor;
    mainContent.style.display = 'flex';
    mainContent.style.justifyContent = 'center'; // Center content
    mainContent.style.alignItems = 'center';
    mainContent.style.zIndex = '9999';
    
    // Reset all positioning and layout styles
    if (container) {
        container.style.padding = "0";
        container.style.margin = "0";
        container.style.maxWidth = "100%";
        container.style.width = "100%";
        container.style.height = "100%";
        container.style.overflow = "hidden";
    }
    
    // Define side panel width and spacing
    const sideColumnWidth = 400;
    const sideColumnMargin = 10;
    
    // Make video column fill the entire viewport
    if (videoColumn) {
        videoColumn.style.position = "fixed";
        videoColumn.style.top = "0";
        videoColumn.style.left = "0";
        videoColumn.style.width = "100vw";
        videoColumn.style.height = "100vh";
        videoColumn.style.display = "flex";
        videoColumn.style.justifyContent = "center";
        videoColumn.style.alignItems = "center";
        videoColumn.style.padding = "0";
        videoColumn.style.margin = "0";
        videoColumn.style.zIndex = "1000";
    }
    
    // Make video feeds container fill the entire space
    if (videoFeeds) {
        videoFeeds.style.position = "relative";
        videoFeeds.style.width = "100%";
        videoFeeds.style.height = "100%";
        videoFeeds.style.display = "flex";
        videoFeeds.style.justifyContent = "center";
        videoFeeds.style.alignItems = "center";
        videoFeeds.style.padding = "0";
        videoFeeds.style.margin = "0";
    }
    
    // Make primary feed fill the entire viewport
    if (primaryFeed) {
        primaryFeed.style.position = "absolute";
        primaryFeed.style.top = "0";
        primaryFeed.style.left = "0";
        primaryFeed.style.width = "100%";
        primaryFeed.style.height = "100%";
        primaryFeed.style.display = "flex";
        primaryFeed.style.justifyContent = "center";
        primaryFeed.style.alignItems = "center";
        primaryFeed.style.margin = "0";
        primaryFeed.style.zIndex = "1000";
    }
    
    // Make primary feed container fill the entire viewport
    if (primaryFeedContainer && primaryFeedImg) {
        primaryFeedContainer.style.position = "absolute";
        primaryFeedContainer.style.top = "0";
        primaryFeedContainer.style.left = "0";
        primaryFeedContainer.style.width = "100%";
        primaryFeedContainer.style.height = "100%";
        primaryFeedContainer.style.maxHeight = "none";
        primaryFeedContainer.style.maxWidth = "none";
        primaryFeedContainer.style.borderRadius = "0";
        primaryFeedContainer.style.border = "none";
        primaryFeedContainer.style.margin = "0";
        primaryFeedContainer.style.overflow = "hidden";
        primaryFeedContainer.style.backgroundColor = originalBgColor;
        primaryFeedContainer.style.display = "flex";
        primaryFeedContainer.style.justifyContent = "center";
        primaryFeedContainer.style.alignItems = "center";
        
        // Set image to fill the container exactly
        primaryFeedImg.style.width = "100%";
        primaryFeedImg.style.height = "100%";
        primaryFeedImg.style.objectFit = "cover"; // Fill the entire container
        primaryFeedImg.style.position = "absolute";
        primaryFeedImg.style.display = "block";
        primaryFeedImg.style.margin = "0";
    }
    
    // Make sure the overlay stays aligned with the feed container
    if (primaryFeedOverlay) {
        primaryFeedOverlay.style.position = "absolute";
        primaryFeedOverlay.style.top = "0";
        primaryFeedOverlay.style.left = "0";
        primaryFeedOverlay.style.right = "0";
        primaryFeedOverlay.style.width = "100%";
        primaryFeedOverlay.style.boxSizing = "border-box";
        primaryFeedOverlay.style.zIndex = "1001";
        primaryFeedOverlay.style.padding = "1rem";
        primaryFeedOverlay.style.pointerEvents = "auto";
        primaryFeedOverlay.style.background = "linear-gradient(180deg, rgba(0,0,0,0.6) 0%, transparent 100%)";
        primaryFeedOverlay.style.display = "flex";
        primaryFeedOverlay.style.justifyContent = "space-between";
        primaryFeedOverlay.style.alignItems = "center";
    }
    
    // Ensure the feed label is visible
    if (feedLabel) {
        feedLabel.style.color = "var(--text-primary)";
        feedLabel.style.fontWeight = "600";
        feedLabel.style.fontSize = "1.2rem";
        feedLabel.style.textTransform = "uppercase";
        feedLabel.style.letterSpacing = "0.05em";
        feedLabel.style.textShadow = "0 2px 4px rgba(0,0,0,0.5)";
    }
    
    // Ensure the feed controls are properly aligned
    if (feedControls) {
        feedControls.style.display = "flex";
        feedControls.style.alignItems = "center";
        feedControls.style.gap = "0.5rem";
    }
    
    // Position secondary feed at the top center with increased size
    if (secondaryFeed) {
        secondaryFeed.style.position = "fixed";
        secondaryFeed.style.top = "20px";
        secondaryFeed.style.left = "50%";
        secondaryFeed.style.transform = "translateX(-50%)";
        secondaryFeed.style.width = "400px"; // Increased from 300px to 400px
        secondaryFeed.style.zIndex = "1002";
        
        // Make sure the feed container and image are properly sized
        const secondaryFeedContainer = secondaryFeed.querySelector('.feed-container');
        if (secondaryFeedContainer) {
            secondaryFeedContainer.style.width = "100%";
            secondaryFeedContainer.style.height = "auto";
            
            // Ensure the image fills the container
            const secondaryFeedImg = secondaryFeedContainer.querySelector('img');
            if (secondaryFeedImg) {
                secondaryFeedImg.style.width = "100%";
                secondaryFeedImg.style.height = "100%";
                secondaryFeedImg.style.objectFit = "cover";
            }
        }
    }
    
    // Position status history column on the right side instead of left
    if (statusHistoryColumn) {
        statusHistoryColumn.style.position = "fixed";
        statusHistoryColumn.style.right = `${sideColumnMargin}px`; // Change from left to right
        statusHistoryColumn.style.left = "auto"; // Remove left positioning
        statusHistoryColumn.style.top = "50%";
        statusHistoryColumn.style.transform = "translateY(-50%)";
        statusHistoryColumn.style.width = `${sideColumnWidth}px`;
        statusHistoryColumn.style.maxHeight = "95vh";
        statusHistoryColumn.style.overflowY = "auto";
        statusHistoryColumn.style.margin = "0";
        statusHistoryColumn.style.opacity = "0.75";
        statusHistoryColumn.style.zIndex = "1002";
        statusHistoryColumn.style.padding = "10px";
        statusHistoryColumn.style.boxSizing = "border-box";
        statusHistoryColumn.style.backdropFilter = "blur(5px)";
        
        // Add hover effect to make it fully opaque when hovered
        statusHistoryColumn.addEventListener('mouseenter', function() {
            this.style.opacity = "1";
        });
        statusHistoryColumn.addEventListener('mouseleave', function() {
            this.style.opacity = "0.75";
        });
        
        // Hide the command log and its parent panel
        const historyPanel = statusHistoryColumn.querySelector('.history-panel');
        if (historyPanel) {
            historyPanel.style.display = "none";
        }
        
        // In case the command log is directly accessible
        const commandLog = statusHistoryColumn.querySelector('.command-log');
        if (commandLog) {
            commandLog.style.display = "none";
        }
        
        // Specifically target the vehicle status panel to make it taller
        const vehicleStatusPanel = statusHistoryColumn.querySelector('.status-panel');
        if (vehicleStatusPanel) {
            vehicleStatusPanel.style.minHeight = "520px";
            vehicleStatusPanel.style.height = "auto";
        }
        
        // Ensure status grid items have proper width and don't get cut off
        const statusGrid = statusHistoryColumn.querySelector('.status-grid');
        if (statusGrid) {
            statusGrid.style.gridTemplateColumns = "1fr 1fr";
            statusGrid.style.width = "100%";
            statusGrid.style.boxSizing = "border-box";
            statusGrid.style.gap = "15px";
            statusGrid.style.padding = "5px";
            
            // Make sure each status item has enough space
            const statusItems = statusGrid.querySelectorAll('.status-item');
            statusItems.forEach(item => {
                item.style.width = "100%";
                item.style.boxSizing = "border-box";
                item.style.minWidth = "0";
                item.style.overflow = "hidden";
                item.style.display = "flex";
                item.style.alignItems = "center";
                item.style.padding = "12px";
                item.style.marginBottom = "10px";
                
                // Ensure status values don't get cut off
                const statusValue = item.querySelector('.status-value');
                if (statusValue) {
                    statusValue.style.whiteSpace = "nowrap";
                    statusValue.style.overflow = "hidden";
                    statusValue.style.textOverflow = "ellipsis";
                    statusValue.style.fontSize = "1.1rem";
                }
                
                // Ensure status icons have enough space
                const statusIcon = item.querySelector('.status-icon');
                if (statusIcon) {
                    statusIcon.style.marginRight = "10px";
                    statusIcon.style.fontSize = "1.5rem";
                }
            });
        }
    }
    
    // Position controls column on the right as an overlay
    if (controlsColumn) {
        // Change from right side vertical panel to bottom horizontal bar
        controlsColumn.style.position = "fixed";
        controlsColumn.style.left = "0";
        controlsColumn.style.right = "0";
        controlsColumn.style.bottom = "0";
        controlsColumn.style.top = "auto"; // Remove top positioning
        controlsColumn.style.transform = "none"; // Remove vertical centering
        controlsColumn.style.width = "100%"; // Full width
        controlsColumn.style.height = "auto"; // Height based on content
        controlsColumn.style.maxHeight = "25vh"; // Limit maximum height
        controlsColumn.style.overflowY = "auto";
        controlsColumn.style.margin = "0";
        controlsColumn.style.opacity = "0.75";
        controlsColumn.style.zIndex = "1002";
        controlsColumn.style.padding = "10px";
        controlsColumn.style.boxSizing = "border-box";
        controlsColumn.style.backdropFilter = "blur(5px)";
        controlsColumn.style.borderTop = "1px solid rgba(255, 255, 255, 0.1)"; // Add top border
        controlsColumn.style.display = "flex"; // Use flexbox for layout
        controlsColumn.style.justifyContent = "center"; // Center the controls horizontally
        controlsColumn.style.alignItems = "center"; // Center the controls vertically
        controlsColumn.style.flexDirection = "row"; // Ensure horizontal layout
        
        // Add hover effect to make it fully opaque when hovered
        controlsColumn.addEventListener('mouseenter', function() {
            this.style.opacity = "1";
        });
        controlsColumn.addEventListener('mouseleave', function() {
            this.style.opacity = "0.75";
        });
        
        // Get the control panels
        const wheelControls = controlsColumn.querySelector('.wheel-controls');
        const keyboardControls = controlsColumn.querySelector('.keyboard-controls');
        
        // Remove any existing styles that might interfere with layout
        if (wheelControls) wheelControls.style = "";
        if (keyboardControls) keyboardControls.style = "";
        
        // Arrange control panels side by side
        if (wheelControls && keyboardControls) {
            // Set up the wheel controls
            wheelControls.style.display = "inline-block";
            wheelControls.style.verticalAlign = "middle";
            wheelControls.style.width = "auto";
            wheelControls.style.margin = "0 15px";
            wheelControls.style.padding = "10px";
            wheelControls.style.flexShrink = "0"; // Prevent shrinking
            wheelControls.style.backgroundColor = "rgba(0, 0, 0, 0.3)";
            wheelControls.style.borderRadius = "8px";
            
            // Make the title smaller
            const wheelTitle = wheelControls.querySelector('h2');
            if (wheelTitle) {
                wheelTitle.style.fontSize = "1rem";
                wheelTitle.style.marginBottom = "10px";
                wheelTitle.style.textAlign = "center";
                wheelTitle.style.whiteSpace = "nowrap";
            }
            
            // Adjust wheel and pedals content
            const wheelContent = wheelControls.querySelector('.wheel-and-pedals-content');
            if (wheelContent) {
                wheelContent.style.margin = "0";
                wheelContent.style.display = "flex";
                wheelContent.style.flexDirection = "row";
                wheelContent.style.alignItems = "center";
                wheelContent.style.justifyContent = "space-between";
                wheelContent.style.width = "100%";
                wheelContent.style.gap = "15px";
            }
            
            // Make steering wheel smaller
            const steeringWheel = wheelControls.querySelector('.steering-wheel');
            if (steeringWheel) {
                steeringWheel.style.width = "80px";
                steeringWheel.style.height = "80px";
                steeringWheel.style.margin = "0";
                steeringWheel.style.flexShrink = "0";
            }
            
            // Make pedals smaller and more compact
            const pedals = wheelControls.querySelector('.pedals');
            if (pedals) {
                pedals.style.gap = "10px";
                pedals.style.flexShrink = "0";
                pedals.style.margin = "0 15px";
                
                // Make pedal boxes more compact
                const pedalBoxes = pedals.querySelectorAll('.pedal-box');
                if (pedalBoxes.length > 0) {
                    pedalBoxes.forEach(box => {
                        box.style.margin = "0 5px";
                        
                        // Make labels smaller
                        const label = box.querySelector('label');
                        if (label) {
                            label.style.fontSize = "0.8rem";
                            label.style.marginBottom = "3px";
                        }
                        
                        // Make pedals smaller
                        const pedal = box.querySelector('.pedal');
                        if (pedal) {
                            pedal.style.width = "30px";
                            pedal.style.height = "80px";
                        }
                    });
                }
            }
            
            // Adjust drive mode container to be in the same row
            const driveModeContainer = wheelControls.querySelector('.drive-mode-container');
            if (driveModeContainer) {
                driveModeContainer.style.margin = "0";
                driveModeContainer.style.flexShrink = "0";
                
                // Make drive mode labels smaller
                const driveModeLabels = driveModeContainer.querySelector('.drive-mode-labels');
                if (driveModeLabels) {
                    driveModeLabels.style.fontSize = "0.8rem";
                    driveModeLabels.style.marginBottom = "3px";
                    driveModeLabels.style.width = "100px";
                }
                
                // Make toggle smaller
                const driveModeToggle = driveModeContainer.querySelector('.drive-mode-toggle');
                if (driveModeToggle) {
                    driveModeToggle.style.width = "100px";
                    driveModeToggle.style.height = "30px";
                    
                    // Adjust the indicator position
                    const indicator = driveModeToggle.querySelector('.drive-mode-indicator');
                    if (indicator) {
                        indicator.style.width = "26px";
                        indicator.style.height = "26px";
                    }
                }
            }
            
            // Reorganize the wheel controls layout
            if (wheelTitle && wheelContent) {
                // Move the title to be more compact
                wheelTitle.style.fontSize = "1rem";
                wheelTitle.style.marginBottom = "5px";
                wheelTitle.style.textAlign = "center";
                wheelTitle.style.whiteSpace = "nowrap";
                
                // Make the overall container more compact
                wheelControls.style.padding = "10px";
                wheelControls.style.width = "auto";
                wheelControls.style.display = "flex";
                wheelControls.style.flexDirection = "column";
                wheelControls.style.alignItems = "center";
                wheelControls.style.backgroundColor = "rgba(0, 0, 0, 0.3)";
                wheelControls.style.borderRadius = "8px";
                wheelControls.style.margin = "0 15px";
            }
            
            // Set up the keyboard controls
            keyboardControls.style.display = "flex";
            keyboardControls.style.flexDirection = "column"; // Change to column to properly position the title
            keyboardControls.style.alignItems = "center";
            keyboardControls.style.justifyContent = "flex-start";
            keyboardControls.style.width = "auto";
            keyboardControls.style.margin = "0 15px";
            keyboardControls.style.padding = "15px";
            keyboardControls.style.paddingTop = "35px"; // Add padding at the top for the title
            keyboardControls.style.flexShrink = "0"; // Prevent shrinking
            keyboardControls.style.backgroundColor = "rgba(0, 0, 0, 0.3)";
            keyboardControls.style.borderRadius = "8px";
            keyboardControls.style.minWidth = "700px"; // Ensure minimum width to fit all parts
            keyboardControls.style.position = "relative"; // Needed for absolute positioning of the title
            
            // Make the title smaller and position it at the top center of the keyboard controls box
            const keyboardTitle = keyboardControls.querySelector('h2');
            if (keyboardTitle) {
                keyboardTitle.style.fontSize = "1rem";
                keyboardTitle.style.margin = "0";
                keyboardTitle.style.padding = "5px 0";
                keyboardTitle.style.textAlign = "center";
                keyboardTitle.style.whiteSpace = "nowrap";
                keyboardTitle.style.position = "absolute";
                keyboardTitle.style.top = "5px";
                keyboardTitle.style.left = "0";
                keyboardTitle.style.right = "0";
                keyboardTitle.style.width = "100%";
                keyboardTitle.style.color = "var(--text-primary)";
                keyboardTitle.style.zIndex = "1"; // Ensure it's above other elements
            }
            
            // Ensure the controls content has proper layout
            const controlsContent = keyboardControls.querySelector('.controls-content');
            if (controlsContent) {
                controlsContent.style.width = "100%";
                controlsContent.style.padding = "0";
                controlsContent.style.boxSizing = "border-box";
                controlsContent.style.display = "flex";
                controlsContent.style.flexDirection = "row"; // Arrange in a row
                controlsContent.style.alignItems = "center";
                controlsContent.style.justifyContent = "space-between";
                controlsContent.style.gap = "20px";
                controlsContent.style.margin = "0";
            }
            
            // COMPLETELY RESTRUCTURE THE LAYOUT
            // First, get all the components we need to rearrange
            const keyboardGrid = keyboardControls.querySelector('.keyboard-grid');
            const controlsRow = keyboardControls.querySelector('.controls-row');
            const controlsDescription = keyboardControls.querySelector('.controls-description');
            
            // Remove all existing styles that might interfere
            if (keyboardGrid) keyboardGrid.style = "";
            if (controlsRow) controlsRow.style = "";
            if (controlsDescription) controlsDescription.style = "";
            
            // Part 1: WASDXC Keyboard Grid on the left
            if (keyboardGrid) {
                keyboardGrid.style.margin = "0";
                keyboardGrid.style.width = "auto";
                keyboardGrid.style.display = "flex";
                keyboardGrid.style.flexDirection = "column";
                keyboardGrid.style.alignItems = "center";
                keyboardGrid.style.gap = "5px";
                keyboardGrid.style.flexShrink = "0"; // Prevent shrinking
                keyboardGrid.style.order = "1"; // Ensure it's first in the row
                
                // Make key rows more visible
                const keyRows = keyboardGrid.querySelectorAll('.key-row');
                keyRows.forEach(row => {
                    row.style.display = "flex";
                    row.style.justifyContent = "center";
                    row.style.width = "100%";
                    row.style.gap = "5px";
                    row.style.marginBottom = "5px";
                });
                
                // Make keys slightly smaller but ensure they're all visible
                const keys = keyboardGrid.querySelectorAll('.key');
                keys.forEach(key => {
                    key.style.width = "45px";
                    key.style.height = "45px";
                    key.style.fontSize = "1rem";
                    key.style.margin = "2px";
                    key.style.padding = "0";
                    key.style.display = "flex";
                    key.style.justifyContent = "center";
                    key.style.alignItems = "center";
                    key.style.flexShrink = "0"; // Prevent shrinking
                });
            }
            
            // Part 2: Acceleration and Steering Controls in the middle
            if (controlsRow) {
                controlsRow.style.width = "auto";
                controlsRow.style.display = "flex";
                controlsRow.style.flexDirection = "column"; // Stack acceleration on top of steering
                controlsRow.style.justifyContent = "center";
                controlsRow.style.gap = "10px";
                controlsRow.style.margin = "0 20px";
                controlsRow.style.flexShrink = "0"; // Prevent shrinking
                controlsRow.style.order = "2"; // Ensure it's in the middle
                
                // Fix acceleration controls - ensure title is above buttons
                const accelerationControls = controlsRow.querySelector('.acceleration-controls');
                if (accelerationControls) {
                    accelerationControls.style.width = "auto";
                    accelerationControls.style.marginBottom = "5px";
                    accelerationControls.style.display = "flex";
                    accelerationControls.style.flexDirection = "column";
                    accelerationControls.style.alignItems = "center";
                    
                    // Style the title if it exists
                    const accelerationTitle = accelerationControls.querySelector('h3');
                    if (accelerationTitle) {
                        accelerationTitle.style.marginBottom = "5px";
                        accelerationTitle.style.fontSize = "0.8rem";
                        accelerationTitle.style.whiteSpace = "nowrap";
                    }
                    
                    // Make sure acceleration buttons are visible
                    const accelerationButtonsRow = accelerationControls.querySelector('.key-row');
                    if (accelerationButtonsRow) {
                        accelerationButtonsRow.style.display = "flex";
                        accelerationButtonsRow.style.justifyContent = "center";
                        accelerationButtonsRow.style.gap = "5px";
                        accelerationButtonsRow.style.width = "100%";
                    }
                }
                
                // Fix steering controls - ensure title is above buttons
                const steeringControls = controlsRow.querySelector('.steering-controls');
                if (steeringControls) {
                    steeringControls.style.width = "auto";
                    steeringControls.style.display = "flex";
                    steeringControls.style.flexDirection = "column";
                    steeringControls.style.alignItems = "center";
                    
                    // Style the title if it exists
                    const steeringTitle = steeringControls.querySelector('h3');
                    if (steeringTitle) {
                        steeringTitle.style.marginBottom = "5px";
                        steeringTitle.style.fontSize = "0.8rem";
                        steeringTitle.style.whiteSpace = "nowrap";
                    }
                    
                    // Make sure steering buttons are visible
                    const steeringButtonsRow = steeringControls.querySelector('.key-row');
                    if (steeringButtonsRow) {
                        steeringButtonsRow.style.display = "flex";
                        steeringButtonsRow.style.justifyContent = "center";
                        steeringButtonsRow.style.gap = "5px";
                        steeringButtonsRow.style.width = "100%";
                    }
                }
                
                // Make acceleration and steering buttons more compact
                const actionButtons = controlsRow.querySelectorAll('.acceleration-button, .steering-button');
                actionButtons.forEach(button => {
                    button.style.width = "auto"; // Allow width to fit content
                    button.style.minWidth = "40px"; // Minimum width
                    button.style.height = "35px"; // Smaller height
                    button.style.padding = "0 8px"; // Add horizontal padding
                    button.style.fontSize = "0.85rem"; // Smaller font
                    button.style.whiteSpace = "nowrap"; // Prevent text wrapping
                    button.style.display = "flex";
                    button.style.justifyContent = "center";
                    button.style.alignItems = "center";
                    button.style.flexShrink = "0"; // Prevent shrinking
                });
            }
            
            // Part 3: Button Indicators (Controls Description) on the right
            if (controlsDescription) {
                controlsDescription.style.display = "block"; // Show it
                controlsDescription.style.margin = "0";
                controlsDescription.style.width = "auto";
                controlsDescription.style.maxHeight = "120px"; // Limit height to fit in the bar
                controlsDescription.style.overflowY = "auto"; // Add scrolling if needed
                controlsDescription.style.flexShrink = "0"; // Prevent shrinking
                controlsDescription.style.order = "3"; // Ensure it's last in the row
                
                // Style the list items to be more compact
                const controlsList = controlsDescription.querySelector('.controls-list');
                if (controlsList) {
                    controlsList.style.padding = "0";
                    controlsList.style.margin = "0";
                    
                    // Make list items more compact
                    const listItems = controlsList.querySelectorAll('li');
                    listItems.forEach(item => {
                        item.style.display = "flex";
                        item.style.alignItems = "center";
                        item.style.fontSize = "0.8rem";
                        item.style.marginBottom = "5px";
                        
                        // Style the key name
                        const keyName = item.querySelector('.key-name');
                        if (keyName) {
                            keyName.style.width = "25px";
                            keyName.style.height = "25px";
                            keyName.style.display = "flex";
                            keyName.style.justifyContent = "center";
                            keyName.style.alignItems = "center";
                            keyName.style.marginRight = "8px";
                            keyName.style.fontSize = "0.8rem";
                            keyName.style.flexShrink = "0";
                        }
                    });
                }
            }
            
            // Ensure the controls content has all three parts in the correct order
            if (controlsContent) {
                // First, remove all children
                while (controlsContent.firstChild) {
                    controlsContent.removeChild(controlsContent.firstChild);
                }
                
                // Then add them back in the correct order
                if (keyboardGrid) controlsContent.appendChild(keyboardGrid);
                if (controlsRow) controlsContent.appendChild(controlsRow);
                if (controlsDescription) controlsContent.appendChild(controlsDescription);
            }
        }
    }
    
    // Update fullscreen button to show exit icon
    if (fullscreenBtn) {
        fullscreenBtn.textContent = "⤓";
        fullscreenBtn.title = "Exit Fullscreen";
        fullscreenBtn.style.zIndex = "1003";
        fullscreenBtn.style.fontSize = "1.2rem";
    }
    
    // Hide the header
    const header = document.querySelector('.header');
    if (header) {
        header.style.display = 'none';
    }
    
    console.log("Fullscreen mode enabled");
}

function exitFullscreen() {
    console.log("Exiting fullscreen mode");
    isFullscreen = false;
    
    // Get all the elements we need to modify
    const body = document.body;
    const mainContent = document.querySelector('.main-content');
    const container = document.querySelector('.container');
    const videoColumn = document.querySelector('.video-column');
    const videoFeeds = document.querySelector('.video-feeds');
    const primaryFeed = document.querySelector('.video-feed.primary');
    const primaryFeedContainer = document.querySelector('.video-feed.primary .feed-container');
    const primaryFeedImg = document.querySelector('.video-feed.primary .feed-container img');
    const primaryFeedOverlay = document.querySelector('.video-feed.primary .feed-overlay');
    const feedLabel = primaryFeedOverlay ? primaryFeedOverlay.querySelector('.feed-label') : null;
    const feedControls = primaryFeedOverlay ? primaryFeedOverlay.querySelector('.feed-controls') : null;
    const secondaryFeed = document.querySelector('.video-feed.secondary');
    const controlsColumn = document.querySelector('.controls-column');
    const statusHistoryColumn = document.querySelector('.status-history-column');
    const fullscreenBtn = document.getElementById('frontCamFullscreen');
    
    // Show resize buttons when exiting fullscreen mode
    const resizeButtons = primaryFeedOverlay ? primaryFeedOverlay.querySelectorAll('.resize-btn') : null;
    if (resizeButtons) {
        resizeButtons.forEach(button => {
            button.style.display = '';
        });
    }
    
    if (!mainContent) {
        console.error("Main content element not found");
        return;
    }
    
    // Restore body scrolling
    body.style.overflow = '';
    
    // Remove fullscreen class
    mainContent.classList.remove('fullscreen-mode');
    
    // Reset main content styles
    mainContent.style.position = '';
    mainContent.style.top = '';
    mainContent.style.left = '';
    mainContent.style.width = '';
    mainContent.style.height = '';
    mainContent.style.padding = '';
    mainContent.style.margin = '';
    mainContent.style.backgroundColor = '';
    mainContent.style.display = '';
    mainContent.style.flexDirection = '';
    mainContent.style.justifyContent = '';
    mainContent.style.alignItems = '';
    mainContent.style.zIndex = '';
    
    // Reset container styles
    if (container) {
        container.style.padding = "";
        container.style.margin = "";
        container.style.maxWidth = "";
        container.style.width = "";
        container.style.height = "";
        container.style.overflow = "";
    }
    
    // Reset video column styles
    if (videoColumn) {
        videoColumn.style.position = "";
        videoColumn.style.top = "";
        videoColumn.style.left = "";
        videoColumn.style.right = "";
        videoColumn.style.width = "";
        videoColumn.style.height = "";
        videoColumn.style.display = "";
        videoColumn.style.justifyContent = "";
        videoColumn.style.alignItems = "";
        videoColumn.style.padding = "";
        videoColumn.style.margin = "";
        videoColumn.style.flex = "";
        videoColumn.style.order = "";
        videoColumn.style.zIndex = "";
    }
    
    // Reset video feeds styles
    if (videoFeeds) {
        videoFeeds.style.position = "";
        videoFeeds.style.width = "";
        videoFeeds.style.height = "";
        videoFeeds.style.display = "";
        videoFeeds.style.flexDirection = "";
        videoFeeds.style.justifyContent = "";
        videoFeeds.style.alignItems = "";
        videoFeeds.style.padding = "";
        videoFeeds.style.margin = "";
    }
    
    // Reset primary feed styles
    if (primaryFeed) {
        primaryFeed.style.position = "";
        primaryFeed.style.width = "";
        primaryFeed.style.height = "";
        primaryFeed.style.display = "";
        primaryFeed.style.justifyContent = "";
        primaryFeed.style.alignItems = "";
        primaryFeed.style.margin = "";
        primaryFeed.style.zIndex = "";
    }
    
    // Reset primary feed container styles
    if (primaryFeedContainer) {
        primaryFeedContainer.style.position = "";
        primaryFeedContainer.style.width = "";
        primaryFeedContainer.style.height = "";
        primaryFeedContainer.style.maxHeight = "";
        primaryFeedContainer.style.maxWidth = "";
        primaryFeedContainer.style.minWidth = "";
        primaryFeedContainer.style.minHeight = "";
        primaryFeedContainer.style.borderRadius = "";
        primaryFeedContainer.style.border = "";
        primaryFeedContainer.style.margin = "";
        primaryFeedContainer.style.overflow = "";
        primaryFeedContainer.style.backgroundColor = "";
        primaryFeedContainer.style.display = "";
        primaryFeedContainer.style.justifyContent = "";
        primaryFeedContainer.style.alignItems = "";
    }
    
    // Reset primary feed image styles
    if (primaryFeedImg) {
        primaryFeedImg.style.width = "";
        primaryFeedImg.style.height = "";
        primaryFeedImg.style.objectFit = "";
        primaryFeedImg.style.position = "";
        primaryFeedImg.style.display = "";
        primaryFeedImg.style.margin = "";
    }
    
    // Reset primary feed overlay styles
    if (primaryFeedOverlay) {
        primaryFeedOverlay.style.position = "";
        primaryFeedOverlay.style.top = "";
        primaryFeedOverlay.style.left = "";
        primaryFeedOverlay.style.right = "";
        primaryFeedOverlay.style.width = "";
        primaryFeedOverlay.style.boxSizing = "";
        primaryFeedOverlay.style.zIndex = "";
        primaryFeedOverlay.style.padding = "";
        primaryFeedOverlay.style.pointerEvents = "";
        primaryFeedOverlay.style.background = "";
        primaryFeedOverlay.style.display = "";
        primaryFeedOverlay.style.justifyContent = "";
        primaryFeedOverlay.style.alignItems = "";
    }
    
    // Reset feed label styles
    if (feedLabel) {
        feedLabel.style.color = "";
        feedLabel.style.fontWeight = "";
        feedLabel.style.fontSize = "";
        feedLabel.style.textTransform = "";
        feedLabel.style.letterSpacing = "";
        feedLabel.style.textShadow = "";
    }
    
    // Reset feed controls styles
    if (feedControls) {
        feedControls.style.display = "";
        feedControls.style.alignItems = "";
        feedControls.style.gap = "";
    }
    
    // Reset secondary feed styles
    if (secondaryFeed) {
        secondaryFeed.style.position = "";
        secondaryFeed.style.top = "";
        secondaryFeed.style.left = "";
        secondaryFeed.style.right = "";
        secondaryFeed.style.transform = "";
        secondaryFeed.style.width = "";
        secondaryFeed.style.zIndex = "";
    }
    
    // Reset controls column styles
    if (controlsColumn) {
        controlsColumn.style.position = "";
        controlsColumn.style.bottom = "";
        controlsColumn.style.left = "";
        controlsColumn.style.right = "";
        controlsColumn.style.transform = "";
        controlsColumn.style.width = "";
        controlsColumn.style.maxHeight = "";
        controlsColumn.style.opacity = "";
        controlsColumn.style.zIndex = "";
        controlsColumn.style.overflow = "";
        controlsColumn.style.display = "";
        controlsColumn.style.flexDirection = "";
        controlsColumn.style.justifyContent = "";
        controlsColumn.style.alignItems = "";
        controlsColumn.style.padding = "";
        controlsColumn.style.margin = "";
        controlsColumn.style.backgroundColor = "";
        
        // Reset keyboard controls
        const keyboardControls = controlsColumn.querySelector('.keyboard-controls');
        if (keyboardControls) {
            keyboardControls.style.width = "";
            keyboardControls.style.minWidth = "";
            keyboardControls.style.margin = "";
            keyboardControls.style.padding = "";
            keyboardControls.style.display = "";
            keyboardControls.style.flexDirection = "";
        }
        
        // Reset controls content
        const controlsContent = controlsColumn.querySelector('.controls-content');
        if (controlsContent) {
            controlsContent.style.display = "";
            controlsContent.style.flexDirection = "";
            controlsContent.style.width = "";
            controlsContent.style.padding = "";
        }
        
        // Reset keyboard grid
        const keyboardGrid = controlsColumn.querySelector('.keyboard-grid');
        if (keyboardGrid) {
            keyboardGrid.style.display = "";
            keyboardGrid.style.flexDirection = "";
        }
        
        // Reset key styles
        const keys = controlsColumn.querySelectorAll('.key');
        keys.forEach(key => {
            key.style.width = "";
            key.style.height = "";
            key.style.fontSize = "";
            key.style.flexShrink = "";
        });
        
        // Reset controls row
        const controlsRow = controlsColumn.querySelector('.controls-row');
        if (controlsRow) {
            controlsRow.style.display = "";
            controlsRow.style.flexDirection = "";
            controlsRow.style.gap = "";
        }
        
        // Reset acceleration and steering controls
        const accelerationControls = controlsColumn.querySelector('.acceleration-controls');
        if (accelerationControls) {
            accelerationControls.style.display = "";
            accelerationControls.style.flexDirection = "";
            accelerationControls.style.alignItems = "";
            accelerationControls.style.margin = "";
        }
        
        const steeringControls = controlsColumn.querySelector('.steering-controls');
        if (steeringControls) {
            steeringControls.style.display = "";
            steeringControls.style.flexDirection = "";
            steeringControls.style.alignItems = "";
            steeringControls.style.margin = "";
        }
        
        // Reset controls description
        const controlsDescription = controlsColumn.querySelector('.controls-description');
        if (controlsDescription) {
            controlsDescription.style.display = "";
        }
        
        // Reset wheel controls
        const wheelControls = controlsColumn.querySelector('.wheel-controls');
        if (wheelControls) {
            wheelControls.style.width = "";
            wheelControls.style.margin = "";
            wheelControls.style.padding = "";
        }
        
        // Reset wheel and pedals content
        const wheelAndPedalsContent = controlsColumn.querySelector('.wheel-and-pedals-content');
        if (wheelAndPedalsContent) {
            wheelAndPedalsContent.style.display = "";
            wheelAndPedalsContent.style.flexDirection = "";
            wheelAndPedalsContent.style.justifyContent = "";
            wheelAndPedalsContent.style.alignItems = "";
        }
        
        // Reset steering wheel
        const steeringWheel = controlsColumn.querySelector('.steering-wheel');
        if (steeringWheel) {
            steeringWheel.style.width = "";
            steeringWheel.style.height = "";
            steeringWheel.style.flexShrink = "";
        }
        
        // Reset pedals
        const pedals = controlsColumn.querySelector('.pedals');
        if (pedals) {
            pedals.style.display = "";
            pedals.style.flexDirection = "";
            pedals.style.gap = "";
        }
        
        // Reset drive mode container
        const driveModeContainer = controlsColumn.querySelector('.drive-mode-container');
        if (driveModeContainer) {
            driveModeContainer.style.width = "";
            driveModeContainer.style.margin = "";
            driveModeContainer.style.flexShrink = "";
        }
    }
    
    // Reset status history column styles
    if (statusHistoryColumn) {
        statusHistoryColumn.style.position = "";
        statusHistoryColumn.style.top = "";
        statusHistoryColumn.style.left = "";
        statusHistoryColumn.style.right = ""; // Make sure to reset right position
        statusHistoryColumn.style.transform = "";
        statusHistoryColumn.style.width = "";
        statusHistoryColumn.style.maxHeight = "";
        statusHistoryColumn.style.opacity = "";
        statusHistoryColumn.style.zIndex = "";
        statusHistoryColumn.style.overflow = "";
        
        // Reset history panel display
        const historyPanel = statusHistoryColumn.querySelector('.history-panel');
        if (historyPanel) {
            historyPanel.style.display = "";
        }
        
        // Reset command log display
        const commandLog = statusHistoryColumn.querySelector('.command-log');
        if (commandLog) {
            commandLog.style.display = "";
        }
    }
    
    // Reset fullscreen button to show fullscreen icon
    if (fullscreenBtn) {
        fullscreenBtn.textContent = "⛶";
        fullscreenBtn.title = "Fullscreen";
        fullscreenBtn.style.zIndex = "";
        fullscreenBtn.style.fontSize = "";
    }
    
    // Show the header again
    const header = document.querySelector('.header');
    if (header) {
        header.style.display = '';
    }
    
    // Force a small delay and then trigger a resize event to help layout recalculation
    setTimeout(() => {
        window.dispatchEvent(new Event('resize'));
    }, 50);
    
    console.log("Fullscreen mode disabled");
}

function toggleFullscreen() {
    console.log("toggleFullscreen called, current state:", isFullscreen);
    if (!isFullscreen) {
        enterFullscreen();
    } else {
        exitFullscreen();
    }
}

// Add this function after the toggleFullscreen function
function setupResizeButtons() {
    console.log("Setting up resize buttons");
    
    // Get all resize buttons
    const resizeButtons = document.querySelectorAll('.resize-btn');
    
    resizeButtons.forEach(button => {
        // Remove existing event listeners by cloning and replacing
        const newButton = button.cloneNode(true);
        button.parentNode.replaceChild(newButton, button);
        
        // Add click event listener to the new button
        newButton.addEventListener('click', function(e) {
            e.preventDefault();
            e.stopPropagation();
            
            const action = this.getAttribute('data-action');
            const target = this.getAttribute('data-target');
            
            console.log(`Resize button clicked: ${action} for ${target}`);
            
            // Get the target feed container
            const feedContainer = document.querySelector(`.video-feed.${target} .feed-container`);
            if (!feedContainer) {
                console.error(`Feed container for ${target} not found`);
                return;
            }
            
            // Get current width
            const currentWidth = parseInt(getComputedStyle(feedContainer).width);
            let newWidth;
            
            // Calculate new width based on action
            if (action === 'increase') {
                newWidth = currentWidth * 1.1; // Increase by 10%
            } else if (action === 'decrease') {
                newWidth = currentWidth * 0.9; // Decrease by 10%
            }
            
            // Apply new width
            feedContainer.style.width = `${newWidth}px`;
            
            console.log(`Resized ${target} feed to ${newWidth}px`);
        });
    });
}

// Update the IIFE at the bottom of the file to include the resize buttons setup and telemetry updates
(function() {
    // Function to set up the fullscreen button
    function setupFullscreenButton() {
        const fullscreenBtn = document.getElementById('frontCamFullscreen');
        if (!fullscreenBtn) {
            console.error("Fullscreen button not found in the DOM");
            return;
        }
        
        console.log("Fullscreen button found, setting up direct click handler");
        
        // Remove the button and replace it with a clone to remove any existing listeners
        const parent = fullscreenBtn.parentNode;
        const newBtn = fullscreenBtn.cloneNode(true);
        parent.replaceChild(newBtn, fullscreenBtn);
        
        // Add a direct onclick handler
        newBtn.onclick = function(e) {
            e.preventDefault();
            e.stopPropagation();
            console.log("Fullscreen button clicked directly");
            
            // Direct call to enterFullscreen
            if (!isFullscreen) {
                enterFullscreen();
            } else {
                exitFullscreen();
            }
            
            return false;
        };
    }
    
    // Set up the escape key handler
    function setupEscapeKeyHandler() {
        document.addEventListener('keydown', function(e) {
            if (e.key === 'Escape' && isFullscreen) {
                console.log("Escape key pressed, exiting fullscreen");
                exitFullscreen();
            }
        });
    }
    
    // Try to set up the button immediately if the DOM is already loaded
    if (document.readyState === "complete" || document.readyState === "interactive") {
        setupFullscreenButton();
        setupEscapeKeyHandler();
        setupResizeButtons();
        setupTelemetryUpdates();
    }
    
    // Also set it up on DOMContentLoaded to be safe
    document.addEventListener('DOMContentLoaded', function() {
        setupFullscreenButton();
        setupEscapeKeyHandler();
        setupResizeButtons();
        setupTelemetryUpdates();
    });
    
    // And set it up on load as a final fallback
    window.addEventListener('load', function() {
        setupFullscreenButton();
        setupEscapeKeyHandler();
        setupResizeButtons();
        setupTelemetryUpdates();
    });
})();
