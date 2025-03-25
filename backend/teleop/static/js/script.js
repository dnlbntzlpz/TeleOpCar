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
    const driveModeToggle = document.getElementById("drive-mode-toggle");
    if (!driveModeToggle) return; // Add a guard clause to prevent errors
    
    if (drivingMode === "forward") {
        driveModeToggle.classList.add("drive-mode-forward");
        driveModeToggle.classList.remove("drive-mode-reverse");
    } else {
        driveModeToggle.classList.add("drive-mode-reverse");
        driveModeToggle.classList.remove("drive-mode-forward");
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
    
    // steering axis is -1 (full left) to 1 (full right)
    // rotate steering wheel image -180deg to +180deg
    const wheelRotation = steering * 180;
    updateSteeringWheel(wheelRotation);

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
    
    // Position secondary feed at the top right corner instead of top center
    if (secondaryFeed) {
        // First, completely remove any existing inline styles that might be interfering
        secondaryFeed.removeAttribute('style');
        
        // Check if we're in mobile view to apply mobile-specific sizing
        if (isMobileDevice()) {
            // Make rear camera 50% smaller for mobile fullscreen
        secondaryFeed.style.cssText = `
            position: fixed !important;
            top: 20px !important;
            left: auto !important;
            right: 20px !important;
            transform: none !important;
                width: 150px !important; /* Reduced from 300px to 150px (50%) */
            z-index: 9999 !important;
            margin: 0 !important;
            padding: 0 !important;
        `;
        } else {
            // Desktop view - maintain current size
        secondaryFeed.style.cssText = `
            position: fixed !important;
            top: 20px !important;
            left: auto !important;
            right: 20px !important;
            transform: none !important;
            width: 300px !important;
            z-index: 9999 !important;
            margin: 0 !important;
            padding: 0 !important;
        `;
        }
        
        // Make sure the feed container has the correct styles
        const secondaryFeedContainer = secondaryFeed.querySelector('.feed-container');
        if (secondaryFeedContainer) {
            secondaryFeedContainer.removeAttribute('style');
            secondaryFeedContainer.style.cssText = `
                width: 100% !important;
                height: auto !important;
                margin: 0 !important;
                border: 2px solid #00a8ff !important;
                border-radius: 8px !important;
                overflow: hidden !important;
            `;
            
            // Ensure the image fills the container
            const secondaryFeedImg = secondaryFeedContainer.querySelector('img');
            if (secondaryFeedImg) {
                secondaryFeedImg.removeAttribute('style');
                secondaryFeedImg.style.cssText = `
                    width: 100% !important;
                    height: 100% !important;
                    object-fit: cover !important;
                    display: block !important;
                `;
            }
        }
        
        // Make sure the overlay is properly positioned and text size is appropriate for mobile
        const secondaryFeedOverlay = secondaryFeed.querySelector('.feed-overlay');
        if (secondaryFeedOverlay) {
            secondaryFeedOverlay.removeAttribute('style');
            
            if (isMobileDevice()) {
                // Smaller text and padding for mobile
                secondaryFeedOverlay.style.cssText = `
                    position: absolute !important;
                    top: 0 !important;
                    left: 0 !important;
                    right: 0 !important;
                    width: 100% !important;
                    z-index: 1 !important;
                    padding: 4px !important; /* Reduced padding */
                    background: linear-gradient(180deg, rgba(0,0,0,0.6) 0%, transparent 100%) !important;
                    display: flex !important;
                    justify-content: space-between !important;
                    align-items: center !important;
                `;
                
                // Make the label text smaller for mobile
                const feedLabel = secondaryFeedOverlay.querySelector('.feed-label');
                if (feedLabel) {
                    feedLabel.style.fontSize = '0.7rem';
                }
                
                // Make status text smaller for mobile
                const feedStatus = secondaryFeedOverlay.querySelector('.feed-status');
                if (feedStatus) {
                    feedStatus.style.fontSize = '0.7rem';
                }
            } else {
                // Regular styling for desktop
            secondaryFeedOverlay.style.cssText = `
                position: absolute !important;
                top: 0 !important;
                left: 0 !important;
                right: 0 !important;
                width: 100% !important;
                z-index: 1 !important;
                padding: 8px !important;
                background: linear-gradient(180deg, rgba(0,0,0,0.6) 0%, transparent 100%) !important;
                display: flex !important;
                justify-content: space-between !important;
                align-items: center !important;
            `;
            }
        }
        
        // Try to force a reflow to ensure changes take effect
        secondaryFeed.offsetHeight;
        
        // Log to console for debugging
        console.log("Applied forced styles to secondary feed:", secondaryFeed);
    }
    
    // Position status history column on the left side instead of right for mobile
    if (statusHistoryColumn) {
        // First check if we're in mobile view
        if (isMobileDevice()) {
        statusHistoryColumn.style.position = "fixed";
            statusHistoryColumn.style.left = `${sideColumnMargin}px`; // Position on left
            statusHistoryColumn.style.right = "auto"; // Clear right positioning
            statusHistoryColumn.style.top = "10px";  // Position at top
            statusHistoryColumn.style.transform = "none"; // Remove transform
            statusHistoryColumn.style.width = "45%"; // Narrower width
            statusHistoryColumn.style.maxHeight = "auto";
            statusHistoryColumn.style.maxWidth = "300px";
            statusHistoryColumn.style.overflowY = "auto";
            statusHistoryColumn.style.margin = "0";
            statusHistoryColumn.style.opacity = "0.75"; 
            statusHistoryColumn.style.zIndex = "1002";
            statusHistoryColumn.style.padding = "8px";
            statusHistoryColumn.style.boxSizing = "border-box";
            statusHistoryColumn.style.backdropFilter = "blur(5px)";
            
            // Try to find the secondary feed and make sure it's on the right side
            const secondaryFeed = document.querySelector('.video-feed.secondary');
            if (secondaryFeed) {
                secondaryFeed.style.top = "10px";
                secondaryFeed.style.right = "10px";
                secondaryFeed.style.left = "auto";
            }
        } else {
            // Desktop position remains on the right
            statusHistoryColumn.style.position = "fixed";
            statusHistoryColumn.style.right = `${sideColumnMargin}px`; 
            statusHistoryColumn.style.left = "auto"; 
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
        }
        
        // Store handlers so they can be removed later
        statusHistoryColumn.mouseEnterHandler = function() {
            this.style.opacity = "1";
        };
        statusHistoryColumn.mouseLeaveHandler = function() {
            this.style.opacity = "0.25";
        };
        
        // Add hover effect to make it fully opaque when hovered
        statusHistoryColumn.addEventListener('mouseenter', statusHistoryColumn.mouseEnterHandler);
        statusHistoryColumn.addEventListener('mouseleave', statusHistoryColumn.mouseLeaveHandler);
        
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
        
        // Specifically target the vehicle status panel
        const vehicleStatusPanel = statusHistoryColumn.querySelector('.status-panel');
        if (vehicleStatusPanel) {
            if (isMobileDevice()) {
                vehicleStatusPanel.style.minHeight = "auto";
                vehicleStatusPanel.style.maxHeight = "90vh";
                vehicleStatusPanel.style.backgroundColor = "rgba(0, 0, 0, 0.6)";
                vehicleStatusPanel.style.borderRadius = "8px";
                
                // Remove title to save space on mobile
                const statusTitle = vehicleStatusPanel.querySelector('h2');
                if (statusTitle) {
                    statusTitle.style.display = "none";
                }
            } else {
            vehicleStatusPanel.style.minHeight = "520px";
            vehicleStatusPanel.style.height = "auto";
                vehicleStatusPanel.style.backgroundColor = "rgba(0, 0, 0, 0.3)";
                vehicleStatusPanel.style.borderRadius = "8px";
            }
        }
        
        // Mobile-specific grid layout
        const statusGrid = statusHistoryColumn.querySelector('.status-grid');
        if (statusGrid && isMobileDevice()) {
            statusGrid.style.cssText = "grid-template-columns: 1fr !important; width: 100%; gap: 5px; padding: 5px;";
            
            // Compact status items for mobile
            const statusItems = statusGrid.querySelectorAll('.status-item');
            statusItems.forEach(item => {
                item.style.width = "100%";
                item.style.boxSizing = "border-box";
                item.style.minWidth = "0";
                item.style.overflow = "hidden";
                item.style.display = "flex";
                item.style.alignItems = "center";
                item.style.padding = "5px 8px";
                item.style.marginBottom = "3px";
                item.style.backgroundColor = "rgba(0, 0, 0, 0.4)";
                item.style.borderRadius = "6px";
                item.style.minHeight = "28px";
                
                // Adjust icon and text size
                const statusIcon = item.querySelector('.status-icon');
                if (statusIcon) {
                    statusIcon.style.fontSize = "0.9rem";
                    statusIcon.style.marginRight = "8px";
                }
                
                const statusValue = item.querySelector('.status-value');
                if (statusValue) {
                    statusValue.style.fontSize = "0.9rem";
                }
            });
        } else if (statusGrid) {
            // Desktop grid
            statusGrid.style.cssText = "grid-template-columns: 1fr !important; width: 100%; gap: 10px; padding: 5px;";
            
            // Desktop status items
            const statusItems = statusGrid.querySelectorAll('.status-item');
            statusItems.forEach(item => {
                item.style.width = "100%";
                item.style.boxSizing = "border-box";
                item.style.minWidth = "0";
                item.style.overflow = "hidden";
                item.style.display = "flex";
                item.style.alignItems = "center";
                item.style.padding = "10px";
                item.style.marginBottom = "5px";
                item.style.backgroundColor = "rgba(0, 0, 0, 0.2)";
                item.style.borderRadius = "6px";
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
        controlsColumn.style.backdropFilter = "none";
        controlsColumn.style.borderTop = "none"; // Add top border
        controlsColumn.style.display = "flex"; // Use flexbox for layout
        controlsColumn.style.justifyContent = "center"; // Center the controls horizontally
        controlsColumn.style.alignItems = "center"; // Center the controls vertically
        controlsColumn.style.flexDirection = "row"; // Ensure horizontal layout
        
        // Store handlers so they can be removed later
        controlsColumn.mouseEnterHandler = function() {
            this.style.opacity = "1";
        };
        controlsColumn.mouseLeaveHandler = function() {
            this.style.opacity = "0.75";
        };
        
        // Add hover effect to make it fully opaque when hovered
        controlsColumn.addEventListener('mouseenter', controlsColumn.mouseEnterHandler);
        controlsColumn.addEventListener('mouseleave', controlsColumn.mouseLeaveHandler);
        
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
    
    // ADD THIS SECTION: Check if we're in mobile view and hide the bottom control bar and status panel
    if (isMobileDevice()) {
        // Hide the controls column completely on mobile fullscreen
        if (controlsColumn) {
            controlsColumn.style.display = "none";
        }
        
        // Hide the status history column completely on mobile fullscreen
        if (statusHistoryColumn) {
            statusHistoryColumn.style.display = "none";
        }
        
        // Add mobile-specific touch controls if they don't exist
        addFullscreenMobileControls();
    }
    
    // Hide the header
    const header = document.querySelector('.header');
    if (header) {
        header.style.display = 'none';
    }
    
    console.log("Fullscreen mode enabled");
}

// Create this new function to add mobile fullscreen controls
function addFullscreenMobileControls() {
    // Only add if they don't already exist
    if (document.getElementById('mobile-fullscreen-controls')) return;
    
    const container = document.querySelector('.container');
    if (!container) return;
    
    // Create a floating control panel
    const controlPanel = document.createElement('div');
    controlPanel.id = 'mobile-fullscreen-controls';
    controlPanel.innerHTML = `
        <div class="mobile-fullscreen-buttons">
            <button id="mobile-fs-stop" class="mobile-fs-btn emergency">⏹</button>
            <button id="mobile-fs-center" class="mobile-fs-btn center">⊕</button>
        </div>
    `;
    container.appendChild(controlPanel);
    
    // Style the floating control panel
    controlPanel.style.position = 'fixed';
    controlPanel.style.bottom = '20px';
    controlPanel.style.right = '20px';
    controlPanel.style.zIndex = '9999';
    controlPanel.style.display = 'flex';
    controlPanel.style.flexDirection = 'column';
    controlPanel.style.alignItems = 'flex-end';
    
    // Style the buttons container
    const buttonsContainer = controlPanel.querySelector('.mobile-fullscreen-buttons');
    buttonsContainer.style.display = 'flex';
    buttonsContainer.style.flexDirection = 'column';
    buttonsContainer.style.gap = '10px';
    
    // Style all buttons
    const buttons = controlPanel.querySelectorAll('.mobile-fs-btn');
    buttons.forEach(btn => {
        btn.style.width = '60px';
        btn.style.height = '60px';
        btn.style.borderRadius = '50%';
        btn.style.border = 'none';
        btn.style.fontSize = '28px';
        btn.style.fontWeight = 'bold';
        btn.style.display = 'flex';
        btn.style.justifyContent = 'center';
        btn.style.alignItems = 'center';
        btn.style.boxShadow = '0 4px 8px rgba(0,0,0,0.3)';
        btn.style.opacity = '0.7';
        btn.style.transition = 'opacity 0.2s, transform 0.2s';
    });
    
    // Add hover effects
    buttons.forEach(btn => {
        btn.addEventListener('touchstart', () => {
            btn.style.opacity = '1';
            btn.style.transform = 'scale(1.1)';
        });
        
        btn.addEventListener('touchend', () => {
            btn.style.opacity = '0.7';
            btn.style.transform = 'scale(1)';
        });
    });
    
    // Style the emergency stop button
    const stopBtn = document.getElementById('mobile-fs-stop');
    if (stopBtn) {
        stopBtn.style.backgroundColor = '#e74c3c';
        stopBtn.style.color = 'white';
        stopBtn.addEventListener('click', () => sendCommand('stop'));
    }
    
    // Style the center button
    const centerBtn = document.getElementById('mobile-fs-center');
    if (centerBtn) {
        centerBtn.style.backgroundColor = '#3498db';
        centerBtn.style.color = 'white';
        centerBtn.addEventListener('click', () => sendCommand('center'));
    }
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
    mainContent.style = "";
    
    // Reset container styles
    if (container) {
        container.style = "";
    }
    
    // Reset video column styles
    if (videoColumn) {
        videoColumn.style = "";
    }
    
    // Reset video feeds styles
    if (videoFeeds) {
        videoFeeds.style = "";
    }
    
    // Reset primary feed styles
    if (primaryFeed) {
        primaryFeed.style = "";
    }
    
    // Reset primary feed container styles
    if (primaryFeedContainer) {
        primaryFeedContainer.style = "";
    }
    
    // Reset primary feed image styles
    if (primaryFeedImg) {
        primaryFeedImg.style = "";
    }
    
    // Reset primary feed overlay styles
    if (primaryFeedOverlay) {
        primaryFeedOverlay.style = "";
    }
    
    // Reset feed label styles
    if (feedLabel) {
        feedLabel.style = "";
    }
    
    // Reset feed controls styles
    if (feedControls) {
        feedControls.style = "";
    }
    
    // Reset secondary feed styles
    if (secondaryFeed) {
        secondaryFeed.style = "";
    }
    
    // Reset controls column styles - this is the key part that was failing
    if (controlsColumn) {
        // Remove the event listeners completely
        if (controlsColumn.mouseEnterHandler) {
            controlsColumn.removeEventListener('mouseenter', controlsColumn.mouseEnterHandler);
            controlsColumn.removeEventListener('mouseleave', controlsColumn.mouseLeaveHandler);
            // Clear the handlers
            controlsColumn.mouseEnterHandler = null;
            controlsColumn.mouseLeaveHandler = null;
        }
        
        // Clear all styles completely
        controlsColumn.style = "";
        
        // ADD THIS: Make sure controls are visible when exiting fullscreen
        controlsColumn.style.display = "";
        
        // Reset keyboard controls by clearing all styles
        const keyboardControls = controlsColumn.querySelector('.keyboard-controls');
        if (keyboardControls) {
            keyboardControls.style = "";
            
            // Get all elements within keyboard controls and reset their styles too
            const allKeyboardElements = keyboardControls.querySelectorAll('*');
            allKeyboardElements.forEach(element => {
                element.style = "";
            });
        }
        
        // Reset wheel controls by clearing all styles
        const wheelControls = controlsColumn.querySelector('.wheel-controls');
        if (wheelControls) {
            wheelControls.style = "";
            
            // Get all elements within wheel controls and reset their styles too
            const allWheelElements = wheelControls.querySelectorAll('*');
            allWheelElements.forEach(element => {
                element.style = "";
            });
        }
    }
    
    // Reset status history column styles
    if (statusHistoryColumn) {
        // Remove the event listeners completely
        if (statusHistoryColumn.mouseEnterHandler) {
            statusHistoryColumn.removeEventListener('mouseenter', statusHistoryColumn.mouseEnterHandler);
            statusHistoryColumn.removeEventListener('mouseleave', statusHistoryColumn.mouseLeaveHandler);
            // Clear the handlers
            statusHistoryColumn.mouseEnterHandler = null;
            statusHistoryColumn.mouseLeaveHandler = null;
        }
        
        // Clear all styles completely
        statusHistoryColumn.style = "";
        
        // Reset all child elements
        const statusItems = statusHistoryColumn.querySelectorAll('*');
        statusItems.forEach(item => {
            item.style = "";
        });
        
        // Make sure the history panel is displayed again
        const historyPanel = statusHistoryColumn.querySelector('.history-panel');
        if (historyPanel) {
            historyPanel.style.display = "";
        }
    }
    
    // Reset fullscreen button to show fullscreen icon
    if (fullscreenBtn) {
        fullscreenBtn.textContent = "⛶";
        fullscreenBtn.title = "Fullscreen";
        fullscreenBtn.style = "";
    }
    
    // Show the header again
    const header = document.querySelector('.header');
    if (header) {
        header.style.display = '';
    }
    
    // ADD THIS: Remove any fullscreen mobile controls
    const mobileFullscreenControls = document.getElementById('mobile-fullscreen-controls');
    if (mobileFullscreenControls) {
        mobileFullscreenControls.remove();
    }
    
    // Force a small delay and then trigger a resize event to help layout recalculation
    setTimeout(() => {
        window.dispatchEvent(new Event('resize'));
    }, 100);
    
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

// Add mobile responsiveness functions
function isMobileDevice() {
    // Check for touch capability and screen size
    return (('ontouchstart' in window) || 
            (navigator.maxTouchPoints > 0) || 
            (navigator.msMaxTouchPoints > 0)) && 
           window.innerWidth < 768;
}

function setupMobileLayout() {
    const isMobile = isMobileDevice();
    document.body.classList.toggle('mobile-view', isMobile);
    
    // When switching between mobile and desktop views, 
    // always clean up any lingering elements first
    cleanupMobileElements();
    
    if (isMobile) {
        // Apply mobile-specific optimizations
        applyMobileOptimizations();
    }
    
    // Trigger a layout adjustment
    setTimeout(() => {
        window.dispatchEvent(new Event('resize'));
    }, 100);
}

function cleanupMobileElements() {
    // Remove any mobile-specific elements that might have been created
    const mobileNav = document.querySelector('.mobile-nav');
    if (mobileNav) {
        mobileNav.remove();
    }
    
    // Remove any steering indicators
    const steerIndicator = document.getElementById('steer-indicator');
    if (steerIndicator) {
        steerIndicator.remove();
    }
    
    // Reset styles on elements that might have been modified
    const elements = [
        '.main-content', '.video-column', '.controls-column', 
        '.status-history-column', '.video-feeds', '.video-feed',
        '.keyboard-controls', '.wheel-controls', '.key',
        '.acceleration-button', '.steering-button'
    ];
    
    elements.forEach(selector => {
        const items = document.querySelectorAll(selector);
        items.forEach(el => {
            if (!isMobileDevice()) {
                el.style = ""; // Only reset styles when going back to desktop
            }
        });
    });
    
    // Make sure hidden elements are visible again on desktop
    if (!isMobileDevice()) {
        const hiddenElements = document.querySelectorAll('.controls-description, .drive-mode-container, .wheel-controls, .history-panel');
        hiddenElements.forEach(el => {
            el.style.display = '';
        });
    }
}

function applyMobileOptimizations() {
    console.log("Applying mobile optimizations");
    
    // 1. Main layout structure
    const mainContent = document.querySelector('.main-content');
    if (mainContent) {
        mainContent.style.display = 'flex';
        mainContent.style.flexDirection = 'column';
        mainContent.style.alignItems = 'center';
        mainContent.style.gap = '10px';
        mainContent.style.padding = '10px 5px 70px 5px'; // Extra padding at bottom for mobile nav
    }
    
    // 2. Video feed optimizations
    optimizeVideoFeeds();
    
    // 3. Controls optimization
    optimizeControls();
    
    // 4. Status panel optimization
    optimizeStatusPanel();
    
    // 5. Add fixed navigation for core actions
    addMobileNavBar();
}

function optimizeVideoFeeds() {
    const videoColumn = document.querySelector('.video-column');
    const videoFeeds = document.querySelector('.video-feeds');
    const primaryFeed = document.querySelector('.video-feed.primary');
    const secondaryFeed = document.querySelector('.video-feed.secondary');
    
    // Make video column full width
    if (videoColumn) {
        videoColumn.style.width = '100%';
        videoColumn.style.minWidth = 'unset';
        videoColumn.style.maxWidth = '100%';
    }
    
    // Change feeds to stack vertically
    if (videoFeeds) {
        videoFeeds.style.flexDirection = 'column';
        videoFeeds.style.alignItems = 'center';
        videoFeeds.style.width = '100%';
    }
    
    // Make primary feed full width
    if (primaryFeed) {
        primaryFeed.style.width = '100%';
        primaryFeed.style.margin = '0';
        
        const feedContainer = primaryFeed.querySelector('.feed-container');
        if (feedContainer) {
            feedContainer.style.width = '100%';
            feedContainer.style.maxWidth = '100%';
            // Use aspect ratio to maintain feed proportions
            feedContainer.style.aspectRatio = '16/9';
            
            // Add swipe gesture support for steering
            addSwipeGestures(feedContainer);
        }
    }
    
    // Make secondary feed smaller
    if (secondaryFeed) {
        secondaryFeed.style.width = '60%';
        secondaryFeed.style.margin = '5px auto';
        
        const secondaryContainer = secondaryFeed.querySelector('.feed-container');
        if (secondaryContainer) {
            secondaryContainer.style.width = '100%';
            secondaryContainer.style.border = '2px solid #00a8ff';
            secondaryContainer.style.borderRadius = '8px';
        }
    }
}

function optimizeControls() {
    const controlsColumn = document.querySelector('.controls-column');
    const keyboardControls = document.querySelector('.keyboard-controls');
    const wheelControls = document.querySelector('.wheel-controls');
    
    // Full width but centered controls
    if (controlsColumn) {
        controlsColumn.style.width = '100%';
        controlsColumn.style.maxWidth = '600px';
        controlsColumn.style.margin = '0 auto';
        controlsColumn.style.padding = '10px 5px';
    }
    
    // Make all touch controls larger
    const touchControls = document.querySelectorAll('.key, .acceleration-button, .steering-button');
    touchControls.forEach(control => {
        control.style.minHeight = '60px';
        control.style.minWidth = '60px';
        control.style.fontSize = '1.4rem';
        control.style.borderRadius = '10px'; 
        control.style.margin = '5px';
    });
    
    // Make WASD controls even larger
    const primaryControls = document.querySelectorAll('#keyW, #keyA, #keyS, #keyD');
    primaryControls.forEach(key => {
        key.style.minHeight = '75px';
        key.style.minWidth = '75px';
        key.style.fontSize = '1.8rem';
        key.style.fontWeight = 'bold';
    });
    
    // Hide wheel controls on mobile (too complex for small screens)
    if (wheelControls) {
        wheelControls.style.display = 'none';
    }
    
    // Hide controls description to save space
    const controlsDescription = document.querySelector('.controls-description');
    if (controlsDescription) {
        controlsDescription.style.display = 'none';
    }
    
    // Center keyboard grid keys
    if (keyboardControls) {
        keyboardControls.style.width = '100%';
        
        const keyboardGrid = keyboardControls.querySelector('.keyboard-grid');
        if (keyboardGrid) {
            keyboardGrid.style.margin = '0 auto';
            keyboardGrid.style.display = 'flex';
            keyboardGrid.style.flexDirection = 'column';
            keyboardGrid.style.alignItems = 'center';
            
            const keyRows = keyboardGrid.querySelectorAll('.key-row');
        keyRows.forEach(row => {
            row.style.display = 'flex';
            row.style.justifyContent = 'center';
                row.style.gap = '8px';
            });
        }
        
        // Better layout for acceleration/steering controls
        const controlsRow = keyboardControls.querySelector('.controls-row');
        if (controlsRow) {
            controlsRow.style.display = 'flex';
            controlsRow.style.flexDirection = 'row';
            controlsRow.style.flexWrap = 'wrap';
            controlsRow.style.justifyContent = 'center';
            controlsRow.style.gap = '20px';
            controlsRow.style.marginTop = '15px';
        }
    }
}

function optimizeStatusPanel() {
    const statusColumn = document.querySelector('.status-history-column');
    if (!statusColumn) return;
    
    statusColumn.style.width = '100%';
    statusColumn.style.maxWidth = '600px';
    statusColumn.style.margin = '0 auto';
    
    // Optimize status grid
    const statusGrid = statusColumn.querySelector('.status-grid');
    if (statusGrid) {
        statusGrid.style.gridTemplateColumns = 'repeat(2, 1fr)';
        statusGrid.style.gap = '8px';
        
        // Make items more compact
        const statusItems = statusGrid.querySelectorAll('.status-item');
        statusItems.forEach(item => {
            item.style.padding = '8px';
        });
    }
    
    // Hide history panel to save space
    const historyPanel = statusColumn.querySelector('.history-panel');
    if (historyPanel) {
        historyPanel.style.display = 'none';
    }
}

function addMobileNavBar() {
    // Only add if it doesn't already exist
    if (document.querySelector('.mobile-nav')) return;
    
    const container = document.querySelector('.container');
    if (!container) return;
    
    const mobileNav = document.createElement('div');
    mobileNav.className = 'mobile-nav';
    mobileNav.innerHTML = `
        <button id="mobile-emergency-stop" class="mobile-nav-btn emergency">STOP</button>
        <button id="mobile-center" class="mobile-nav-btn center">CENTER</button>
        <button id="mobile-toggle-history" class="mobile-nav-btn toggle">HISTORY</button>
    `;
    container.appendChild(mobileNav);
    
    // Style the nav bar
    mobileNav.style.position = 'fixed';
    mobileNav.style.bottom = '0';
    mobileNav.style.left = '0';
    mobileNav.style.width = '100%';
    mobileNav.style.display = 'flex';
    mobileNav.style.justifyContent = 'space-around';
    mobileNav.style.padding = '10px';
    mobileNav.style.backgroundColor = '#1e272e';
    mobileNav.style.borderTop = '1px solid #444';
    mobileNav.style.zIndex = '1000';
    
    // Style buttons
    const navButtons = mobileNav.querySelectorAll('.mobile-nav-btn');
    navButtons.forEach(button => {
        button.style.padding = '15px';
        button.style.fontSize = '1.2rem';
        button.style.borderRadius = '8px';
        button.style.width = '30%';
        button.style.fontWeight = 'bold';
        button.style.border = 'none';
        button.style.boxShadow = '0 2px 4px rgba(0,0,0,0.3)';
    });
    
    // Set up button styles and functionality
    const stopBtn = document.getElementById('mobile-emergency-stop');
    if (stopBtn) {
        stopBtn.style.backgroundColor = '#e74c3c';
        stopBtn.style.color = 'white';
        stopBtn.addEventListener('click', () => sendCommand('stop'));
    }
    
    const centerBtn = document.getElementById('mobile-center');
    if (centerBtn) {
        centerBtn.style.backgroundColor = '#3498db';
        centerBtn.style.color = 'white';
        centerBtn.addEventListener('click', () => sendCommand('center'));
    }
    
    const historyBtn = document.getElementById('mobile-toggle-history');
    if (historyBtn) {
        historyBtn.style.backgroundColor = '#2c3e50';
        historyBtn.style.color = 'white';
        historyBtn.addEventListener('click', () => {
            const historyPanel = document.querySelector('.history-panel');
            if (historyPanel) {
                const isVisible = historyPanel.style.display !== 'none';
                historyPanel.style.display = isVisible ? 'none' : 'block';
                historyBtn.textContent = isVisible ? 'HISTORY' : 'HIDE';
            }
        });
    }
}

function addSwipeGestures(element) {
    // Only add once
    if (element.hasSwipeGestures) return;
    
    let touchStartX = 0;
    let touchStartY = 0;
    let lastDirection = null;
    let lastSteerValue = 0;
    
    element.addEventListener('touchstart', function(e) {
        touchStartX = e.touches[0].clientX;
        touchStartY = e.touches[0].clientY;
    });
    
    element.addEventListener('touchmove', function(e) {
        const touchX = e.touches[0].clientX;
        const touchY = e.touches[0].clientY;
        
        // Calculate distance moved
        const diffX = touchX - touchStartX;
        const diffY = touchY - touchStartY;
        
        // If horizontal movement is dominant, prevent scrolling
        if (Math.abs(diffX) > Math.abs(diffY)) {
            e.preventDefault();
            
            // Calculate steering angle based on how far the swipe moved
            const containerWidth = this.offsetWidth;
            const steerPercent = Math.min(Math.max(diffX / (containerWidth / 2), -1), 1);
            const steerAngle = Math.round(Math.abs(steerPercent) * selectedSteeringAngle);
            
            // Only send command if significant movement or direction change
            if (Math.abs(steerPercent) > 0.05 && 
                (Math.abs(steerPercent - lastSteerValue) > 0.05 ||
                 (steerPercent < 0 && lastDirection !== 'left') ||
                 (steerPercent > 0 && lastDirection !== 'right')
                )) {
                
                lastSteerValue = steerPercent;
                
                if (steerPercent < 0) {
                    sendCommand('left', steerAngle);
                    lastDirection = 'left';
    } else {
                    sendCommand('right', steerAngle);
                    lastDirection = 'right';
                }
                
                // Show visual feedback - create or update steering indicator
                showSteeringIndicator(element, steerPercent);
            }
        }
    });
    
    element.addEventListener('touchend', function() {
        // Center steering when touch ends
        sendCommand('center');
        lastDirection = null;
        lastSteerValue = 0;
        
        // Hide or reset indicator
        const indicator = document.getElementById('steer-indicator');
        if (indicator) {
            indicator.style.transform = 'translateX(0)';
            indicator.style.opacity = '0.3';
        }
    });
    
    // Mark as having gestures
    element.hasSwipeGestures = true;
}

function showSteeringIndicator(container, steerPercent) {
    let indicator = document.getElementById('steer-indicator');
    
    // Create indicator if it doesn't exist
    if (!indicator) {
        indicator = document.createElement('div');
        indicator.id = 'steer-indicator';
        
        // Style the indicator
        indicator.style.position = 'absolute';
        indicator.style.bottom = '20px';
        indicator.style.left = '50%';
        indicator.style.transform = 'translateX(0)';
        indicator.style.width = '50px';
        indicator.style.height = '50px';
        indicator.style.marginLeft = '-25px'; // Center the indicator
        indicator.style.borderRadius = '50%';
        indicator.style.backgroundColor = 'rgba(52, 152, 219, 0.3)';
        indicator.style.border = '2px solid #00a8ff';
        indicator.style.zIndex = '100';
        indicator.style.transition = 'transform 0.1s ease-out, opacity 0.2s';
        indicator.style.opacity = '0.3';
        
        container.appendChild(indicator);
    }
    
    // Update position and make more visible when active
    indicator.style.transform = `translateX(${steerPercent * 100}px)`;
    indicator.style.opacity = '0.8';
}

// Listen for orientation changes
window.addEventListener('orientationchange', function() {
    // Short delay to allow browser to finish rotating
    setTimeout(setupMobileLayout, 300);
});

// Update the IIFE at the bottom of the file to include mobile setup
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
    
    // Add mobile setup function to the initialization
    function initializeLayout() {
        setupFullscreenButton();
        setupEscapeKeyHandler();
        setupResizeButtons();
        setupMobileLayout(); // Initialize mobile layout
    }
    
    // Try to set up the layout immediately if the DOM is already loaded
    if (document.readyState === "complete" || document.readyState === "interactive") {
        initializeLayout();
    }
    
    // Also set it up on DOMContentLoaded to be safe
    document.addEventListener('DOMContentLoaded', function() {
        initializeLayout();
    });
    
    // And set it up on load as a final fallback
    window.addEventListener('load', function() {
        initializeLayout();
    });
})();

function setDriveMode(isForward) {
    const toggle = document.getElementById('drive-mode-toggle');
    if (isForward) {
        toggle.classList.remove('drive-mode-reverse');
        toggle.classList.add('drive-mode-forward');
        document.querySelector('.drive-mode-indicator').style.top = '5px';
        document.querySelector('.drive-mode-indicator').style.bottom = '';
    } else {
        toggle.classList.remove('drive-mode-forward');
        toggle.classList.add('drive-mode-reverse');
        document.querySelector('.drive-mode-indicator').style.bottom = '5px';
        document.querySelector('.drive-mode-indicator').style.top = '';
    }
}

// Update the layout function to account for the horizontal control bar
function adjustLayoutForControls() {
    const controlsHeight = document.querySelector('.controls-column').offsetHeight;
    document.querySelector('.main-content').style.paddingBottom = `${controlsHeight + 20}px`;
    
    // Make sure the status column doesn't overlap with controls
    const statusColumn = document.querySelector('.status-history-column');
    if (statusColumn) {
        statusColumn.style.marginBottom = `${controlsHeight + 20}px`;
    }
}

// Call this function when the page loads and on resize
document.addEventListener('DOMContentLoaded', adjustLayoutForControls);
window.addEventListener('resize', adjustLayoutForControls);
