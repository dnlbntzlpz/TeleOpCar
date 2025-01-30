function sendCommand(action) {
    console.log(`Sending command: ${action}`);
    // TODO: Use AJAX or WebSocket to send the command to the backend
}

function updateTelemetry(speed, battery, distance) {
    document.getElementById("speed").innerText = speed;
    document.getElementById("battery").innerText = battery;
    document.getElementById("distance").innerText = distance;
}

// Simulate telemetry updates for testing
setInterval(() => {
    updateTelemetry(
        Math.floor(Math.random() * 100),
        Math.floor(Math.random() * 100),
        Math.floor(Math.random() * 500)
    );
}, 2000);
