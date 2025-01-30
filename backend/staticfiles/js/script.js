const socket = new WebSocket('ws://192.168.10.183:8000/ws/control/');

socket.onopen = () => {
    console.log("WebSocket connection established.");
};

socket.onmessage = (event) => {
    const data = JSON.parse(event.data);
    console.log("Received from server:", data);
};

socket.onclose = () => {
    console.log("WebSocket connection closed.");
};

function sendCommand(action, speed = 1.0) {
    if (socket.readyState === WebSocket.OPEN) {
        socket.send(JSON.stringify({ action, speed }));
        console.log("Command sent:", action, speed);
    } else {
        console.error("WebSocket is not open.");
    }
}
