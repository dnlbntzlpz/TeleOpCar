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
