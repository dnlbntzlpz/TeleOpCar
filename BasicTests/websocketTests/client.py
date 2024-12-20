import socket

# Server configuration
SERVER_IP = "192.168.10.183"  # Replace with your Raspberry Pi's IP
PORT = 8765                   # Port to connect to

# Create a socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

try:
    print(f"Connecting to {SERVER_IP}:{PORT}...")
    client_socket.connect((SERVER_IP, PORT))
    print("Connected to the server.")

    # Example: Send a message
    message = "Hello from the client!"
    client_socket.sendall(message.encode())
    print(f"Sent: {message}")

    # Receive a response
    response = client_socket.recv(1024)
    print(f"Received: {response.decode()}")
finally:
    print("Closing the connection.")
    client_socket.close()
