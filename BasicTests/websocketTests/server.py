import socket

# Server configuration
HOST = "0.0.0.0"  # Listen on all network interfaces
PORT = 8765       # Port for the TCP server

# Create a socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((HOST, PORT))
server_socket.listen(1)  # Allow 1 client to connect

print(f"Server started on {HOST}:{PORT}, waiting for a connection...")

try:
    while True:
        conn, addr = server_socket.accept()  # Accept incoming connection
        print(f"Connected by {addr}")
        with conn:
            while True:
                data = conn.recv(1024)  # Receive up to 1024 bytes
                if not data:
                    break
                print(f"Received: {data.decode()}")

                # Send a response back to the client
                response = "Message received"
                conn.sendall(response.encode())
except KeyboardInterrupt:
    print("\nServer shutting down.")
finally:
    server_socket.close()
