import socket

# Replace with the chosen port number for communication
macbook_port = 50000

# Create a socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to a specific address and port
sock.bind(('0.0.0.0', macbook_port))

# Listen for incoming connections
sock.listen()

print(f"Waiting for connections on port {macbook_port}")

# Accept a connection
conn, addr = sock.accept()
print(f"Connected to {addr}")

try:
    while True:
        # Receive data
        data = conn.recv(1024)
        if not data:
            break

        # Process the received data (parse as needed)
        received_message = data.decode('utf-8').strip()  # Remove leading/trailing whitespaces

        # Skip empty lines
        if not received_message:
            continue

        print("Received message:", received_message)

        # Convert the received string into a list of floats
        values = [float(item) for item in received_message.split(',')]

        # Extract specific values
        linear_acceleration = values[:3]
        rotational_acceleration = values[3:6]
        joint_positions = values[7:]
        heading = values[6]

        # Print the extracted values
        print("Linear Acceleration:", linear_acceleration)
        print("Rotational Acceleration:", rotational_acceleration)
        print("Joint Positions:", joint_positions)
        print("Heading: ", heading)

except KeyboardInterrupt:
    pass

finally:
    # Close the connection and the socket
    conn.close()
    sock.close()
