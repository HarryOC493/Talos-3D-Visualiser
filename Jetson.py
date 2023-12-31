import serial
import socket

# Replace with the appropriate serial ports for your Arduino devices and their respective baud rates
ser1 = serial.Serial('/dev/ttyACM1', 9600)
ser3 = serial.Serial('/dev/ttyACM0', 9600)
ser2 = serial.Serial('/dev/ttyACM2', 9600)

# Replace with your MacBook's IP address and the chosen port number
macbook_ip = '192.168.0.26'
macbook_port = 50000

# Create a socket
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    # Connect to the MacBook
    sock.connect((macbook_ip, macbook_port))
    print(f"Connected to MacBook at {macbook_ip}:{macbook_port}")

    while True:
        try:
            debug = 1
            message = []
            left_contact, right_contact = None, None  # Initialize the contact variables

            # Read data from the joints
            positions = ser1.readline().decode('utf-8').strip()
            if positions:
                if debug:
                    print(positions)
                data = positions.replace('[', '').replace(']', '').split(',')
                # All elements except the last two are joint angles
                joint_angles_degrees = [float(angle) for angle in data[:-2]]
                # The second last element is the left contact binary
                left_contact = int(data[-2])
                # The last element is the right contact binary
                right_contact = int(data[-1])
                message.extend(joint_angles_degrees)
                # Add left and right contact at the end of the message
                message.append(left_contact)
                message.append(right_contact)

            # Read data from the imu
            angles = ser2.readline().decode('utf-8').strip()
            if angles:
                if debug:
                    print(angles)
                imu_angles_degrees = [float(angle) for angle in angles.replace('[', '').replace(']', '').split(',')]
                message.extend(imu_angles_degrees)

            # Read data from the distance sensor
            distance_str = ser3.readline().decode('utf-8').strip()
            if distance_str:
                if debug:
                    print(distance_str)
                distance = float(distance_str)
                message.append(distance)

            # Send the message to the MacBook
            serialized_message = ",".join(map(str, message))
            sock.sendall(serialized_message.encode('utf-8'))
            sock.sendall(b'\n')  # Add a newline to mark the end of each message

            print(message)

        except ValueError as e:
            print(f"Error converting data to float: {e}")

        except KeyboardInterrupt:
            # Close the socket when the script is interrupted
            print("Interrupt received, stopping...")
            break

        except Exception as e:
            # Handle other exceptions
            print(f"An unexpected error occurred: {e}")

finally:
    # Ensure the socket is closed even if an exception occurs
    print("Closing socket...")
    sock.close()