import serial

# Replace with the appropriate serial ports for your Arduino devices and their respective baud rates
ser1 = serial.Serial('/dev/ttyACM0', 9600)
ser2 = serial.Serial('/dev/ttyACM1', 9600)

try:
    while True:
        try:
            # Read data from the first joints
            positions = ser1.readline().decode('utf-8').strip()
            print("Received data from Arduino 1:", positions)

            joint_angles_degrees = [float(angle) for angle in positions.replace('[', '').replace(']', '').split(',')]

            print(joint_angles_degrees)

            # Read data from the second imu
            angles = ser2.readline().decode('utf-8').strip()
            print("Received data from Arduino 2:", angles)

            

        except KeyboardInterrupt:
            # Close the serial connections when the script is interrupted
            ser1.close()
            ser2.close()
            break

finally:
    # Ensure serial connections are closed even if an exception occurs
    ser1.close()
    ser2.close()
