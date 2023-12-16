import serial

# Replace 'COM3' with the appropriate serial port for your Arduino
ser = serial.Serial('/dev/cu.usbmodem1101', 9600)

while True:
    try:
        # Read the serial data
        serial_data = ser.readline().decode('utf-8').strip()

        # Print the received data
        print("Received data:", serial_data)

    except KeyboardInterrupt:
        # Close the serial connection when the script is interrupted
        ser.close()
        break
