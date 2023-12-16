import serial

# Replace '/dev/ttyACM0' with the correct serial port for your device
serial_port = '/dev/ttyACM0'
baud_rate = 9600

# Create a serial object
ser = serial.Serial(serial_port, baud_rate)

try:
    while True:
        # Read a line from the serial port
        line = ser.readline().decode('utf-8').strip()
        
        # Print the received data
        print("Received:", line)

except KeyboardInterrupt:
    print("Script interrupted.")

finally:
    # Close the serial port when done
    ser.close()
