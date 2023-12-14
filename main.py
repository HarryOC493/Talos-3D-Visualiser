import pybullet as p
import time
import serial

# Connect to Arduino over serial
arduino_port = '/dev/ttyUSB0'  # Change this to your Arduino port
arduino_baudrate = 9600
arduino_serial = serial.Serial(arduino_port, arduino_baudrate, timeout=1)

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-visual mode
p.setGravity(0, 0, -9.81)

# Load URDF model
robot_urdf_path = "Talos_Lite.urdf"
robot_id = p.loadURDF(robot_urdf_path, [0, 0, 0], useFixedBase=True)

# Main loop
try:
    while True:
        # Read joint angles from Arduino over serial
        angles_str = arduino_serial.readline().decode().strip()
        joint_angles = [float(angle) for angle in angles_str.split(',')]

        # Set joint angles in PyBullet
        for joint_index, angle in enumerate(joint_angles):
            p.resetJointState(robot_id, joint_index, angle)

        # Step the simulation
        p.stepSimulation()

        # Optional: Add a delay to control the update rate
        time.sleep(0.02)  # Adjust as needed

except KeyboardInterrupt:
    pass

finally:
    # Clean up
    p.disconnect()
    arduino_serial.close()
