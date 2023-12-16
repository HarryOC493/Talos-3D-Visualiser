import pybullet as p
import time
import serial
import math

ser = serial.Serial('/dev/cu.usbmodem1101', 9600)

# Connect to PyBullet
physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-visual mode
p.setGravity(0, 0, 0)

# Load URDF model
robot_urdf_path = "Talos_Lite.urdf"
robot_id = p.loadURDF(robot_urdf_path, [0, 0, 0], useFixedBase=True)

# Define joint indices
joint_indices = [0, 2, 4, 6, 8, 11]  # Adjust based on your robot's joint order

# Set joint control mode and gains
for joint_index in joint_indices:
    p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=0, force=0)

# Main loop
try:
    while True:
        # Read joint angles from Arduino over serial
        angles_str = ser.readline().decode('utf-8').strip()
        print(angles_str)

        # Check if the received data is not empty
        if angles_str:
            try:
                # Convert string to a list of floats (degrees)
                joint_angles_degrees = [float(angle) for angle in angles_str.replace('[', '').replace(']', '').split(',')]

                # Convert degrees to radians
                joint_angles_radians = [math.radians(angle) for angle in joint_angles_degrees]

                # Set joint angles using setJointMotorControl2
                for joint_index, angle_rad in zip(joint_indices, joint_angles_radians):
                    p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=angle_rad, force=500)

                # Step the simulation
                p.stepSimulation()

            except ValueError as e:
                print(f"Error converting data to float: {e}")

except KeyboardInterrupt:
    pass

finally:
    # Clean up
    p.disconnect()
    ser.close()
