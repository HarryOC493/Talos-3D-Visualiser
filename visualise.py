import pybullet as p
import time
import math
import socket
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Initialize a list to store distance values
distance_values = []

# Connect to PyBullet
physicsClient = p.connect(p.GUI)
p.setGravity(0, 0, 0)
robot_urdf_path = "Assets/Talos_Lite.urdf"
robot_id = p.loadURDF(robot_urdf_path, [0, 0, 0], useFixedBase=True)
joint_indices = [0, 2, 4, 6, 8, 11]

for joint_index in joint_indices:
    p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=0, force=0)

macbook_port = 50000
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.bind(('0.0.0.0', macbook_port))
sock.listen()
print(f"Waiting for connections on port {macbook_port}")
conn, addr = sock.accept()
print(f"Connected to {addr}")

fig = plt.figure(figsize=(12, 8))  # Adjust the figure size as needed
ax_lin = fig.add_subplot(2, 2, 1, projection='3d')
ax_rot = fig.add_subplot(2, 2, 2, projection='3d')
ax_dist = fig.add_subplot(2, 2, 3)  # 2D plot for distance

ax_lin.set_title('Linear Acceleration')
ax_rot.set_title('Rotational Acceleration')
ax_dist.set_title('Talos Height')

# Define the maximum number of data points to display
MAX_WINDOW_SIZE = 50  # Adjust this number as needed

def update_plots(lin_acc, rot_acc, distance):
    global distance_values

    # Keep only the last MAX_WINDOW_SIZE values
    if len(distance_values) > MAX_WINDOW_SIZE:
        distance_values = distance_values[-MAX_WINDOW_SIZE:]

    ax_lin.cla()
    ax_rot.cla()
    ax_dist.cla()


    # Linear acceleration plot
    ax_lin.quiver(0, 0, 0, lin_acc[0], lin_acc[1], lin_acc[2], normalize=True)
    ax_lin.set_xlim([-1, 1])
    ax_lin.set_ylim([-1, 1])
    ax_lin.set_zlim([-1, 1])

    # Rotational acceleration plot
    ax_rot.quiver(0, 0, 0, rot_acc[0], rot_acc[1], rot_acc[2], normalize=True)
    ax_rot.set_xlim([-1, 1])
    ax_rot.set_ylim([-1, 1])
    ax_rot.set_zlim([-1, 1])

    # Distance plot
    distance_values.append(distance)
    # Update the distance plot
    ax_dist.stairs(distance_values, fill=True)
    # Set the y-axis limit dynamically based on the displayed data
    ax_dist.set_ylim([min(distance_values) - 1, max(distance_values) + 1])  

    plt.draw()
    plt.pause(0.001)



plt.ion()  # Turn on interactive mode
plt.show()

try:
    while True:
        data = conn.recv(1024)
        if not data:
            break

        received_message = data.decode('utf-8').strip()

        if not received_message:
            continue

        print("Received message:", received_message)

        # Split the received message and remove any empty strings or non-numeric values
        split_message = received_message.split(',')
        cleaned_values = []
        for item in split_message:
            item = item.strip()  # Remove whitespace
            if item.replace('.', '', 1).replace('-', '', 1).isdigit():
                cleaned_values.append(item)

        # Convert the cleaned values to floats
        try:
            values = [float(item) for item in cleaned_values]
        except ValueError as e:
            print("Error converting data to float:", e)
            continue

        # Check if the received message has all the necessary values
        if len(values) < 14:
            print("Incomplete data received. Skipping this iteration.")
            continue

        linear_acceleration = values[:3]
        rotational_acceleration = values[3:6]
        joint_positions = values[7:13]
        heading = values[6]
        distance = values[13]

        # Update the plots with the new acceleration data and distance
        update_plots(linear_acceleration, rotational_acceleration, distance)

        if joint_positions:
            joint_angles_radians = [math.radians(angle) for angle in joint_positions]
            for joint_index, angle_rad in zip(joint_indices, joint_angles_radians):
                p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=angle_rad, force=500)
            p.stepSimulation()

except KeyboardInterrupt:
    pass

finally:
    p.disconnect()
    conn.close()
    sock.close()
