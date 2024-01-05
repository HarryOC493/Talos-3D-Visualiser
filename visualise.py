from re import split
from turtle import left, right
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

# Adjusted subplot positions in a 3x3 grid
ax_lin = fig.add_subplot(3, 3, 1, projection='3d')
ax_rot = fig.add_subplot(3, 3, 2, projection='3d')
ax_dist = fig.add_subplot(3, 3, 4)  # 2D plot for distance
ax_heading = fig.add_subplot(3, 3, 5, projection='polar')  # Polar plot for heading
ax_left_foot = fig.add_subplot(3, 3, 7)  # Subplot for left foot contact
ax_right_foot = fig.add_subplot(3, 3, 8)  # Subplot for right foot contact


ax_lin.set_title('Linear Acceleration')
ax_rot.set_title('Rotational Acceleration')
ax_dist.set_title('Talos Height')
ax_heading.set_title('Heading')
ax_left_foot.set_title('Left Foot Contact')
ax_right_foot.set_title('Right Foot Contact')

for ax in [ax_left_foot, ax_right_foot]:
    ax.set_xticks([])
    ax.set_yticks([])

# Define the maximum number of data points to display
MAX_WINDOW_SIZE = 50  # Adjust this number as needed
# Define your desired and extreme values
DESIRED_DISTANCE = 38  # For example, 40 cm
MIN_DISTANCE = 36      # Minimum acceptable distance, e.g., 35 cm
MAX_DISTANCE = 41      # Maximum acceptable distance, e.g., 45 cm

def update_plots(lin_acc, rot_acc, distance, heading, left_contact, right_contact):
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
    # Add lines for desired and extreme values
    ax_dist.axhline(DESIRED_DISTANCE, color='green', linestyle='-', linewidth=2)
    ax_dist.axhline(MIN_DISTANCE, color='red', linestyle='--', linewidth=2)
    ax_dist.axhline(MAX_DISTANCE, color='red', linestyle='--', linewidth=2)

     # Heading plot
    ax_heading.cla()
    ax_heading.set_theta_zero_location('N')  # Set North as the zero direction
    ax_heading.set_theta_direction(-1)  # Clockwise
    ax_heading.plot([0, math.radians(heading)], [0, 1], color='blue')  # Draw heading line
    ax_heading.set_rmax(1)  # Set max radius
    ax_heading.grid(True)

    ax_left_foot.cla()
    ax_right_foot.cla()
    ax_left_foot.add_patch(plt.Circle((0.5, 0.5), 0.4, color='green' if left_contact else 'red'))
    ax_right_foot.add_patch(plt.Circle((0.5, 0.5), 0.4, color='green' if right_contact else 'red'))

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

        # Split the received message and convert to float values
        split_message = [float(val) for val in received_message.split(',') if val]

        # Check if the received message has all the necessary values
        if len(split_message) < 16:
            print("Incomplete data received. Skipping this iteration.")
            continue

        # Extract the data from the split_message list
        linear_acceleration = split_message[:3]
        print(linear_acceleration)
        rotational_acceleration = split_message[3:6]
        print(rotational_acceleration)
        joint_positions = split_message[7:13]
        print(joint_positions)
        joint_velocities = split_message[14:20]
        heading = split_message[6]
        print(heading)
        distance = split_message[22]
        print(distance)
        left_contact = int(split_message[20])
        print(left_contact)
        right_contact = int(split_message[21])
        print(right_contact)

        if joint_positions:
            print(joint_positions)
            joint_angles_radians = [math.radians(angle) for angle in joint_positions]

            for joint_index, angle_rad in zip(joint_indices, joint_angles_radians):
                p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=angle_rad, force=500)
            p.stepSimulation()
            print("Stepped Simulation")

        # Update the plots with the new data
        update_plots(linear_acceleration, rotational_acceleration, distance, heading, left_contact, right_contact)
        

except KeyboardInterrupt:
    pass

finally:
    p.disconnect()
    conn.close()
    sock.close()