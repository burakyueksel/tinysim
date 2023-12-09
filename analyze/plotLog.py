import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Read data from log.txt
with open('log.txt', 'r') as file:
    lines = file.readlines()

# Parse data
timestamps = []
x_positions = []
y_positions = []
z_positions = []

for line in lines:
    data = line.split()
    timestamps.append(float(data[0]))
    x_positions.append(float(data[1]))
    y_positions.append(float(data[2]))
    z_positions.append(float(data[3]))

every_xth = 20

# Select every xth data point
timestamps_red = timestamps[::every_xth]
x_positions_red = x_positions[::every_xth]
y_positions_red = y_positions[::every_xth]
z_positions_red = z_positions[::every_xth]


# Create subplots
fig, axs = plt.subplots(2, 2, figsize=(12, 10))

# Subplot 1: 3D position plot
axs[0, 0] = fig.add_subplot(221, projection='3d')
axs[0, 0].plot(x_positions_red, y_positions_red, z_positions_red, marker='o')
axs[0, 0].set_title('3D Position Plot')
axs[0, 0].set_xlabel('X Position [m]')
axs[0, 0].set_ylabel('Y Position [m]')
axs[0, 0].set_zlabel('Z Position [m]')

# Subplot 2: x position vs. time
axs[0, 1].plot(timestamps_red, x_positions_red, marker='o')
axs[0, 1].set_title('X Position vs Time')
axs[0, 1].set_xlabel('Time [s]')
axs[0, 1].set_ylabel('X Position [m]')
axs[0, 1].grid(True)

# Subplot 3: y position vs. time
axs[1, 0].plot(timestamps_red, y_positions_red, marker='o')
axs[1, 0].set_title('Y Position vs Time')
axs[1, 0].set_xlabel('Time [s]')
axs[1, 0].set_ylabel('Y Position [m]')
axs[1, 0].grid(True)

# Subplot 4: z position vs. time
axs[1, 1].plot(timestamps_red, z_positions_red, marker='o')
axs[1, 1].set_title('Z Position vs Time')
axs[1, 1].set_xlabel('Time [s]')
axs[1, 1].set_ylabel('Z Position [m]')
axs[1, 1].grid(True)

# Adjust layout to prevent overlap
plt.tight_layout()

plt.show()
