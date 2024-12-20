import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd
import numpy as np

# Load the data using pandas
filename = "tractor_data_log.csv"
data = pd.read_csv(filename)

# Ensure columns are interpreted correctly
print("Column headers in CSV:", list(data.columns))

# Extract columns
time = data["time"].values
x = data["x"].values
y = data["y"].values
heading = data["heading"].values

# Set up the figure and axis
fig, ax = plt.subplots()
ax.set_xlim(x.min() - 1, x.max() + 1)
ax.set_ylim(y.min() - 1, y.max() + 1)
ax.set_aspect('equal', adjustable='datalim')

# Plot elements
trajectory_line, = ax.plot([], [], 'b-', label="Trajectory")
current_position, = ax.plot([], [], 'ro', label="Current Position")
heading_quiver = ax.quiver([], [], [], [], angles='xy', scale_units='xy', scale=1, color='r')
goal_point, = ax.plot(5, 5, 'go', label="Goal")  # Goal point at (5, 5)

# Initialize the plot elements
def init():
    trajectory_line.set_data([], [])
    current_position.set_data([], [])
    heading_quiver.set_offsets(np.array([[], []]).T)
    heading_quiver.set_UVC([], [])
    return trajectory_line, current_position, heading_quiver

# Update function for the animation
def update(frame):
    trajectory_line.set_data(x[:frame], y[:frame])
    current_position.set_data(x[frame - 1], y[frame - 1])
    heading_quiver.set_offsets(np.array([[x[frame - 1], y[frame - 1]]]))
    heading_quiver.set_UVC([np.cos(np.radians(heading[frame - 1]))],
                           [np.sin(np.radians(heading[frame - 1]))])
    return trajectory_line, current_position, heading_quiver

# Create the animation
ani = animation.FuncAnimation(fig, update, frames=len(time), init_func=init, blit=True, interval=50)

# Display the animation
plt.legend()
plt.show()
