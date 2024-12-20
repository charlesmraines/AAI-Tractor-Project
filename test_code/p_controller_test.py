import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Simulation parameters
TIME_STEP = 0.1  # Time step for simulation in seconds
SIMULATION_TIME = 20  # Total simulation time in seconds
LINEAR_GAIN = 0.07
ANGULAR_GAIN = 1.0
goal = np.array([5.0, 5.0])  # Goal position (ENU coordinates)

# Initial robot state
position = np.array([0.0, 0.0])  # Initial position
heading = 0.0  # Initial heading in degrees

# Set up the plot
fig, ax = plt.subplots(figsize=(8, 6))
ax.set_xlim(-1, 6)
ax.set_ylim(-1, 6)
ax.set_xlabel("East (m)")
ax.set_ylabel("North (m)")
ax.set_title("Proportional Controller Simulation")
ax.grid()
ax.scatter(goal[0], goal[1], color="red", label="Goal", s=100, marker="X")
robot_traj, = ax.plot([], [], "b-", label="Trajectory")
heading_quiver = ax.quiver(0, 0, 0, 0, angles="xy", scale_units="xy", scale=1, color="blue", label="Heading")
ax.legend()

# Data storage for trajectory
trajectory = [position.copy()]

def update(frame):
    global position, heading

    # Calculate distance and angle to the goal
    delta_pos = np.linalg.norm(goal - position)
    desired_theta = np.degrees(np.arctan2(goal[1] - position[1], goal[0] - position[0]))
    
    # Calculate heading error
    heading_error = desired_theta - heading
    heading_error = (heading_error + 180) % 360 - 180  # Normalize to [-180, 180]
    
    # Proportional control
    if delta_pos > 0.1:  # Stop when close to goal
        linear_vel = LINEAR_GAIN * delta_pos
        angular_vel = ANGULAR_GAIN * heading_error

        # Clamp velocities
        linear_vel = np.clip(linear_vel, -5.0, 5.0)
        angular_vel = np.clip(angular_vel, -5.0, 5.0)

        # Update heading and position
        heading += angular_vel * TIME_STEP
        heading = heading % 360  # Normalize heading to [0, 360)
        position[0] += linear_vel * np.cos(np.radians(heading)) * TIME_STEP
        position[1] += linear_vel * np.sin(np.radians(heading)) * TIME_STEP

        # Update trajectory
        trajectory.append(position.copy())
    
    # Update plot
    traj_np = np.array(trajectory)
    robot_traj.set_data(traj_np[:, 0], traj_np[:, 1])
    heading_quiver.set_offsets([position[0], position[1]])
    heading_quiver.set_UVC(np.cos(np.radians(heading)), np.sin(np.radians(heading)))

    return robot_traj, heading_quiver

# Animate the simulation
anim = FuncAnimation(fig, update, frames=int(SIMULATION_TIME / TIME_STEP), interval=TIME_STEP * 1000, blit=False)

plt.show()
