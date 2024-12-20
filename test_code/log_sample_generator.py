import csv
import numpy as np

# Create sample test data for the trajectory log file
filename = "tractor_data_log.csv"
time = np.linspace(0, 10, 100)  # 100 timesteps over 10 seconds
x = 5 * np.sin(time * 0.5)  # Example trajectory: sinusoidal eastward motion
y = 5 * np.cos(time * 0.5)  # Example trajectory: sinusoidal northward motion
heading = np.degrees(np.arctan2(np.gradient(y), np.gradient(x)))  # Heading angle

# Write to CSV file
with open(filename, mode='w', newline='') as file:
    writer = csv.DictWriter(file, fieldnames=["time", "x", "y", "heading"])
    writer.writeheader()
    for t, x_val, y_val, h in zip(time, x, y, heading):
        writer.writerow({"time": t, "x": x_val, "y": y_val, "heading": h})