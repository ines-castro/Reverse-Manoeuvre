
import matplotlib.pyplot as plt
import numpy as np

# Read the custom CSV format: initial, target, separator, then path
with open("path_case2.csv") as f:
    lines = f.readlines()

initial = [float(x) for x in lines[0].strip().split(",")]
target = [float(x) for x in lines[1].strip().split(",")]
# Find separator
sep_idx = next(i for i, l in enumerate(lines) if l.strip() == ",,")
path_data = np.genfromtxt(lines[sep_idx+1:], delimiter=",")

x_coords = path_data[:, 0]
y_coords = path_data[:, 1]

# Draw it
plt.style.use('dark_background')
plt.figure()
plt.plot(x_coords, y_coords, color="#74D3AE", linewidth=1.5, label="Path", zorder=4)


plt.scatter(x_coords[0], y_coords[0], color="#DD9787", s=60, marker='D', label="Start", zorder=5)
plt.scatter(x_coords[-1], y_coords[-1], color="#BAA5FF", s=60, marker='D', label="End", zorder=5)

arrow_length = 0.9
plt.quiver(
    initial[0], initial[1],
    np.cos(initial[2]) * arrow_length, np.sin(initial[2]) * arrow_length,
    angles='xy', scale_units='xy', scale=0.9, color="#DD9787", width=0.01, zorder=6
)

plt.quiver(
    target[0], target[1],
    np.cos(target[2]) * arrow_length, np.sin(target[2]) * arrow_length,
    angles='xy', scale_units='xy', scale=1, color="#BAA5FF", width=0.01, zorder=6
)

plt.axis('equal')
plt.grid(True, which='both', linewidth=0.5, color='gray')
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.legend(loc="center left", bbox_to_anchor=(1, 0.5))

# Save it
plt.savefig("testing_case.png", facecolor=plt.gcf().get_facecolor(), bbox_inches='tight')

# Show it
plt.show()

