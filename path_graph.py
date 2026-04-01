import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt("path_points.csv", delimiter=",", skip_header=1)
x_coords = data[:, 0]
y_coords = data[:, 1]

# Draw it
plt.style.use('dark_background')
plt.figure()
plt.plot(x_coords, y_coords, color="#98FBCB")
plt.axis('equal')
plt.grid(True, which='both', linewidth=0.5, color='gray')
plt.title("Path")
plt.xlabel("X [m]")
plt.ylabel("Y [m]")

# Save it
plt.savefig("assets/standard_path.png", facecolor=plt.gcf().get_facecolor(), bbox_inches='tight')

# Show it
plt.show()

