import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt("Reverse-Manoeuvre/path_points.csv", delimiter=",", skip_header=1)
x_coords = data[:, 0]
y_coords = data[:, 1]

# Draw it
plt.figure(figsize=(10, 6))
plt.plot(x_coords, y_coords, 'b-')
plt.axis('equal')
plt.grid(True)
plt.title("Path of the Cart")
plt.show()