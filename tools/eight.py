import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")

width = 0.1
shift = 0.2
points_number = 100
t = np.linspace(0, 2 * np.pi, points_number)  # Parameter t from 0 to 2¦Р
x = [shift] * points_number
# y = width * np.sin(2 * t) + shift
y = width * np.sin(2 * t)
z = width * np.sin(t) + shift

ax.plot(x, y, z)
plt.show()
