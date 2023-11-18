import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from ntpath import join

'''
# normal to [1, 0, 0]
X, Y = np.meshgrid(np.arange(-6, 6), np.arange(-6, 6))
#Z = 0*X
orig_dist_x = 3
plane_offset_x = 0.2
Z = -orig_dist_x - plane_offset_x

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.plot_surface(X, Y, Z, alpha=0.5)  # the horizontal plane
ax.plot_surface(Z, Y, X, alpha=0.5)  # the vertical plane


# normal to [0, 0, 1]
plane_offset_z = 3

points = [[1, 2, plane_offset_z],
           [5 , 8 , plane_offset_z],
           [12, 3, plane_offset_z]]

p0, p1, p2 = points
x0, y0, z0 = p0
x1, y1, z1 = p1
x2, y2, z2 = p2

ux, uy, uz = u = [x1-x0, y1-y0, z1-z0]
vx, vy, vz = v = [x2-x0, y2-y0, z2-z0]

u_cross_v = [uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx]

point  = np.array(p0)
normal = np.array(u_cross_v)
print(normal)

d = -point.dot(normal)

xx, yy = np.meshgrid(range(10), range(10))

z = (-normal[0] * xx - normal[1] * yy - d) * 1. / normal[2]

# plot the surface
fig = plt.figure()
plt3d = fig.add_subplot(projection='3d')
plt3d.plot_surface(xx, yy, z)
plt.show()


# normal to [1, 0, 0], normal to [1, 0, 1]
X, Y = np.meshgrid(np.arange(-6, 6), np.arange(-6, 6))
#Z = 0*X
plane_offset_y = 0.2
Z = - plane_offset_y

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#ax.plot_surface(X, Y, Z, alpha=0.5)  # the horizontal plane
#ax.plot_surface(Z, Y, X, alpha=0.5)  # the vertical plane
ax.plot_surface(X, Z, Y, alpha=0.5)  # the vertical plane
plt.show()
'''
# 21.59cm * 27.94cm
#############################################
# Define a plane equation: ax + by + cz + d = 0
# For example, let's use the equation of the xy-plane: z = 0
a, b, c, d = 0, 0, 1, 0

# Create a grid of points in the xy-plane
x = np.linspace(-10, 10, 100)
y = np.linspace(-10, 10, 100)
x, y = np.meshgrid(x, y)

# Calculate the corresponding z values for each point on the grid
plane1 = -0.5*x - 0.5*y
plane2 = 0*x - y 
plane3 = 0*x + y
plane4 = -x + 0*y
plane5 = 0* x + 0*y

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot the planes
ax.plot_surface(x, y, plane1, alpha=0.5, rstride=100, cstride=100, color='r', label='Plane 1')
ax.plot_surface(x, y, plane2, alpha=0.5, facecolors='g', rstride=100, cstride=100, label='Plane 2')
ax.plot_surface(x, y, plane3, alpha=0.5, facecolors='b', rstride=100, cstride=100, label='Plane 3')
ax.plot_surface(x, y, plane4, alpha=0.5, facecolors='m', rstride=100, cstride=100, label='Plane 4')
ax.plot_surface(x, y, plane5, alpha=0.5, facecolors='c', rstride=100, cstride=100, label='Plane 5')
ax.plot_surface(plane5, x, y, alpha=0.5, facecolors='c', rstride=100, cstride=100, label='Plane 6')
ax.plot_surface(y, plane5, x, alpha=0.5, facecolors='c', rstride=100, cstride=100, label='Plane 7')
# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show the plot
plt.show()

# normal to [1, 1, 0]
# todo 

# normal to [1, 0 , 1]
# todo
 
#normal to [0, 1, 1]
# todo 


#normal to [1, 1, 1]
# todo 