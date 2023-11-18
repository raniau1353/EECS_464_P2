import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import math
from ntpath import join


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


# normal to [1, 1, 0]
# todo 

# normal to [1, 0 , 1]
# todo
 
#normal to [0, 1, 1]
# todo 


#normal to [1, 1, 1]
# todo 