import numpy as np
import quaternion
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d

def from_axis(theta, n):
    c = np.cos(theta/2)
    s = np.sin(theta/2)
    nn = np.linalg.norm(n)
    return np.quaternion(c, s*n[0]/nn, s*n[1]/nn, s*n[2]/nn)

def to_axis(q):
    d = np.sqrt(q.x*q.x + q.y*q.y+ q.z*q.z)
    n = np.array([q.x, q.y, q.z]) / d
    theta = 2.0 * np.arctan2(d, q.w)
    return theta, n

def random_orientation(theta_min, theta_max):
    x = np.random.random()
    y = np.random.random()
    z = np.random.random()
    d = np.sqrt(x*x + y*y + z*z)
    theta = theta_min + (theta_max - theta_min)*np.random.random()
    return from_axis(theta, np.array([x/d, y/d, z/d]))

"""
we have a known orientation quaternion of the rocket in the world frame
we know the rocket frame's x axis orientation
we would like to know the angle that the rocket has to rotate around this axis such that the y axis lies in the world (x,y) plane again
and the same for the x axis rotating around the y axis to lie in the (x,y) plane again
1. rotate point (1,0,0) with orientation quaternion p' = q p q⁻¹
2. determine angle to z-axis as alpha = arccos(p'.z)
    can be linearized around pi/2 (cos crosses 0 here) using
        arccos(x) ~= pi/2 - x
3. angle to (x,y) plane is alpha - pi/2 (positive result below plane, negative result above)
    after linearization the angle is directly
        -x 
"""

def plot_world_coord(ax):
    axes = np.array([[[0,1],[0,0],[0,0]],
                     [[0,0],[0,1],[0,0]],
                     [[0,0],[0,0],[0,1]]])
    colors = ["blue", "green", "red"]
    for i in range(3):
        ax.plot(axes[i, 0, :], axes[i, 1, :], axes[i, 2, :], color=colors[i])

def plot_rocket_coord(ax, axes):
    for i in range(3):
        ax.plot(axes[i, 0, :], axes[i, 1, :], axes[i, 2, :], color="black", lw=2)

fig = plt.figure(figsize=(12,12))
ax = fig.add_subplot(111, projection='3d')

plot_world_coord(ax)

r = random_orientation(-0.26, 0.26)
x = r * np.quaternion(0, 1, 0, 0) * r.conj()
y = r * np.quaternion(0, 0, 1, 0) * r.conj()
z = r * np.quaternion(0, 0, 0, 1) * r.conj()
axes = np.array([[[0,x.x],[0,x.y],[0,x.z]],
                 [[0,y.x],[0,y.y],[0,y.z]],
                 [[0,z.x],[0,z.y],[0,z.z]]])
plot_rocket_coord(ax, axes)

a_x = x.z * 180 / np.pi
a_y = y.z * 180 / np.pi
ax.text((1.0+x.x)/2, (0.0+x.y)/2, (0.0+x.z)/2, "{:1.1f}°".format(a_x), None)
ax.text((0.0+y.x)/2, (1.0+y.y)/2, (0.0+y.z)/2, "{:1.1f}°".format(a_y), None)

plt.show()
