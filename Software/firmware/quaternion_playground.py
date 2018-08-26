import numpy as np
import quaternion

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

def random_orientation():
    x = np.random.random()
    y = np.random.random()
    z = np.random.random()
    d = np.sqrt(x*x + y*y + z*z)
    theta = np.pi * (-1.0 + 2.0*np.random.random())
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

