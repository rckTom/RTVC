import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt
import math

plt.figure(figsize=(10,10))

MMT_RADIUS = 10.0
EXTEND_MMT = 2.0
WIRE_LENGTH = 12.0
SERVO_ARM_LENGTH = 12.0
DESIRED_RANGE = 6.2
MAX_ANGLE = 45.0 / 180.0 * math.pi
CORRECTION = 1.3

# servo 0
x0 = -WIRE_LENGTH + CORRECTION                                  # position
y0 =  MMT_RADIUS + EXTEND_MMT - SERVO_ARM_LENGTH + CORRECTION   # position
l0 =  SERVO_ARM_LENGTH                                          # arm length

# attachment 0
a0 =  WIRE_LENGTH                                               # attachment length

# servo 1
x1 =  MMT_RADIUS + EXTEND_MMT - SERVO_ARM_LENGTH + CORRECTION   # position
y1 = -WIRE_LENGTH + CORRECTION                                  # position
l1 =  SERVO_ARM_LENGTH                                          # arm length

# attachment 1
a1 =  WIRE_LENGTH                                               # attachment length

# distance between attachments 0 and 1
dx =  MMT_RADIUS + EXTEND_MMT                                   # attachment size x
dy =  MMT_RADIUS + EXTEND_MMT                                   # attachment size y

# parameters for error function
# alpha0  # arm angle
# alpha1  # arm angle

# error function equations
# xa1 - xa0 = dx
# ya1 - ya0 = dy
# (xa0 - xs0)^2 + (ya0 - ys0)^2 = a0^2
# (xa1 - xs1)^2 + (ya1 - ys1)^2 = a1^2
# xs0 = x0 + l0 * sin(alpha0)
# ys0 = y0 + l0 * cos(alpha0)
# xs1 = x1 + l1 * cos(alpha1)
# ys1 = y1 + l1 * sin(alpha1)

def err(a, cx, cy):
    alpha0, alpha1 = a
    xa1 = cx + dx
    xa0 = cx
    ya1 = cy
    ya0 = cy + dy
    xs0 = x0 + l0 * math.sin(alpha0)
    ys0 = y0 + l0 * math.cos(alpha0)
    xs1 = x1 + l1 * math.cos(alpha1)
    ys1 = y1 + l1 * math.sin(alpha1)
    e0 = (xa0 - xs0)**2.0 + (ya0 - ys0)**2.0 - a0**2.0
    e1 = (xa1 - xs1)**2.0 + (ya1 - ys1)**2.0 - a1**2.0
    return abs(e0) + abs(e1)

angles = (-0.1, -0.1)

coords = []
for i in range(-10, 11, 1):
    coords.append((-DESIRED_RANGE, i*DESIRED_RANGE/10.0))
    coords.append(( 0.0,           i*DESIRED_RANGE/10.0))
    coords.append(( DESIRED_RANGE, i*DESIRED_RANGE/10.0))
    # coords.append((i*DESIRED_RANGE/10.0, -DESIRED_RANGE))
    # coords.append((i*DESIRED_RANGE/10.0,  0.0))
    # coords.append((i*DESIRED_RANGE/10.0,  DESIRED_RANGE))

for c in coords:
    c0 = c[0]
    c1 = c[1]
    res = minimize(err, angles, args=(c0, c1), method="SLSQP", bounds=[(-MAX_ANGLE, MAX_ANGLE), (-MAX_ANGLE, MAX_ANGLE)])
    alpha0 = res.x[0]
    alpha1 = res.x[1]
    xs0 = x0 + l0 * math.sin(alpha0)
    ys0 = y0 + l0 * math.cos(alpha0)
    xs1 = x1 + l1 * math.cos(alpha1)
    ys1 = y1 + l1 * math.sin(alpha1)
    cx = c0
    cy = c1
    xa0 = cx
    ya0 = cy + dy
    xa1 = cx + dx
    ya1 = cy
    d0 = math.sqrt((xs0-xa0)**2.0 + (ys0-ya0)**2.0)
    d1 = math.sqrt((xs1-xa1)**2.0 + (ys1-ya1)**2.0)
    if abs(12.0-d0) > 0.01 or abs(12.0-d1) > 0.01:
        marker = 'x'
        size = 8
    else:
        marker = 'o'
        size = 1
    plt.scatter(xs0, -ys0, s=2*size, c='red', marker=marker)
    plt.scatter(xs1, -ys1, s=2*size, c='red', marker=marker)
    plt.scatter(cx, -cy, s=2*size, c='green', marker=marker)
    plt.scatter(xa0, -ya0, s=1*size, c='blue', marker=marker)
    plt.scatter(xa1, -ya1, s=1*size, c='blue', marker=marker)
    plt.plot([xs0, xa0], [-ys0, -ya0], color='gray', linewidth=0.5)
    plt.plot([xs1, xa1], [-ys1, -ya1], color='gray', linewidth=0.5)
    print("d0:", d0)
    print("d1:", d1)

for i in range(360):
    x = 24.4 * math.cos(i / 180.0 * math.pi)
    y = 24.4 * math.sin(i / 180.0 * math.pi)
    plt.scatter(x, y, s=1, c='gray')

plt.axes().set_aspect('equal', 'datalim')
plt.show()