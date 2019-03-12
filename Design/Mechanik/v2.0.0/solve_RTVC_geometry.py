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
MAX_ABS_ANGLE = math.asin(DESIRED_RANGE / SERVO_ARM_LENGTH)

# servo 0
x0 = -WIRE_LENGTH                                   # position
y0 =  MMT_RADIUS + EXTEND_MMT - SERVO_ARM_LENGTH    # position
l0 =  SERVO_ARM_LENGTH                              # arm length

# attachment 0
a0 =  WIRE_LENGTH                                   # attachment length

# servo 1
x1 =  MMT_RADIUS + EXTEND_MMT - SERVO_ARM_LENGTH    # position
y1 = -WIRE_LENGTH                                   # position
l1 =  SERVO_ARM_LENGTH                              # arm length

# attachment 1
a1 =  WIRE_LENGTH                                   # attachment length

# distance between attachments 0 and 1
dx =  MMT_RADIUS + EXTEND_MMT                       # attachment size x
dy =  MMT_RADIUS + EXTEND_MMT                       # attachment size y

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

def err(c, alpha0, alpha1):
    cx, cy = c
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

c0 = (1.0, 1.0)

alphas = []
for i in range(-10, 11, 1):
    alphas.append((-MAX_ABS_ANGLE, i*MAX_ABS_ANGLE/10.0))
    alphas.append((MAX_ABS_ANGLE, i*MAX_ABS_ANGLE/10.0))
    alphas.append((i*MAX_ABS_ANGLE/10.0, -MAX_ABS_ANGLE))
    alphas.append((i*MAX_ABS_ANGLE/10.0, MAX_ABS_ANGLE))

for a in alphas:
    alpha0 = a[0]
    alpha1 = a[1]
    res = minimize(err, c0, args=(alpha0, alpha1), method="Powell")
    xs0 = x0 + l0 * math.sin(alpha0)
    ys0 = y0 + l0 * math.cos(alpha0)
    xs1 = x1 + l1 * math.cos(alpha1)
    ys1 = y1 + l1 * math.sin(alpha1)
    cx = res.x[0]
    cy = res.x[1]
    xa0 = cx
    ya0 = cy + dy
    xa1 = cx + dx
    ya1 = cy
    plt.scatter(xs0, -ys0, s=2, c='red')
    plt.scatter(xs1, -ys1, s=2, c='red')
    plt.scatter(cx, -cy, s=2, c='green')
    plt.scatter(xa0, -ya0, s=1, c='blue')
    plt.scatter(xa1, -ya1, s=1, c='blue')

for i in range(360):
    x = 24.4 * math.cos(i / 180.0 * math.pi)
    y = 24.4 * math.sin(i / 180.0 * math.pi)
    plt.scatter(x, y, s=1, c='gray')

plt.axes().set_aspect('equal', 'datalim')
plt.show()