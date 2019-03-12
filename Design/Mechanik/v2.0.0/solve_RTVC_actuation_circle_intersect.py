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

def intersect_circles(c0, r0, c1, r1):
    c0x = c0[0]
    c0y = c0[1]
    c1x = c1[0]
    c1y = c1[1]
    d = math.sqrt((c0x-c1x)**2.0 + (c0y-c1y)**2.0)
    if d > (r0 + r1):
        raise ValueError("Circles are separate from each other")
    if d < abs(r0 - r1):
        raise ValueError("One circle is contained within the other")
    a = (r0**2.0 - r1**2.0 + d**2.0) / (2.0 * d)
    h = math.sqrt(r0**2.0 - a**2.0)
    c2x = c0x + a * (c1x - c0x) / d
    c2y = c0y + a * (c1y - c0y) / d
    c3xa = c2x + h * (c1y - c0y) / d
    c3xb = c2x - h * (c1y - c0y) / d
    c3ya = c2y - h * (c1x - c0x) / d
    c3yb = c2y + h * (c1x - c0x) / d
    return ((c3xa, c3ya), (c3xb, c3yb))

def get_angle(p0, p1):
    return math.atan2(p1[1]-p0[1], p1[0]-p0[0])

coords = []
for i in range(-10, 11, 1):
    coords.append((-DESIRED_RANGE,     i*DESIRED_RANGE/10.0))
    coords.append((-DESIRED_RANGE/2.0, i*DESIRED_RANGE/10.0))
    coords.append(( 0.0,               i*DESIRED_RANGE/10.0))
    coords.append(( DESIRED_RANGE/2.0, i*DESIRED_RANGE/10.0))
    coords.append(( DESIRED_RANGE,     i*DESIRED_RANGE/10.0))
    # coords.append((i*DESIRED_RANGE/10.0, -DESIRED_RANGE))
    # coords.append((i*DESIRED_RANGE/10.0,  0.0))
    # coords.append((i*DESIRED_RANGE/10.0,  DESIRED_RANGE))

min_ai0 = math.pi / 2.0
max_ai0 = math.pi / 2.0
min_ai1 = 0.0
max_ai1 = 0.0

for c in coords:
    size = 1
    marker = 'o'
    cx = c[0]
    cy = c[1]
    xa0 = cx
    ya0 = cy + dy
    xa1 = cx + dx
    ya1 = cy
    i0 = intersect_circles((xa0, ya0), a0, (x0, y0), l0)
    ai00 = get_angle((x0, y0), i0[0])
    ai01 = get_angle((x0, y0), i0[1])
    i1 = intersect_circles((xa1, ya1), a1, (x1, y1), l1)
    ai10 = get_angle((x1, y1), i1[0])
    ai11 = get_angle((x1, y1), i1[1])
    if abs(ai00 - math.pi/2.0) < abs(ai01 - math.pi/2.0):
        s0 = i0[0]
        if (ai00 < min_ai0):
            min_ai0 = ai00
        if (ai00 > max_ai0):
            max_ai0 = ai00
    else:
        s0 = i0[1]
        if (ai01 < min_ai0):
            min_ai0 = ai01
        if (ai01 > max_ai0):
            max_ai0 = ai01
    if abs(ai10) < abs(ai11):
        s1 = i1[0]
        if (ai10 < min_ai1):
            min_ai1 = ai10
        if (ai10 > max_ai1):
            max_ai1 = ai10
    else:
        s1 = i1[1]
        if (ai11 < min_ai1):
            min_ai1 = ai11
        if (ai11 > max_ai1):
            max_ai1 = ai11
    xs0 = s0[0]
    ys0 = s0[1]
    xs1 = s1[0]
    ys1 = s1[1]
    plt.scatter(xs0, -ys0, s=2*size, c='red', marker=marker)
    plt.scatter(xs1, -ys1, s=2*size, c='red', marker=marker)
    plt.scatter(cx, -cy, s=2*size, c='green', marker=marker)
    plt.scatter(xa0, -ya0, s=1*size, c='blue', marker=marker)
    plt.scatter(xa1, -ya1, s=1*size, c='blue', marker=marker)
    plt.plot([xs0, xa0], [-ys0, -ya0], color='gray', linewidth=0.5)
    plt.plot([xs1, xa1], [-ys1, -ya1], color='gray', linewidth=0.5)

for i in range(360):
    x = 24.4 * math.cos(i / 180.0 * math.pi)
    y = 24.4 * math.sin(i / 180.0 * math.pi)
    plt.scatter(x, y, s=1, c='gray')

plt.axes().set_aspect('equal', 'datalim')
plt.show()

print("angles servo 0:", min_ai0 * 180.0 / math.pi, max_ai0 * 180.0 / math.pi)
print("angles servo 1:", min_ai1 * 180.0 / math.pi, max_ai1 * 180.0 / math.pi)
