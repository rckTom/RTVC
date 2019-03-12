import matplotlib.pyplot as plt
import math

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

def plot_circle(c, r):
    for i in range(360):
        x = c[0] + r * math.cos(i / 180.0 * math.pi)
        y = c[1] + r * math.sin(i / 180.0 * math.pi)
        plt.scatter(x, y, s=1, c='gray')

c0 = (-2.0, 0.0)
r0 = 3.0
c1 = (3.0, 0.0)
r1 = 4.0
plot_circle(c0, r0)
plot_circle(c1, r1)
intersections = intersect_circles(c0, r0, c1, r1)
plt.scatter(intersections[0][0], intersections[0][1], s=6, c='red')
plt.scatter(intersections[1][0], intersections[1][1], s=6, c='red')

plt.axes().set_aspect('equal', 'datalim')
plt.show()