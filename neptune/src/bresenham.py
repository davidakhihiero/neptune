#!/usr/bin/env python

"""
An implementation of the Bresenham's line algorithm to determine the free cells in the occupancy grip map after 
a lidar scan
Reference: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
"""
def bresenham(start, end):
    x0 = start[0]
    y0 = start[1]

    x1 = end[0]
    y1 = end[1]

    dx = x1 - x0
    dy = y1 - y0

    if abs(dy) < abs(dx):
        if x0 > x1:
            return bresenham_low(end, start)[::-1][:-1]
        else:
            return bresenham_low(start, end)[:-1]
    else:
        if y0 > y1:
            return bresenham_high(end, start)[::-1][:-1]
        else:
            return bresenham_high(start, end)[:-1]


def bresenham_low(start, end):
    x0 = start[0]
    y0 = start[1]

    x1 = end[0]
    y1 = end[1]

    dx = x1 - x0
    dy = y1 - y0

    yi = 1

    if dy < 0:
        yi = -1
        dy = -dy

    d = (2 * dy) - dx
    y = y0

    points = []

    for x in range(x0, x1 + 1):
        points.append([x, y])
        if d > 0:
            y += yi
            d = d + (2 * (dy - dx))
        else:
            d = d + (2 * dy)

    return points


def bresenham_high(start, end):
    x0 = start[0]
    y0 = start[1]

    x1 = end[0]
    y1 = end[1]

    dx = x1 - x0
    dy = y1 - y0

    xi = 1

    if dx < 0:
        xi = -1
        dx = -dx

    d = (2 * dx) - dy
    x = x0

    points = []

    for y in range(y0, y1 + 1):
        points.append([x, y])
        if d > 0:
            x += xi
            d = d + (2 * (dx - dy))
        else:
            d = d + (2 * dx)

    return points


s = [20, 20]
e = [5, 5]

#print (bresenham(s, e))
