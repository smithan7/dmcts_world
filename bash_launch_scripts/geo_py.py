#!/usr/bin/env python

import rospy, math
import numpy as np
from random import *
import sys, termios, tty, select, os
from sympy import *
from sympy.geometry import *


if __name__ == '__main__':
    x = Point(0.5, 10.2)
    y = Point(1, 1)
    z = Point(2, 2)
    zp = Point(1, 0)
    d = zp.distance(x).evalf()
    print "d: ", d
    Point.is_collinear(x, y, z)
    Point.is_collinear(x, y, zp)
    t = Triangle(zp, y, x)
    print t.area
    print t.medians[x]
    print Segment(Point(0, 0), Point(1, 1/2))
    print Segment(Point(1, S(1)/2), Point(0, 0))
    print Segment(Point(0, 0), Point(1, 1/2))
    m = t.medians
    print intersection(m[x], m[y], m[zp])
    c = Circle(x, 5)
    l = Line(Point(5, -5), Point(5, 5))
    print c.is_tangent(l) # is l tangent to c?
    l = Line(x, y)
    print c.is_tangent(l) # is l tangent to c?
    print intersection(c, l)