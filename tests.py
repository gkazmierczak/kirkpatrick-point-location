import numpy as np
import matplotlib.pyplot as plt
from numpy.lib import utils
from geometry import Point, Polygon, Segment, Triangle
from triangulation import triangulate
from scipy.spatial import Delaunay
from utils import intersect, pointCCW


T = [1, 10, 20, 30, 40, 50, 60, 70]
R = [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]


def circlePoints(n, r):
    points = []
    indexes = {}
    for i in range(n):
        t = i * (2 * np.pi / n)
        point = Point(r * np.cos(t), r * np.sin(t))
        indexes[point] = i
        points.append(point)
    np_pts = np.array([(p.x, p.y) for p in points])
    pts = np.array(points)
    triangulation = Delaunay(np_pts)
    triangles = triangulation.simplices
    with open("./random"+str(n)+".txt", 'w') as f:
        f.write(str(len(points))+'\n')
        for p in points:
            f.write(str(p)+'\n')
        f.write(str(list(range(n)))+'\n')
        for t in triangles:
            a = [x for x in t]
            f.write(str(a)+'\n')
        f.close()


circlePoints(1000, 50)
