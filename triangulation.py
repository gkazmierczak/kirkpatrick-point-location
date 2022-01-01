import numpy as np
from itertools import product
from matplotlib import pyplot as plt
from geometry import Point, Polygon, Segment, Triangle, loadPolygon
import visualization
import utils


def _segment_intersect(segment, polygon):
    return any(utils.intersect(segment, edge) for edge in polygon.segments)


def _bridgeHole(polygon, hole):
    possibleBridges = list(product(polygon.points, hole.points))
    possibleBridges.sort(key=lambda b: utils.distance(*b))
    bridge = None
    for possibleBridge in possibleBridges:
        if _segment_intersect(Segment(possibleBridge[0], possibleBridge[1]), polygon) or _segment_intersect(Segment(possibleBridge[0], possibleBridge[1]), hole):
            continue
        bridge = possibleBridge
        break
    if bridge == None:
        bridge = possibleBridges[0]
    polygonPoint = bridge[0]
    holePoint = bridge[1]
    polygonIndex = polygon.points.index(polygonPoint)
    polygonPoints = list(np.roll(polygon.points, -polygonIndex))
    holePoints = hole.points[::-1]
    holeIndex = holePoints.index(holePoint)
    holePoints = list(np.roll(holePoints, -holeIndex))
    points = polygonPoints+[polygonPoint]+holePoints+[holePoint]
    return Polygon(points)


def triangulate(polygon, hole=None):
    if hole:
        polygon = _bridgeHole(polygon, hole)
    points = np.array(polygon.points)
    n = polygon.size
    curr_n = n
    ears = {v[1]: v for v in polygon.getEars()}
    triangles = []
    adjacencyDict = {i: ((i-1) % n, (i+1) % n) for i in range(n)}
    while len(triangles) < n-2:
        b, ear = ears.popitem()
        triangles.append(Triangle(points[ear].tolist()))
        a, b, c = ear
        adjacencyDict[a] = (adjacencyDict[a][0], c)
        adjacencyDict[c] = (a, adjacencyDict[c][1])
        ear_a = (adjacencyDict[a][0], a, c)
        ear_c = (a, c, adjacencyDict[c][1])
        if polygon.isEar(ear_a):
            ears[a] = list(ear_a)
        else:
            ears.pop(a, None)

        if polygon.isEar(ear_c):
            ears[c] = list(ear_c)
        else:
            ears.pop(c, None)

        curr_n -= 1

    return triangles


# polygon = loadPolygon("./sample_triangulations/example2.txt")
# tri = Triangle.boundingTriangle(
#     polygon.getLowerLeft(), polygon.getUpperRight())
# visualization.plotPolygons(triangulate(tri, polygon))
# # visualization.plotPolygons(triangulate(polygon))
# plt.show()