from matplotlib import pyplot as plt
from geometry import Point, Polygon, Triangle


def plotPoints(points):
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    plt.scatter(xs, ys, zorder=2, color='black')


def plotPolygon(polygon):
    if polygon is None:
        return
    points = polygon.points + [polygon.points[0]]
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    plt.plot(xs, ys)


def plotHighlightedPolygon(polygon):
    if polygon is None:
        return
    points = polygon.points + [polygon.points[0]]
    xs = [p.x for p in points]
    ys = [p.y for p in points]
    plt.fill(xs, ys, color='yellow')


def plotPolygons(polygons):
    for polygon in polygons:
        plotPolygon(polygon)
