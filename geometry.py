from typing import List, Tuple
import numpy as np
import utils


class Point:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def __hash__(self) -> int:
        return hash((self.x, self.y))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

    def __str__(self) -> str:
        return "("+str(self.x)+" , "+str(self.y)+")"


class Segment:
    def __init__(self, start: Point, end: Point) -> None:
        self.start = start
        self.end = end

    def __str__(self) -> str:
        return "["+str(self.start)+" ; "+str(self.end)+"]"


class Polygon:
    def __init__(self, points: List[Point]) -> None:
        self.points = points
        self.size = len(points)
        self.segments = [Segment(points[i-1], points[i])
                         for i in range(self.size)]
        self.reflexVertices = self.getReflexVertices()

    def getReflexVertices(self):
        potentialEars = [[i-1, i, i+1]
                         for i in range(1, self.size-1)]+[[self.size-2, self.size-1, 0], [self.size-1, 0, 1]]
        reflexVertices = [ear[1] for ear in potentialEars if utils.pointCCW(
            self.points[ear[0]], self.points[ear[1]], self.points[ear[2]]) == -1]
        return reflexVertices

    def isCCWOriented(self):
        crossProduct = sum((s.end.x-s.start.x)*(s.end.y+s.start.y)
                           for s in self.segments)
        return crossProduct < 0

    def fixOrient(self):
        if not self.isCCWOriented():
            return Polygon(self.points[::-1])
        else:
            return self

    def isEar(self, ear) -> bool:
        pts = np.array(self.points)
        ear = list(ear)
        if utils.pointCCW(*pts[ear]) != 1:
            return False
        if self.segmentIntersect(Segment(self.points[ear[0]], self.points[ear[2]])):
            return False
        closure = Triangle(pts[ear])
        if any(closure.contains(self.points[v], closed=False) for v in self.reflexVertices):
            return False
        return True

    def getLowerLeft(self):
        yMin = min(self.points, key=lambda p: p.y).y
        xMin = min(self.points, key=lambda p: p.x).x
        return Point(xMin, yMin)

    def getUpperRight(self):
        yMax = max(self.points, key=lambda p: p.y).y
        xMax = max(self.points, key=lambda p: p.x).x
        return Point(xMax, yMax)

    def getBoundingTriangle(self):
        return Triangle.boundingTriangle(self.getLowerLeft(), self.getUpperRight())

    def segmentIntersect(self, segment):
        return any(utils.intersect(segment, edge, closed=False) for edge in self.segments)

    def getEars(self):
        ears = []
        for i in range(self.size):
            ear = [(i-1) % self.size, i, (i+1) % self.size]
            if utils.pointCCW(self.points[ear[0]], self.points[ear[1]], self.points[ear[2]]) == 1 and not self.segmentIntersect(Segment(self.points[ear[0]], self.points[ear[2]])):
                triangle = Triangle([
                    self.points[ear[0]], self.points[ear[1]], self.points[ear[2]]])
                if any(triangle.contains(self.points[vertex], False) for vertex in self.reflexVertices):
                    continue
                ears.append(ear)
        return ears

    def __hash__(self):
        return hash(tuple(self.points))


class Triangle(Polygon):
    def __init__(self, points: List[Point]) -> None:
        super().__init__(points)

    def contains(self, point, closed=True):
        ccw01 = utils.pointCCW(self.points[0], self.points[1], point)
        ccw12 = utils.pointCCW(self.points[1], self.points[2], point)
        ccw20 = utils.pointCCW(self.points[2], self.points[0], point)
        if closed:
            return point in self.points or ccw01*ccw12*ccw20 == 0 or ccw01 == ccw12 == ccw20
        return ccw01 == ccw12 == ccw20

    @classmethod
    def boundingTriangle(cls, lowerLeft, upperRight):
        width = upperRight.x-lowerLeft.x
        height = upperRight.y-lowerLeft.y
        d = height/(3**(1/2))
        a = Point(lowerLeft.x-d, lowerLeft.y-0.5)
        b = Point(upperRight.x+d, lowerLeft.y-0.5)
        c = Point(lowerLeft.x+width/2, lowerLeft.y+height+2*d)
        return cls([a, b, c])


def loadPolygon(file):
    points = []
    triangles = []
    with open(file, 'r') as f:
        pointCount = int(f.readline())
        for _ in range(pointCount):
            coords = f.readline().split(",")
            points.append(Point(float(coords[0][1:]), float(coords[1][:-2])))
        line = f.readline()
        while line:
            triangles.append(eval(line))
            line = f.readline()
    return Polygon(points)


def loadPlanarSubdivision(file):
    points = []
    triangles = []
    with open(file, 'r') as f:
        pointCount = int(f.readline())
        for _ in range(pointCount):
            coords = f.readline().split(",")
            points.append(Point(float(coords[0][1:]), float(coords[1][:-2])))
        line = f.readline()
        while line:
            trianglePointIndexes = eval(line)
            a = points[trianglePointIndexes[0]]
            b = points[trianglePointIndexes[1]]
            c = points[trianglePointIndexes[2]]
            triangles.append(Triangle([a, b, c]))
            line = f.readline()
    originalPolygon = Polygon(points)
    return originalPolygon, triangles
