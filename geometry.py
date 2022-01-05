from typing import List, Tuple
import numpy as np


class Point:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def __hash__(self) -> int:
        return hash((self.x, self.y))

    def __eq__(self, other):
        if isinstance(other, Point):
            return self.x == other.x and self.y == other.y
        return False

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
        self.reflexVertices = self._getReflexVertices()

    def _getReflexVertices(self):
        potentialEars = [[i-1, i, i+1]
                         for i in range(1, self.size-1)]+[[self.size-2, self.size-1, 0], [self.size-1, 0, 1]]
        reflexVertices = [ear[1] for ear in potentialEars if pointCCW(
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
        if pointCCW(*pts[ear]) != 1:
            return False
        if self._pointIntersect(Triangle([self.points[ear[0]], self.points[ear[1]], self.points[ear[2]]])):
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

    def _pointIntersect(self, triangle):
        return any(triangle.contains(p) for p in self.points if p not in triangle.points)

    def getEars(self):
        ears = []
        for i in range(self.size):
            ear = [(i-1) % self.size, i, (i+1) % self.size]
            if self.isEar(ear):
                ears.append(ear)
        return ears

    def __hash__(self):
        return hash(tuple(self.points))


class Triangle(Polygon):
    def __init__(self, points: List[Point]) -> None:
        super().__init__(points)

    def contains(self, point, closed=True):
        ccw01 = pointCCW(point, self.points[0], self.points[1])
        ccw12 = pointCCW(point, self.points[1], self.points[2])
        ccw20 = pointCCW(point, self.points[2], self.points[0])
        neg = (-1 in [ccw01, ccw12, ccw20])
        pos = (1 in [ccw01, ccw12, ccw20])
        if closed:
            return not(neg and pos)
        return ccw01 == ccw12 == ccw20

    @classmethod
    def boundingTriangle(cls, lowerLeft, upperRight):
        width = upperRight.x-lowerLeft.x
        height = upperRight.y-lowerLeft.y
        d = height/(3**(1/2))
        a = Point(lowerLeft.x-d, lowerLeft.y-0.5)
        b = Point(upperRight.x+d, lowerLeft.y-0.5)
        c = Point(lowerLeft.x+width/2, lowerLeft.y+2*height+2*d)
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
    polygons = []
    with open(file, 'r') as f:
        pointCount = int(f.readline())
        for _ in range(pointCount):
            coords = f.readline().split(",")
            points.append(Point(float(coords[0][1:]), float(coords[1][:-2])))
        line = f.readline()
        while line:
            pointIndexes = eval(line)
            selectedPoints = []
            for i in pointIndexes:
                selectedPoints.append(points[i])
            if len(selectedPoints) == 3:
                polygons.append(Triangle(selectedPoints))
            else:
                polygon = Polygon(selectedPoints)
                polygons.append(polygon.fixOrient())
            line = f.readline()
    originalPolygon = polygons[0].fixOrient()
    return originalPolygon, polygons[1:]


def pointCCW(a, b, c) -> int:
    '''
    abc is counter-clockwise:
        return 1
    abc is clockwise:
        return -1
    abc are collinear:
        return 0
    '''
    d = (a.x*b.y)+(b.x*c.y)+(c.x*a.y) - \
        (c.x*b.y)-(b.x*a.y)-(a.x*c.y)
    e = 10**-12
    if d < -e:
        return -1
    elif d < e:
        return 0
    else:
        return 1


def distance(a, b):
    return ((b.x-a.x)**2+(b.y-a.y)**2)**(1/2)
