from collections import defaultdict
import numpy as np
from geometry import Point, Polygon, Triangle
from graph import Graph, DirectedGraph
from triangulation import triangulate
from geometry import loadPlanarSubdivision
from visualization import Visualizer
from time import time


class Kirkpatrick:
    def __init__(self, filepath=None, stepVisualization=False) -> None:
        self._directedGraph = DirectedGraph()
        self._visualizer = Visualizer(stepVisualization)
        if filepath is not None:
            self._loadPolygons(filepath)
        else:
            self._getPolygonInput()

    def _loadPolygons(self, filepath):
        self.originalPolygon, self.originalTriangles = loadPlanarSubdivision(
            filepath)
        self._preprocessPolygons(self.originalTriangles, self.originalPolygon)

    def _preprocessPolygons(self, polygons, originalPolygon: Polygon, boundingTriangle=None):
        self.activePoints = set()
        triangles = self._triangulateToGraph(polygons)
        if boundingTriangle is None:
            boundingTriangle = originalPolygon.getBoundingTriangle()
            self._visualizer.boundingTriangle = boundingTriangle
        if self._visualizer.active:
            for polygon in polygons:
                for point in polygon.points:
                    self.activePoints.add(point)
            self._visualizer._addTriangulation(
                self.originalTriangles, self.activePoints.copy())
            self._visualizer._addTriangulation(
                triangles, self.activePoints.copy())
            self._visualizer._addTriangulation(
                triangles+[boundingTriangle], self.activePoints.copy())
        ringTriangulation = triangulate(boundingTriangle, originalPolygon)
        for triangle in ringTriangulation:
            self._directedGraph.addNode(triangle, False)
        triangulation = triangles+ringTriangulation
        if self._visualizer.active:
            self._visualizer._addTriangulation(
                triangulation, self.activePoints.copy())
        while len(triangulation) > 1:
            triangulation = self._nextTriangulation(
                triangulation, boundingTriangle)
            if self._visualizer.active:
                self._visualizer._addTriangulation(
                    triangulation, self.activePoints.copy())
        self.layer = triangulation

    def _nextTriangulation(self, prevTriangulation, boundingTriangle):
        pointPolygonMap = defaultdict(set)
        for i, triangle in enumerate(prevTriangulation):
            for point in triangle.points:
                pointPolygonMap[point].add(i)
        graph = Graph()
        for triangle in prevTriangulation:
            for u, v in zip(triangle.points, np.roll(triangle.points, 1)):
                graph.addNode(u)
                graph.addNode(v)
                graph.addEdge(u, v)
        independentSet = graph.findIndependentSet(boundingTriangle.points)
        if self._visualizer.active:
            self.activePoints -= independentSet
            self._visualizer._addIndependentSetFrame(
                prevTriangulation, independentSet, self.activePoints.copy())
        holes = []
        unaffectedTriangles = set(range(len(prevTriangulation)))
        newTriangulation = []
        for point in independentSet:
            affectedTriangles = pointPolygonMap[point]
            unaffectedTriangles -= affectedTriangles
            newPolygon = self._removePointFromTriangulation(
                point, [prevTriangulation[i] for i in affectedTriangles])
            holes.append(newPolygon)
            newTriangles = triangulate(newPolygon)
            newTriangulation += newTriangles
            for triangle in newTriangles:
                for i in affectedTriangles:
                    self._directedGraph.addNode(triangle, False)
                    self._directedGraph.addEdge(
                        triangle, prevTriangulation[i])
        newTriangulation += [prevTriangulation[i] for i in unaffectedTriangles]
        if self._visualizer.active:
            if len(unaffectedTriangles) > 0 or len(holes) > 1:
                self._visualizer._addTriangulation(
                    [prevTriangulation[i] for i in unaffectedTriangles]+holes, self.activePoints.copy())
        return newTriangulation

    def _removePointFromTriangulation(self, point, affectedTriangles):
        graph = Graph()
        startnode = None
        for triangle in affectedTriangles:
            affectedPoints = set(triangle.points)-{point}
            u = affectedPoints.pop()
            v = affectedPoints.pop()
            if startnode is None:
                startnode = u
            graph.addNode(u)
            graph.addNode(v)
            graph.addEdge(u, v)
        cycleEnd = graph._findCycle(startnode)
        node = graph._parents[cycleEnd]
        points = []
        while node is not None:
            points.append(node)
            node = graph._parents.get(node, None)
        points.append(cycleEnd)
        newPolygon = Polygon(points)
        newPolygon = newPolygon.fixOrient()
        return newPolygon

    def _triangulateToGraph(self, polygons):
        triangles = []
        for polygon in polygons:
            self._directedGraph.addNode(polygon, True)
            if polygon.size == 3:
                polygon.__class__ = Triangle
                triangles.append(polygon)
            else:
                for triangle in triangulate(polygon):
                    triangles.append(triangle)
                    self._directedGraph.addNode(triangle, False)
                    self._directedGraph.addEdge(
                        triangle, polygon)
        return triangles

    def _locatePoint(self, point):
        start = time()
        node = None
        k = 0
        for triangle in self.layer:
            if triangle.contains(point):
                node = triangle
                if self._visualizer.active:
                    self._visualizer._addLocationFrame(
                        self.layer, point, triangle)
                break
        else:
            return None
        k = 1
        nodeNeighbours = self._directedGraph.getNeighbours(node)
        while nodeNeighbours is not None:
            for triangle in nodeNeighbours:
                if isinstance(triangle, Triangle) and triangle.contains(point):
                    node = triangle
                    if self._visualizer.active and self._directedGraph.nodes[node] == False:
                        self._visualizer._addLocationFrame(
                            [t for t in self._visualizer.triangulations if node in t][0], point, triangle)
                        k += 1
                    nodeNeighbours = self._directedGraph.getNeighbours(node)
                    break
                elif not isinstance(triangle, Triangle):
                    node = triangle
                    if self._visualizer.active and self._directedGraph.nodes[node] == False:
                        self._visualizer._addLocationFrame(
                            [t for t in self._visualizer.triangulations if node in t][0], point, triangle)
                        k += 1
                    nodeNeighbours = self._directedGraph.getNeighbours(node)
                    break
            else:
                return None
        if self._directedGraph.nodes[node]:
            totalTime = time()-start
            if self._visualizer.active:
                self._visualizer._addLocationFrame(
                    self._visualizer.triangulations[2], point, triangle)
            self._visualizer._addLocationFrame(
                self._visualizer.originalTriangles, point, node, time=totalTime)
            return node
        else:
            return None

    def locatePoint(self, point):
        node = self._locatePoint(point)
        if node != None:
            self._visualizer._drawFinalLocation()
        else:
            self._visualizer._locationFailure(point)
        return node

    def _getPolygonInput(self):
        boundingTriangle = Triangle(
            [Point(0, 0), Point(20, 0), Point(10, 17.3)])
        self.originalPolygon = self._visualizer.getPolygonInput(
            boundingTriangle)
        self.originalTriangles = triangulate(self.originalPolygon)
        self._preprocessPolygons(
            self.originalTriangles, self.originalPolygon, boundingTriangle)

    def pickLocatePoint(self):
        self._visualizer.originalTriangles = self.originalTriangles
        self._visualizer.originalPolygon = self.originalPolygon
        pointToLocate = self._visualizer._getPointToLocate()
        return self.locatePoint(pointToLocate)
