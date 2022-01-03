from collections import defaultdict
from matplotlib import pyplot as plt
import numpy as np
from geometry import Point, Polygon, Triangle
from graph import Graph, DirectedGraph
from triangulation import triangulate
from geometry import loadPlanarSubdivision
from visualization import Visualizer


class Kirkpatrick:
    def __init__(self, name=None, stepVisualization=False) -> None:
        self.directedGraph = DirectedGraph()
        self.visualizer = Visualizer(stepVisualization)
        if name is not None:
            self.loadPolygons(name)
        else:
            self.getPolygonInput()

    def loadPolygons(self, name):
        self.originalPolygon, self.originalTriangles = loadPlanarSubdivision(
            name)
        self._preprocessPolygons(self.originalTriangles, self.originalPolygon)

    def _preprocessPolygons(self, polygons, originalPolygon: Polygon, boundingTriangle=None):
        self.currentLayer = 0
        triangles = self.triangulateToGraph(polygons)
        if boundingTriangle is None:
            boundingTriangle = originalPolygon.getBoundingTriangle()
        ringTriangulation = triangulate(boundingTriangle, originalPolygon)
        for triangle in ringTriangulation:
            self.directedGraph.addNode(triangle, False)
        triangulation = triangles+ringTriangulation
        if self.visualizer.active:
            self.visualizer.triangulations.append(triangulation)
            self.visualizer.plotPolygons(triangulation)
            # plt.savefig("triangulation0.png")
            plt.clf()
        while len(triangulation) > 1:
            triangulation = self.nextTriangulation(
                triangulation, boundingTriangle)
            self.currentLayer += 1
            if self.visualizer.active:
                self.visualizer.triangulations.append(triangulation)
                self.visualizer.plotPolygons(triangulation)
                # plt.savefig("triangulation{}.png".format(k))
                # plt.clf()
        self.layer = triangulation

    def nextTriangulation(self, prevTriangulation, boundingTriangle):
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
        unaffectedTriangles = set(range(len(prevTriangulation)))
        newTriangulation = []
        for point in independentSet:
            affectedTriangles = pointPolygonMap[point]
            unaffectedTriangles -= affectedTriangles
            newPolygon = self.removePointFromTriangulation(
                point, [prevTriangulation[i] for i in affectedTriangles])
            newTriangles = triangulate(newPolygon)
            newTriangulation += newTriangles
            for triangle in newTriangles:
                for i in affectedTriangles:
                    self.directedGraph.addNode(triangle, False)
                    self.directedGraph.addEdge(
                        triangle, prevTriangulation[i], self.currentLayer)
        newTriangulation += [prevTriangulation[i] for i in unaffectedTriangles]
        return newTriangulation

    def removePointFromTriangulation(self, point, affectedTriangles):
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
        cycleEnd = graph.findCycle(startnode)
        node = graph.parents[cycleEnd]
        points = []
        while node is not None:
            points.append(node)
            node = graph.parents.get(node, None)
        points.append(cycleEnd)
        newPolygon = Polygon(points)
        newPolygon = newPolygon.fixOrient()
        return newPolygon

    def triangulateToGraph(self, polygons):
        triangles = []
        for polygon in polygons:
            self.directedGraph.addNode(polygon, True)
            if polygon.size == 3:
                polygon.__class__ = Triangle
                triangles.append(polygon)
            else:
                for triangle in triangulate(polygon):
                    triangles.append(triangle)
                    self.directedGraph.addNode(triangle, False)
                    self.directedGraph.addEdge(
                        triangle, polygon, self.currentLayer)
        return triangles

    def _locatePoint(self, point):
        # print(self.directedGraph.nodes)
        node = None
        k = 0
        for triangle in self.layer:
            if triangle.contains(point):
                node = triangle
                if self.visualizer.active:
                    self.visualizer.addLocationFrame(
                        self.layer, point, triangle)
                break
        else:
            return None
        k = 1
        nodeNeighbours = self.directedGraph.getNeighbours(node)
        while nodeNeighbours is not None:
            for (triangle, layerIndex) in nodeNeighbours:
                if isinstance(triangle, Triangle) and triangle.contains(point):
                    node = triangle
                    if self.visualizer.active and self.directedGraph.nodes[node] == False:
                        self.visualizer.addLocationFrame(
                            self.visualizer.triangulations[layerIndex], point, triangle)
                        k += 1
                    nodeNeighbours = self.directedGraph.getNeighbours(node)
                    break
                elif not isinstance(triangle, Triangle):
                    node = triangle
                    if self.visualizer.active and self.directedGraph.nodes[node] == False:
                        self.visualizer.addLocationFrame(
                            self.visualizer.triangulations[layerIndex], point, triangle)
                        k += 1
                    nodeNeighbours = self.directedGraph.getNeighbours(node)
                    break
            else:
                return None
        if self.directedGraph.nodes[node]:
            if self.visualizer.active:
                self.visualizer.addLocationFrame(
                    self.visualizer.triangulations[0], point, triangle)
            return node
        else:
            return None

    def locatePoint(self, point):
        node = self._locatePoint(point)
        self.visualizer.drawFinalLocation()

    def getPolygonInput(self):
        boundingTriangle = Triangle(
            [Point(0, 0), Point(20, 0), Point(10, 17.3)])
        self.originalPolygon = self.visualizer.getPolygonInput(
            boundingTriangle)
        self.originalTriangles = triangulate(self.originalPolygon)
        self._preprocessPolygons(
            self.originalTriangles, self.originalPolygon, boundingTriangle)

    def pickLocatePoint(self):
        self.visualizer.originalTriangles = self.originalTriangles
        self.visualizer.originalPolygon = self.originalPolygon
        pointToLocate = self.visualizer.getPointToLocate()
        self.locatePoint(pointToLocate)


# locationTool = Kirkpatrick("./sample_triangulations/example2.txt", True)
# locationTool = Kirkpatrick(stepVisualization=True)
# locationTool.pickLocatePoint()
