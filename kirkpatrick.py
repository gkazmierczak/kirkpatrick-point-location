from collections import defaultdict
from matplotlib import pyplot as plt
import numpy as np
from geometry import Point, Polygon, Triangle
from graph import Graph, DirectedGraph
from triangulation import triangulate
from geometry import loadPlanarSubdivision
import visualization


class Kirkpatrick:
    def __init__(self, name, stepVisualization=False) -> None:
        self.directedGraph = DirectedGraph()
        self.stepVisualization = stepVisualization
        self.loadPolygons(name)

    def loadPolygons(self, name):
        self.originalPolygon, self.originalTriangles = loadPlanarSubdivision(
            name)
        self._preprocessPolygons(self.originalTriangles, self.originalPolygon)

    def _preprocessPolygons(self, polygons, originalPolygon: Polygon):
        triangles = self.triangulateToGraph(polygons)
        boundingTriangle = originalPolygon.getBoundingTriangle()
        ringTriangulation = triangulate(boundingTriangle, originalPolygon)
        for triangle in ringTriangulation:
            self.directedGraph.addNode(triangle, False)
        triangulation = triangles+ringTriangulation
        if self.stepVisualization:
            visualization.plotPolygons(triangulation)
            # plt.show()
            plt.clf()
        k = 0
        while len(triangulation) > 1:
            triangulation = self.nextTriangulation(
                triangulation, boundingTriangle)
            k += 1
            if self.stepVisualization:
                visualization.plotPolygons(triangulation)
                plt.savefig("triangulation{}.png".format(k))
                plt.clf()
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
                        triangle, prevTriangulation[i])
        newTriangulation += [prevTriangulation[i] for i in unaffectedTriangles]
        return newTriangulation

    def removePointFromTriangulation(self, point, affectedTriangles):
        graph = Graph()
        print("removing: ", point)
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
                    self.directedGraph.addEdge(triangle, polygon)
        return triangles

    def _locatePoint(self, point):
        node = None
        for triangle in self.layer:
            if triangle.contains(point):
                node = triangle
                break
        else:
            return None

        nodeNeighbours = self.directedGraph.getNeighbours(node)
        while nodeNeighbours is not None:
            for triangle in nodeNeighbours:
                if triangle.contains(point):
                    # print(triangle)
                    # print(self.directedGraph.getNeighbours(triangle))
                    node = triangle
                    nodeNeighbours = self.directedGraph.getNeighbours(node)
                    break
            else:
                return None
        if self.directedGraph.nodes[node]:
            return node
        else:
            return None
        # return node
        # print(node)
        # for p in node.points:
        #     print(p)

    def locatePoint(self, point):
        node = self._locatePoint(point)
        visualization.plotPolygons(self.originalTriangles)
        visualization.plotPoints([point])
        if node != None:
            visualization.plotHighlightedPolygon(node)
        plt.show()


locationTool = Kirkpatrick("./sample_triangulations/example3.txt", True)
point = Point(2.8, 8.7)
locationTool.locatePoint(point)
# if node != None:
#     visualization.plotPolygon(node)
#     visualization.plotPoints([point])
#     plt.show()
