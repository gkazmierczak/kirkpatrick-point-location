from collections import defaultdict


class DirectedGraph:
    def __init__(self) -> None:
        self.edges = defaultdict(set)
        self.nodes = {}
        pass

    def addNode(self, node, isOriginalPolygon):
        self.nodes[node] = isOriginalPolygon

    def addEdge(self, node, relative):
        self.edges[node].add(relative)

    def getNeighbours(self, node):
        if node in self.nodes and self.edges[node] != set():
            return self.edges[node]
        else:
            return None


class Graph:
    def __init__(self) -> None:
        self.edges = defaultdict(set)
        self.nodes = set()
        self._cycleEnd = None
        self._parents = {}

    def addEdge(self, u, v):
        self.edges[u].add(v)
        self.edges[v].add(u)

    def addNode(self, node):
        self.nodes.add(node)

    def findIndependentSet(self, excludedNodes):
        possibleNodes = {node for node in self.nodes if len(
            self.edges[node]) <= 8 and node not in excludedNodes}
        independentSet = set()
        while len(possibleNodes) > 0:
            node = possibleNodes.pop()
            independentSet.add(node)
            possibleNodes -= self.edges[node]
        return independentSet

    def _findCycle(self, node):
        visited = set()
        self._DFSUtil(node, visited)
        return self._cycleEnd

    def _DFSUtil(self, node, visited):
        visited.add(node)
        continued = False
        for edge in self.edges[node]:
            if edge not in visited:
                self._parents[edge] = node
                continued = True
                self._DFSUtil(edge, visited)
        if not continued:
            self._cycleEnd = node
