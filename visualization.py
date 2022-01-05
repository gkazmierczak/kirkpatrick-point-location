from matplotlib import pyplot as plt
from geometry import Point, Polygon, Triangle
from matplotlib.widgets import Button

from utils import distance


class Scene:
    def __init__(self, points=[], polygons=[]):
        self.points = points
        self.polygons = polygons


class Visualizer:
    def __init__(self, active: bool) -> None:
        self.active = active
        self._scenes = []
        self.triangulations = []
        self.pointData = None
        self.index = 0
        self._inputScene = Scene([], [Triangle(
            [Point(0, 0), Point(20, 0), Point(10, 17.3)])])
        self.originalPolygon = None
        self.originalTriangles = []
        self.boundingTriangle = None

    def _handleClose(self, event):
        plt.close('all')
        exit(0)

    def getPolygonInput(self, boundingTriangle):
        self.boundingTriangle = boundingTriangle
        plt.close()
        fig = plt.figure(figsize=(10, 10))
        self._axPlot = plt.axes((0.05, 0.2, 0.9, 0.7))
        self._axPlot.set_xlim(0, 20)
        self._axPlot.set_ylim(0, 18)
        self._axPlot.clear()
        self._axPlot.set_title("Waiting for polygon input")

        points = boundingTriangle.points+[boundingTriangle.points[0]]
        xs = [p.x for p in points]
        ys = [p.y for p in points]
        self._axPlot.plot(xs, ys, color="black")
        polygonPoints = []
        addingPoints = True
        while addingPoints:
            pointData = plt.ginput(1)[0]
            point = Point(pointData[0], pointData[1])
            while not boundingTriangle.contains(point):
                pointData = plt.ginput(1)[0]
                point = Point(pointData[0], pointData[1])
            polygonPoints.append(point)
            if len(polygonPoints) > 1:
                self._axPlot.plot([polygonPoints[-2].x, point.x],
                                  [polygonPoints[-2].y, point.y], color="blue")
                if distance(point, polygonPoints[0]) < 0.4:
                    addingPoints = False
            plt.draw()
        self._axPlot.plot([polygonPoints[-2].x, polygonPoints[-1].x],
                          [polygonPoints[-2].y, polygonPoints[-1].y], color="blue")
        plt.draw()
        self.originalPolygon = Polygon(polygonPoints[:-1])
        self.originalPolygon = self.originalPolygon.fixOrient()
        return self.originalPolygon

    def _draw(self):
        self._axPlot.clear()
        scene = self._scenes[self.index]
        if 'polygons' in scene:
            for polygon in scene['polygons']:
                points = polygon.points+[polygon.points[0]]
                xs = [p.x for p in points]
                ys = [p.y for p in points]
                self._axPlot.plot(xs, ys, color="black")
        if 'point' in scene:
            self._axPlot.scatter(
                scene['point'].x, scene['point'].y, color="red", zorder=3)
        if 'independentSet' in scene:
            xs = [p.x for p in scene['independentSet']]
            ys = [p.y for p in scene['independentSet']]
            self._axPlot.scatter(xs, ys, color="cyan", zorder=4)
        if 'activePoints' in scene:
            xs = [p.x for p in scene['activePoints']]
            ys = [p.y for p in scene['activePoints']]
            self._axPlot.scatter(xs, ys, color="blue", zorder=3)
        if 'highlight' in scene:
            polygon = scene['highlight']
            points = polygon.points+[polygon.points[0]]
            xs = [p.x for p in points]
            ys = [p.y for p in points]
            self._axPlot.fill(xs, ys, color='yellow')
        if 'input' in scene:
            if scene['bg']:
                self._axPlot.set_facecolor('black')
            for polygon in scene['input']:
                points = polygon.points+[polygon.points[0]]
                xs = [p.x for p in points]
                ys = [p.y for p in points]
                self._axPlot.fill(xs, ys, color="white")
        if 'time' in scene:
            self._axPlot.set_title(
                "N = "+str(self.originalPolygon.size)+"   Location time: "+str(scene['time']))
        points = self.originalPolygon.points+[self.originalPolygon.points[0]]
        xs = [p.x for p in points]
        ys = [p.y for p in points]
        self._axPlot.plot(xs, ys, "red", zorder=2)
        plt.draw()

    def draw(self):
        plt.close()
        plt.figure(figsize=(10, 10))
        self._axPlot = plt.axes((0.05, 0.2, 0.9, 0.7))
        ax_prev = plt.axes((0.2, 0.03, 0.2, 0.09))
        ax_next = plt.axes((0.6, 0.03, 0.2, 0.09))
        prevBtn = Button(ax_prev, 'PREVIOUS',
                         color='silver', hovercolor='slategrey')
        nextBtn = Button(ax_next, 'NEXT', color='silver',
                         hovercolor='slategrey')
        prevBtn.on_clicked(self.prev)
        nextBtn.on_clicked(self.next)
        self._draw()
        plt.show()

    def getPointToLocate(self):
        plt.close()
        fig = plt.figure(figsize=(10, 10))
        cid = fig.canvas.mpl_connect('close_event', self._handleClose)
        self._axPlot = plt.axes((0.05, 0.2, 0.9, 0.7))
        self._axPlot.set_xlim(0, 20)
        self._axPlot.set_ylim(0, 18)
        self._axPlot.clear()
        self._axPlot.set_title("Choose a point")
        for triangle in self.originalTriangles:
            points = triangle.points+[triangle.points[0]]
            xs = [p.x for p in points]
            ys = [p.y for p in points]
            self._axPlot.plot(xs, ys, color="red")
        plt.draw()
        points = self.originalPolygon.points+[self.originalPolygon.points[0]]
        xs = [p.x for p in points]
        ys = [p.y for p in points]
        self._axPlot.plot(xs, ys, color="green")
        plt.draw()
        pointData = plt.ginput(1)

        fig.canvas.mpl_disconnect(cid)
        pointData = pointData[0]
        point = Point(pointData[0], pointData[1])
        while not self.boundingTriangle.contains(point):
            pointData = plt.ginput(1)[0]
            point = Point(pointData[0], pointData[1])
        self._axPlot.scatter(point.x, point.y, color="red")
        return point

    def addLocationFrame(self, polygons, point, highlight, time=None):
        self._scenes.append({
            'polygons': polygons,
            'point': point,
            'highlight': highlight
        })
        if time != None:
            self._scenes[-1]['time'] = time

    def drawFinalLocation(self):
        self.index = len(self._scenes)-1
        self.draw()

    def next(self, event):
        self.index = (self.index+1) % len(self._scenes)
        self._draw()
        pass

    def prev(self, event):
        self.index = (self.index-1) % len(self._scenes)
        self._draw()
        pass

    def addTriangulation(self, triangulation, activePoints):
        self.triangulations.append(triangulation)
        self._scenes.append({
            'polygons': triangulation,
            'activePoints': activePoints,
        })

    def addIndependentSetFrame(self, triangulation, independentSet, activePoints):
        self.triangulations.append(triangulation)
        self._scenes.append({
            'polygons': triangulation,
            'independentSet': independentSet,
            'activePoints': activePoints
        })

    def locationFailure(self, point):
        self._scenes.append({
            'input': self.originalTriangles,
            'point': point,
            'bg': True
        })
        self.draw()
