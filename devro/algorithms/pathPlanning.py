import numpy as np
import cv2

class Node():
    def __init__(self, v, h, parent = None):
        self.v = v
        self.h = h
        self.fValue = 0
        self.gValue = 0
        self.hValue = 0
        self.parent = parent
        self.visited = 0

    def __add__(self, o):
        return Node(self.v + o.v, self.h + o.h)

    def __sub__(self, o):
        return Node(self.v - o.v, self.h - o.h)

    def __mul__(self, o):
        return Node(self.v*o.v, self.h*o.h)

    def __str__(self):
        return str(self.v) + ', ' + str(self.h)

    def __eq__(self, o):
        if self.v == o.v and self.h == o.h:
            return True
        else:
            return False

    def __mod__(self, o):
        return ((self.v-o.v)**2 + (self.h-o.h)**2)**0.5


class PathFinder():
    def __init__(self, startNode, destNode, mapImg, canvas, pixelSpan = 800):
        self.currNode = startNode
        self.startNode = startNode
        self.mapImg = mapImg
        self.destNode = destNode
        self.canvas = canvas
        self.trajectory = []
        self.visited = np.zeros_like(mapImg)
        self.pathCost = 0
        self.refNeighbours = []
        for m in [0, -1, 1]:
            for n in [0, -1, 1]:
                if m != 0 or n != 0:
                    self.refNeighbours.append(Node(m,n))

    def hValue(self, node):
        normFactor = max(abs(self.startNode.v-self.destNode.v), abs(self.startNode.h-self.destNode.h))
        return max(abs(node.v-self.destNode.v), abs(node.h-self.destNode.h))/normFactor

    def gValue(self, node):
        return (255 - self.mapImg[node.v][node.h])/255

    def getLeastFNode(self, kg = 1, kh = 1):
        neighbours = [self.currNode + x for x in self.refNeighbours]
        minFValue = np.inf
        nextNode = neighbours[0]
        for node in neighbours:
            if self.visited[node.v][node.h] == 0:
                node.gValue = self.gValue(node)
                node.hValue = self.hValue(node)
                node.fValue = kg*node.gValue + kh*node.hValue
                if node.fValue < minFValue:
                    minFValue = node.fValue
                    nextNode = node

        self.pathCost += minFValue
        return nextNode

    def aStar(self, record = 0, kg = 1, kh = 1):
        while self.currNode != self.destNode: #and self.nodeState(self.currNode) != 'gray':
            self.visited[self.currNode.v][self.currNode.h] = 1
            if record == 1:
                # cv2.circle(self.canvas, (self.currNode.h, self.currNode.v), 1, (0, 0, 255), 1)
                self.trajectory.append(self.currNode)
            leastFNode = self.getLeastFNode(kg, kh)
            self.currNode = leastFNode
        self.trajectory.append(self.currNode)

    def shortestPath(self):
        self.aStar(0, 1.1, 0.9)

        temp = self.startNode
        self.startNode = self.destNode
        self.destNode = temp
        self.visited = 1-self.visited

        self.pathCost = 0
        self.aStar(1, 0, 1)
        self.trajectory = self.trajectory[::-1]

        return self.trajectory, self.pathCost

def prepareMap(img):
    canvas = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    newImg = np.array(img)
    newImg[newImg < 127] = 0
    newImg[newImg >= 127] = 255
    newImg = cv2.erode(newImg, np.ones((50,50)))
    newImg = cv2.dilate(newImg, np.ones((30,30)))

    final = np.array(newImg)

    kernel = np.ones((50,50),np.float32)/2500
    final = cv2.filter2D(newImg,-1,kernel)

    final[img == 127] = 127
    final[newImg == 0] = 0

    # cv2.imshow('win', final)
    # cv2.waitKey(1)

    return final, canvas

if __name__ == '__main__':
    img = cv2.imread('../../generatedMap.png', 0)
    newImg, canvas = prepareMap(img)

    startNode = Node(400, 400)
    destNode = Node(650, 400)
    finder = PathFinder(startNode, destNode, newImg, canvas, 1)
    finder.shortestPath()
    cv2.imshow('win', finder.canvas)
    cv2.waitKey(0)
