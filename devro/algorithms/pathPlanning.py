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

    @classmethod
    def fromList(cls, coord):
        return cls(int(coord[0]), int(coord[1]))

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


class Astar():
    def __init__(self, startCoord, destCoord, map_):
        self.currNode = Node.fromList(startCoord)
        self.startNode = Node.fromList(startCoord)
        self.map_ = map_
        self.destNode = Node.fromList(destCoord)
        self.trajectory = []
        self.visited = np.zeros_like(map_)
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
        ##
        return (255 - self.map_[node.v][node.h])/255

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

    def astar(self, record = 0, kg = 1, kh = 1):
        while self.currNode != self.destNode: #and self.nodeState(self.currNode) != 'gray':
            self.visited[self.currNode.v][self.currNode.h] = 1
            if record == 1:
                self.trajectory.append(self.currNode)
            leastFNode = self.getLeastFNode(kg, kh)
            self.currNode = leastFNode
        self.trajectory.append(self.currNode)

    def find(self):
        self.astar(1, 1.1, 0.9)
        for node in self.trajectory:
            self.map_ = cv2.circle(self.map_, (int(node.v), int(node.h)), 1, (0,0,0), -1)

        # temp = self.startNode
        # self.startNode = self.destNode
        # self.destNode = temp
        # self.visited = 1-self.visited
        #
        # self.pathCost = 0
        # self.astar(1, 0, 1)
        # self.trajectory = self.trajectory[::-1]

        return self.trajectory, self.pathCost

if __name__ == '__main__':
    pass
