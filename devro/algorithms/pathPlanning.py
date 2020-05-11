import numpy as np
import cv2

class Node():
    def __init__(self, v, h):
        self.v = v
        self.h = h
        self.fValue = np.inf
        self.gValue = np.inf
        self.hValue = np.inf
        self.visited = 0
        self.parent = None

    @classmethod
    def fromList(cls, coord):
        return cls(int(coord[1]), int(coord[0]))

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
    def __init__(self, startCoord, destCoord, map_, optimize = False):
        self.currNode = Node.fromList(startCoord)
        self.startNode = Node.fromList(startCoord)
        self.map_ = map_
        self.destNode = Node.fromList(destCoord)
        self.open_ = []
        self.trajectory = []
        self.visited = np.zeros_like(map_)
        self.refNeighbours = []
        self.k = 3 if optimize is True else 1
        for m in [0, -self.k, self.k]:
            for n in [0, -self.k, self.k]:
                if m != 0 or n != 0:
                    self.refNeighbours.append(Node(m,n))

    def hValue(self, node):
        normFactor = max(abs(self.startNode.v-self.destNode.v), abs(self.startNode.h-self.destNode.h))
        return max(abs(node.v-self.destNode.v), abs(node.h-self.destNode.h))/normFactor

    def gValue(self, node):
        return (255 - self.distmap[node.v][node.h])/255

    def getLeastFNode(self):
        return min(self.open_, key = lambda x: x.fValue)

    def astar(self, record = 0, kg = 1, kh = 1):
        self.currNode.fValue = 0
        self.open_.append(self.currNode)
        while self.currNode % self.destNode > self.k:
            self.visited[self.currNode.v][self.currNode.h] = 1
            self.open_.pop(self.open_.index(self.currNode))
            if record == 1:
                self.trajectory.append(self.currNode)

            neighbours = [self.currNode + x for x in self.refNeighbours]
            for node in neighbours:
                if self.visited[node.v][node.h] == 0:
                    node.gValue = self.gValue(node)
                    node.hValue = self.hValue(node)
                    node.fValue = kg*node.gValue + kh*node.hValue
                    if node in self.open_:
                        j = self.open_.index(node)
                        if self.open_[j].fValue > node.fValue:
                            self.open_.pop(j)
                        else:
                            continue
                    node.parent = self.currNode
                    self.open_.append(node)
            self.currNode = self.getLeastFNode()

        #backtracking
        node = self.currNode
        while node is not None:
            self.trajectory.append((node.v, node.h))
            node = node.parent

        self.trajectory = self.trajectory[::-1]


    def find(self):
        self.distmap = self.prepareMap(self.map_)
        self.astar(0, 1.1, 0.9)

        for node in self.trajectory:
            self.map_ = cv2.circle(self.map_, (int(node[1]), int(node[0])), 1, (0,0,0), -1)

        return self.trajectory

    def prepareMap(self, img):
        newImg = np.asarray(img, np.uint8)
        newImg[newImg < 127] = 0
        newImg[newImg >= 127] = 255
        # newImg = cv2.erode(newImg, np.ones((50,50)))
        # newImg = cv2.dilate(newImg, np.ones((30,30)))

        final = np.asarray(newImg, np.uint8)

        kernel = np.ones((50,50),np.float32)/2500
        final = cv2.filter2D(newImg,-1,kernel)

        final[newImg == 0] = 0

        return final

if __name__ == '__main__':
    pass
