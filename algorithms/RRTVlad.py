
import copy
import math
import platform
import random
import time

import matplotlib.pyplot as plt
import numpy as np
import numpy.linalg as lng

from .collision_checking import check_line_circle_collision, check_chain_circle_tuple_collision, check_point_circle_collision, Circle, Line, Chain, BASE_POINT
from .RRR_3dof_kinematic import *

import matplotlib.pyplot as plt

class RRT:
    def __init__(self,_map=None,method="RRT-Connect",maxIter=5000):
        if _map == None:
            self.map = Map()
        else: self.map = _map
        self.method = method
        self.trees = []
        self.ninit = Node(self.map.xinit,cost=0,lcost=0)
        self.ngoal = Node(self.map.xgoal)
        self.dimension = self.map.dimension
        self.prob = 0.1
        self.maxIter = maxIter
        self.stepSize = 0.05
        self.DISCRETE = 0.005
        self.path = []
        self.pathCost = float('inf')

        # *
        self.nearDis = 2
    
    def Search(self):
        ret = False
        print("method: ",self.method)
        # Search
        start_time = time.time()
        if self.method == "RRT":
            ret = self.rrtSearch()
        elif self.method == "RRT-Connect":
            self.prob = 0
            ret = self.rrtConnectSearch()
        elif self.method == "RRT*":
            ret = self.rrtStarSearch()
        elif self.method == "Informed RRT*":
            ret = self.InformedRRTStarSearch()
        else:
            print("Unsupported Method, please choose one of:")
            print("RRT, RRT-Connect")
        end_time = time.time()
        if not ret:
            print("Solve Failed")
            return False
        
        print("Get!")
        # getPath
        self.getPath()
        print("path cost(distance): ", self.pathCost," steps: ",len(self.path)," time_use: ",end_time-start_time)

        return ret
    
    def getPath(self):
        # knowing that no more than 2 trees
        t = self.trees[0]
        n = t.nodes[-1]
        sum = 0
        while n.parent:
            self.path.append(n.x)
            n = n.parent
            sum += n.lcost
        self.path.append(n.x)
        if len(self.trees)>1:
            nl = n
            n = self.trees[2].root
            sum += Node.distancenn(nl,n)
            while n.parent:
                self.path.insert(0,n.parent.x)
                n = n.parent
                sum += n.lcost
                
        self.pathCost = sum

    def rrtSearch(self):
        tree = Tree(self.ninit)
        self.trees.append(tree)
        for i in range(self.maxIter):
            xrand = self.SampleRandomFreeFast()
            nnearest,dis = tree.getNearest(xrand)
            nnew = self.Extend(nnearest,xrand)
            if nnew!=None:
                tree.addNode(nnew)
                if(Node.distancenn(nnew,self.ngoal)<self.stepSize):
                    tree.addNode(self.ngoal,parent=nnew)
                    print("iter ",i," find!")
                    return True
        return False

    def rrtConnectSearch(self):
        treeI = Tree(nroot=self.ninit)
        treeG = Tree(nroot=self.ngoal)
        self.trees.append(treeI)
        self.trees.append(treeG)
        tree1 = treeI # the less points one
        tree2 = treeG
        for i in range(self.maxIter):
            # print(i)
            xrand = self.SampleRandomFreeFast()
            # print(xrand)
            nnearest,dis = tree1.getNearest(xrand)
            nnew = self.Extend(nnearest,xrand)
            if nnew!=None:
                tree1.addNode(nnew)
                nnears = tree2.getNearby(nnew,self.stepSize)
                if len(nnears):
                    ncon = nnears[0] # or chose the nearest?
                    connectTree = Tree(ncon)
                    self.trees.append(connectTree)
                    print("iter ",i," find!")
                    return True
            
                # another tree
                # directly forward to nnew
                nnearest,dis = tree2.getNearest(nnew.x)
                nnew2 = self.Extend(nnearest,nnew.x)
                while nnew2:
                    tree2.addNode(nnew2)
                    nnears = tree1.getNearby(nnew2,self.stepSize)
                    if len(nnears):
                        ncon = nnears[0]
                        connectTree = Tree(ncon)
                        self.trees = [tree2,tree1,connectTree]
                        print("iter ",i," find!")
                        return True
                    nnearest = nnew2
                    nnew2 = self.Extend(nnearest,nnew.x)
                    

                # check the size,
                # let the less one to random spare
                if tree1.length()>tree2.length():
                    temptree = tree1
                    tree1 = tree2
                    tree2 = temptree

        return False

    def rrtStarSearch(self):
        tree = Tree(self.ninit)
        self.trees.append(tree)
        for i in range(self.maxIter):
            xrand = self.SampleRandomFreeFast()
            nnearest,dis = tree.getNearest(xrand)
            nnew = self.Extend(nnearest,xrand)
            if nnew!=None:
                tree.addNode(nnew)

                # adjust
                self.reParent(nnew,tree)
                self.reWire(nnew,tree)

                if(Node.distancenn(nnew,self.ngoal)<self.stepSize):
                    tree.addNode(self.ngoal,parent=nnew)
                    print("iter ",i," find!")
                    return True
        return False

    def InformedRRTStarSearch(self):
        ret = False
        tree = Tree(self.ninit)
        self.trees.append(tree)

        # max length we expect to find in our 'informed' sample space, starts as infinite
        cBest = float('inf')
        pathLen = float('inf')
        solutionSet = set()
        path = None

        # Computing the sampling space
        cMin = Node.distancenn(self.ninit,self.ngoal)
        xCenter = np.array(list((self.ninit.x+self.ngoal.x)/2))
        a1 = np.transpose([np.array((self.ngoal.x-self.ninit.x)/cMin)])
        
        # TODO
        if self.dimension == 2:
            etheta = math.atan2(a1[1], a1[0])

        # first column of idenity matrix transposed
        id1_t = np.array([1.0]+[0.0,]*(self.dimension-1)).reshape(1,self.dimension)
        M = a1 @ id1_t
        U, S, Vh = np.linalg.svd(M, 1, 1)
        C = np.dot(np.dot(U, 
            np.diag([1.0,]*(self.dimension-1)+[np.linalg.det(U) * np.linalg.det(np.transpose(Vh))]))
            , Vh)
        
        new_best = False

        for i in range(self.maxIter):
            # Sample
            if cBest < float('inf'):
                # informed sample
                if new_best:
                    new_best = False
                    # debug
                    print(cBest,cMin)
                    r = [cBest / 2.0]+[math.sqrt(cBest**2 - cMin**2) / 2.0,]*(self.dimension-1)
                    L = np.diag(r)

                xBall = self.sampleUnitBall()
                xrand = np.dot(np.dot(C, L), xBall) + xCenter  
            else: xrand = self.SampleRandomFreeFast()

            nnearest,dis = tree.getNearest(xrand)
            nnew = self.Extend(nnearest,xrand)
            if nnew!=None:
                tree.addNode(nnew)

                # adjust
                self.reParent(nnew,tree)
                self.reWire(nnew,tree)

                if(Node.distancenn(nnew,self.ngoal)<self.stepSize):
                    tree.addNode(self.ngoal,parent=nnew)
                    print("iter ",i," find!")
                    ret = True
                    oldCost = self.pathCost
                    self.getPath()
                    print("Cost: ",self.pathCost)
                    #TODO what if doesn't improve?
                    if oldCost > self.pathCost:
                        cBest = self.pathCost
                        new_best = True
        return ret

    def _CollisionPoint(self, th): 
        ch = Chain(tuple([BASE_POINT] +list(get_point(th, self.map.links_length))))
        return check_chain_circle_tuple_collision(ch, self.map.obstacles)

    def _CollisionLine(self,x1,x2):
        dis = Node.distancexx(x1,x2)
        if dis<self.DISCRETE:
            return False
        nums = int(dis/self.DISCRETE)
        direction = (np.array(x2)-np.array(x1))/Node.distancexx(x1,x2)
        for i in range(nums+1):
            x = np.add(x1 , i*self.DISCRETE*direction)
            if self._CollisionPoint(x): return True
        if self._CollisionPoint(x2): return True
        return False

    def Nearest(self,xto,nodes=None):
        if nodes == None:
            nodes = self.trees[0].nodes
        dis = float('inf')
        nnearest = None
        for node in nodes:
            curDis = Node.distancenx(node,xto)
            if curDis < dis:
                dis = curDis
                nnearest = node
        return nnearest

    def Extend(self,nnearest,xrand,step=None):

        if not step:
            step = self.stepSize
        dis = Node.distancenx(nnearest,xrand)
        if dis<step:
            xnew = xrand
        else:
            dis = step
            xnew = np.array(nnearest.x) + step*(np.array(xrand)-np.array(nnearest.x))/Node.distancenx(nnearest,xrand)
        if self._CollisionPoint(xnew):
            return None
        if self._CollisionLine(xnew,nnearest.x):
            return None
        nnew = Node(xnew,parent=nnearest,lcost=dis)
        return nnew
    
    def reParent(self,node,tree):
        # TODO: check node in tree
        nears = tree.getNearby(node)
        for n in nears:
            if self._CollisionLine(n.x,node.x):
                continue
            newl = Node.distancenn(n,node)
            if n.cost + newl < node.cost:
                node.parent = n
                node.lcost = newl
                node.cost = n.cost + newl

    # what if combine the both?
    def reWire(self,node,tree):
        nears = tree.getNearby(node)
        for n in nears:
            if self._CollisionLine(n.x,node.x):
                continue
            newl = Node.distancenn(n,node)
            if node.cost + newl < n.cost:
                n.parent = node
                n.lcost = newl
                n.cost = node.cost + newl


    def SampleRandomFreeFast(self):
        r = random.random()
        if r<self.prob:
            return self.map.xgoal
        else:
            ret = self._SampleRandom()
            while self._CollisionPoint(ret):
                ret = self._SampleRandom()
        return ret

    def _SampleRandom(self):
        ret = []
        for i in range(self.dimension):
            ret.append(random.random()*(self.map.field_range[1]-self.map.field_range[0]) + self.map.field_range[0] )
        # print(ret)
        return ret
    

    def drawTree(self,tree=None,color='g'):
        """
        若不指明，将所有树都画出来
        """
        if tree==None:
            trees = self.trees
        else:
            trees = [tree]
        if self.dimension == 2:
            for t in trees:
                for node in t.nodes:
                    if node.parent is not None:
                        plt.plot([node.x[0], node.parent.x[0]], 
                        [node.x[1], node.parent.x[1]], '-'+color)
        elif self.dimension == 3:
            pass

    def drawPath(self):
        if self.dimension == 2:
            plt.plot([x for (x, y) in self.path], [y for (x, y) in self.path], '-r')  
        elif self.dimension == 3:
            pass




class Node:
    def __init__(self,x,lcost=0.0,cost=float('inf'),parent=None):
        self.x = np.array(x)
        self.lcost = lcost # from parent
        self.cost = cost # from init
        self.parent = parent
        if parent:
            self.cost = self.lcost+parent.cost
        # self.children = children

    @staticmethod
    def distancenn(n1,n2):
        return lng.norm(np.array(n1.x)-np.array(n2.x))
    @staticmethod
    def distancenx(n,x):
        return lng.norm(n.x-np.array(x))
    @staticmethod
    def distancexx(x1,x2):
        return lng.norm(np.array(x1)-np.array(x2))

#TODO
class Tree:
    def __init__(self,nroot):
        self.root = nroot
        self.nodes = [nroot]

    def addNodeFromX(self,x,parent):
        self.nodes.append(Node(np.array(x),parent=parent))
    
    def addNode(self,n,parent=None):
        if parent:
            n.parent = parent
            n.cost = n.lcost + parent.cost
        self.nodes.append(n)
    
    def length(self):
        return len(self.nodes)
    
    def getNearest(self,x):
        dis = float('inf')
        nnearest = None
        for node in self.nodes:
            curDis = Node.distancenx(node,x)
            if curDis < dis:
                dis = curDis
                nnearest = node
        return nnearest,dis
    
    def getNearby(self,nto,dis=None):
        ret = []
        print(dis)
        if dis==None:
            dis = 20.0 * math.sqrt((math.log(self.length()) / self.length()))
        print(dis)
        for n in self.nodes:
            if Node.distancenn(nto,n)<dis:
                ret.append(n)
        return ret
    


class Map:
    def __init__(self,
            dim=3, obs_num=10,
            obs_size_min=0.1, 
            obs_size_max=0.3, 
            xinit=np.array([0, 0, 0]), 
            xgoal=np.array([np.pi/2,np.pi/4, np.pi/4]), 
            field_range = np.array([-np.pi , np.pi]), 
            links_length=np.array([0.6, 0.6, 0.8])
        ):
        self.dimension = dim
        self.xinit = xinit
        self.xgoal = xgoal
        self.obstacles = []
        self.field_range = field_range
        self.obs_size_min = obs_size_min
        self.obs_size_max = obs_size_max
        self.obs_num = obs_num
        self.links_length = links_length
        for i in range(obs_num):
            r = random.random()*(obs_size_max-obs_size_min)+obs_size_min
            print(r)
            p = np.random.rand(2)*3.0
            self.obstacles.append(Circle(p, r))


def main():
    print("Start rrt planning")
    LINK_LENGHT = np.array([0.6, 0.6, 0.8])
    # create map
    map2Drand = Map()
    map3Drand = Map(dim=3, obs_num=5,
        obs_size_min=0.05, 
        obs_size_max=0.2, 
        xinit=np.array([0.1, 0.3, 0.1]), 
        xgoal=np.array([0.5, 0.0, 0.5]), 
        field_range = np.array([-np.pi/2 , np.pi]), 
        links_length=LINK_LENGHT
    )
    
    rrt = RRT(_map=map3Drand,method="RRT-Connect", maxIter=100000)
    rrt.Search()
    
    points = []
    for i in range(0, len(rrt.path)):
        point = get_point(rrt.path[i], LINK_LENGHT)[-1]
        points.append(point)
    data = np.array(points)
    print(data)
    plt.plot(data[:,0], data[:,1])
    plt.show()
    


if __name__ == '__main__':
    main()