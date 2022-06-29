import random
from typing import Tuple
import numpy as np
import easygraph as eg
from collision_checking import *
from RRR_3dof_kinematic import *


class RRT_plane:
    def __init__(self, start_point: np.array, goal_point: np.array, circle_p: np.array, circle_r: float, delta: float = 0.1, recursion: int = 100,
                 links_lengths: np.array = np.array([0.5, 0.5, 0.5]), BASE_POINT: np.array = np.array([0.0, 0.0]), map: np.array = np.linspace(0,3,int(3.0/0.005))):

        self.start = start_point
        self.goal = goal_point
        self.delta = delta
        self.K = recursion
        self.l = links_lengths
        self.BASE_POINT = BASE_POINT
        self.circle_p = circle_p
        self.circle_r = circle_r
        self.map = map


    def tree_create(self) -> np.array:
        self.G_points = np.array([self.start])
        self.G = np.array([[0.0]])

    def RRT_processe(self) -> bool:
        state = False
        k = 0
        self.tree_create()
        theta0 = np.array([0.0, 0.0, 0.0])
        q_new = np.array([0.0, 0.0])
        while (k != self.K) and (state == False):
            print('iteration: 'k)
            q_rand, theta0 = self.random_generate(theta0)
            print('q_rand: ', q_rand)
            ok, q_near = self.nearest_vertex(q_rand)
            print(ok, 'q_near: ', q_near)
            if ok:
                state = self.new_configuration(q_near, q_rand)
            print('G_points: ', self.G_points)
            print('G: ', self.G[0:-1,0])
            k += 1

        return state

    def random_generate(self, theta0: np.array = np.array([0.0, 0.0, 0.0])) -> np.array:
        qx = random.choice(self.map)
        qy = random.choice(self.map)
        th = get_angles2(np.array([qx, qy]), self.l, theta0)

        chain = Chain(tuple([self.BASE_POINT] + list(get_point(th, self.l))))
        c1 = Circle(self.circle_p, self.circle_r)

        if check_chain_circle_collision(chain, c1):
            new_q, th = self.random_generate()
        else:
            new_q = np.array([qx, qy])
        return new_q, th

    def init_point():
        pass

    def nearest_vertex(self, q_rand):
        c1 = Circle(self.circle_p, self.circle_r)
        dist = 1000
        q_near = self.start
        ok = False
        for i in range(0, len(self.G_points)):
            q_near_i = self.G_points[i]
            dist_i = np.sqrt((q_rand[0] - q_near_i[0])**2 + (q_rand[1] - q_near_i[1])**2)
            if dist_i < dist:
                line = Line(q_near_i, q_rand)
                if not check_line_circle_collision(line, c1):
                    dist = dist_i
                    q_near = q_near_i
                    ok = True
        return ok, q_near

    def new_configuration(self, q_near, q_new):
        self.add_vertex(q_new)
        self.add_edge(q_near, q_new)
        state = False
        if np.sqrt((q_new[0]-self.goal[0])**2+(q_new[1]-self.goal[1])**2) <= self.delta:
            state = True

        return state

    def add_vertex(self, q_new):
        # print('add_vertex, q_new:', q_new)
        # print('add_vertex, G_points:', self.G_points)
        self.G_points = np.append(self.G_points, [q_new], axis=0)
        # print('G: ', self.G)
        self.G = np.insert(self.G, len(self.G_points)-1, 0, axis = 1)
        self.G = np.insert(self.G, len(self.G_points)-1, 0, axis = 0)
        # print('G: ', self.G)
        # self.G = np.append(self.G, np.array([np.linspace(0,0,len(self.G_points)-1)]), axis=1)
        # self.G = np.append(self.G, np.array([np.linspace(0,0,len(self.G_points))]), axis=0)

    def add_edge(self, q_near, q_new):
        # print('q_near: ', q_near, ' q_new: ', q_new)
        # print('G_points', self.G_points)
        index_near = np.where(self.G_points == q_near)[0][0]
        self.G[index_near, -1] = np.sqrt((q_new[0] - q_near[0])**2 + (q_new[1] - q_near[1])**2)
        self.G[-1, index_near] = np.sqrt((q_new[0] - q_near[0])**2 + (q_new[1] - q_near[1])**2)


if __name__ == "__main__":
    start = np.array([1.0, 1.0])
    goal = np.array([0.75, 1.1])
    RRT_model = RRT_plane(start, goal, np.array([1.6, 1.5]), 0.25)
    RRT_model.RRT_processe()
    # print(RRT_model.random_generate())
>>>>>>> 21173fa5f06f58c3e79ea40ab040a52ad230cc67
