import random
from typing import Tuple
import numpy as np
import easygraph as eg
from collision_checking import *
from RRR_3dof_kinematic import *
import matplotlib.pyplot as plt


class RRT_plane:
    def __init__(self, start_point: np.array, goal_point: np.array, circle_p: np.array, circle_r: np.array, delta: float = 0.2, recursion: int = 100,
                 links_lengths: np.array = np.array([0.95, 0.95, 0.95]), BASE_POINT: np.array = np.array([0.0, 0.0]), map: np.array = np.linspace(0,3,int(3.0/0.005))):

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

        th, state = get_angles2(self.start, self.l, theta0)
        if state == False:
            print('Start point is inaccessible')
        maniulation_position = get_point(th, self.l)
        maniulation_position = np.insert(maniulation_position, 0, 0.0, axis = 0)
        print(maniulation_position)
        plt.plot(maniulation_position[:,0], maniulation_position[:,1])

        theta0 = np.array([0.0, 0.0, 0.0])
        th, state = get_angles2(self.goal, self.l, theta0)
        chain = Chain(tuple([self.BASE_POINT] + list(get_point(th, self.l))))
        crc = []
        for c in range(len(self.circle_r)):
            crc.append(Circle(self.circle_p[c], self.circle_r[c]))
        o = 0
        while check_chain_circle_tuple_collision(chain, crc) or (not state):
            print('Goal point check, iter: ' , o)
            th, state = get_angles2(self.goal, self.l, th)
            chain = Chain(tuple([self.BASE_POINT] + list(get_point(th, self.l))))
            if o > 10:
                break
            o += 1
        if state == False:
            print('Goal point is inaccessible')
        maniulation_position = get_point(th, self.l)
        maniulation_position = np.insert(maniulation_position, 0, 0.0, axis = 0)
        plt.plot(maniulation_position[:,0], maniulation_position[:,1])

        state = False
        while ((k != self.K) and (state == False)):
            s = 0
            print('iteration: ', k)
            q_rand, theta0, s = self.random_generate(s, theta0)
            plt.scatter(q_rand[0], q_rand[1])
            print('q_rand: ', q_rand)
            ok, q_near = self.nearest_vertex(q_rand)
            # print('ok', ok_1)
            # print(ok, 'q_near: ', q_near)
            if ok:
                print('Nearest finded')
                state = self.new_configuration(q_near, q_rand)
                maniulation_position = get_point(theta0, self.l)
                maniulation_position = np.insert(maniulation_position, 0, 0.0, axis = 0)
                print('Joint positions: ', maniulation_position)
                print('In theta: ', theta0)

                plt.plot(maniulation_position[:,0], maniulation_position[:,1])
            print('G_points: ', self.G_points)
            print('G: ', self.G[0:-1,0])
            k += 1
            plt.savefig('./tmp_img/iter_'+ str(k) +'.png')

        return state

    def random_generate(self, s: int = 0, theta0: np.array = np.array([0.0, 0.0, 0.0])) -> np.array:
        s += 1
        print('random value ', s, ' iteration')
        qx = random.choice(self.map)
        qy = random.choice(self.map)
        th, state = get_angles2(np.array([qx, qy]), self.l, theta0)
        print(state)
        chain = Chain(tuple([self.BASE_POINT] + list(get_point(th, self.l))))
        crc = []
        for c in range(len(self.circle_r)):
            crc.append(Circle(self.circle_p[c], self.circle_r[c]))

        if check_chain_circle_tuple_collision(chain, crc) or (not state):
            new_q, th, s = self.random_generate(s, th)
        else:
            new_q = np.array([qx, qy])
        return new_q, th, s

    def check_finish(self, q_new):
        ok = False
        line = Line(self.goal, q_new)
        crc = []
        for c in range(len(self.circle_r)):
            crc.append(Circle(self.circle_p[c], self.circle_r[c]))
            print(crc)
        if not check_line_circle_tuple_collision(line, crc):
            ok = True
            plt.plot([self.goal[0], q_new[0]], [self.goal[1], q_new[1]])
        return ok

    def nearest_vertex(self, q_rand):
        crc = []
        for c in range(len(self.circle_r)):
            crc.append(Circle(self.circle_p[c], self.circle_r[c]))
        dist = 1000
        q_near = self.start
        ok = False
        for i in range(0, len(self.G_points)):
            q_near_i = self.G_points[i]
            dist_i = np.sqrt((q_rand[0] - q_near_i[0])**2 + (q_rand[1] - q_near_i[1])**2)
            if dist_i < dist:
                line = Line(q_near_i, q_rand)
                if not check_line_circle_tuple_collision(line, crc):
                    dist = dist_i
                    q_near = q_near_i
                    ok = True
        return ok, q_near

    def new_configuration(self, q_near, q_new):
        self.add_vertex(q_new)
        self.add_edge(q_near, q_new)
        plt.plot([q_near[0], q_new[0]], [q_near[1], q_new[1]])
        state = False
        # if np.sqrt((q_new[0]-self.goal[0])**2+(q_new[1]-self.goal[1])**2) <= self.delta:
        #     state = True
        if self.check_finish(q_new):
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
        # plt.plot([q_near[0], q_new[0]], [q_near[1], q_new[1]])
        # print('q_near: ', q_near, ' q_new: ', q_new)
        # print('G_points', self.G_points)
        index_near = np.where(self.G_points == q_near)[0][0]
        self.G[index_near, -1] = np.sqrt((q_new[0] - q_near[0])**2 + (q_new[1] - q_near[1])**2)
        self.G[-1, index_near] = np.sqrt((q_new[0] - q_near[0])**2 + (q_new[1] - q_near[1])**2)


if __name__ == "__main__":
    start = np.array([0.0, 1.5])
    goal = np.array([1.75, 2.1])
    plt.figure(1)
    plt.xlim(0, 3)
    plt.ylim(0, 3)
    plt.scatter(start[0], start[1], c="r")
    plt.scatter(goal[0], goal[1], c="g")
    circle_1 = plt.Circle((1.5, 1.0), 0.5, color="r", alpha = 0.5)
    circle_2 = plt.Circle((2.5, 1.75), 0.25, color="r", alpha = 0.5)
    circle_3 = plt.Circle((0.8, 2.6), 0.45, color="r", alpha = 0.5)
    plt.gca().add_patch(circle_1)
    plt.gca().add_patch(circle_2)
    plt.gca().add_patch(circle_3)
    # plt.show()

    RRT_model = RRT_plane(start, goal, np.array([[1.5, 0.5],[2.5, 1.75],[0.8, 2.6]]), [0.45, 0.25, 0.45])
    # RRT_model = RRT_plane(start, goal, np.array([[0.8, 2.6]]), [0.45])
    RRT_model.RRT_processe()

    plt.show()
