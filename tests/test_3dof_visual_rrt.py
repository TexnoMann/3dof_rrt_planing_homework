import pygame
import time
import numpy as np
from typing import Tuple
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import axes3d
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from algorithms.collision_checking import Line, Circle, Chain, check_chain_circle_tuple_collision
from algorithms.RRR_3dof_kinematic import *
from algorithms.RRTVlad import RRT,Map

SCREEN_SIZE = (800, 800)
FIELD_SIZE = (3.0, 3.0)
BASE_POINT = np.array([0.0, 0.0])
SCALE_FACTOR = np.array(SCREEN_SIZE)/np.array(FIELD_SIZE)
LINK_LENGHT = np.array([0.8, 0.7, 0.9])
METRICS_SCALE_FACTOR = np.linalg.norm(np.array(SCREEN_SIZE))/np.linalg.norm(np.array(FIELD_SIZE))

def draw_chain(surf: pygame.display, chain: Chain):
    key_points = []
    for p in chain.points:
        lp = p*SCALE_FACTOR
        key_points.append((lp[0], lp[1],))
    # print(key_points)
    pygame.draw.lines(surf, (0,125, 185), False, key_points, width=5)
    for p in key_points:
        pygame.draw.circle(surf, (255, 80, 10), p, 5)

def draw_circle(surf: pygame.display, circle: Circle, color: list = (100, 100, 100),):
    pygame.draw.circle(surf, color, circle.p*SCALE_FACTOR, circle.r*METRICS_SCALE_FACTOR)

th = np.array([np.pi/4, np.pi/3, np.pi/4])

if __name__ == "__main__":


<<<<<<< HEAD
    map3Drand = Map(dim=3, obs_num=2,
        obs_size_min=0.1,
        obs_size_max=0.75,
        xinit=np.array([0.1, 0.3, 0.1]),
        xgoal=np.array([1.5, 0.0, -0.5]),
        field_range = np.array([-np.pi , np.pi]),
=======
    map3Drand = Map(dim=3, obs_num=3,
        obs_size_min=0.2, 
        obs_size_max=0.4, 
        xinit=np.array([0.1, 0.3, 0.1]), 
        xgoal=np.array([1.5, 0.0, -0.5]), 
        field_range = np.array([-np.pi , np.pi]), 
>>>>>>> af32195eeffac54cd2891554e22d8c59541b3628
        links_length=LINK_LENGHT
    )

    rrt = RRT(_map=map3Drand, method="RRT-Connect", maxIter=1000000)
    rrt.Search()

    points = []
    for i in range(0, len(rrt.path)):
        point = get_point(rrt.path[i], LINK_LENGHT)[-1]
        points.append(point)
    data = np.array(points)

    start_point = Circle(points[0], 0.025)
    goal_point = Circle(points[-1], 0.025)


    print(rrt.path)
    print(len(rrt.path))
    print(rrt.path[5])

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('theta_0')
    ax.set_xlabel('theta_1')
    ax.set_xlabel('theta_2')
    ax.scatter(rrt.path[0][0], rrt.path[0][1], rrt.path[0][2], label='Start point')
    for i in range(len(rrt.path)-1):
        line = np.array([rrt.path[i], rrt.path[i+1]])
        print(line)
        ax.plot(line[:,0], line[:,1], line[:,2])
    ax.scatter(rrt.path[-1][0], rrt.path[-1][1], rrt.path[-1][2], label='Goal point')
    ax.legend()
    plt.show()

    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE)
    screen.fill((255, 255, 255))

    running = True
    time_start = time.time()
    step =0
    while running:
        screen.fill((255, 255, 255))

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        th = rrt.path[step]
        # print(th)
        ch = Chain(tuple([BASE_POINT] +list(get_point(th, LINK_LENGHT))))

        for ob in map3Drand.obstacles:
            draw_circle(screen, ob)
        draw_chain(screen, ch)

        draw_circle(screen, start_point, tuple([169,59,59]))
        draw_circle(screen, goal_point, tuple([0,204,102]))

        print(check_chain_circle_tuple_collision(ch, map3Drand.obstacles) )

        pygame.display.flip()
        step+=1
        time.sleep(0.1)

        if step>=len(rrt.path):
            step=0

    pygame.quit()
