import pygame
import time
import numpy as np
from typing import Tuple
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from algorithms.collision_checking import Line, Circle, Chain
from algorithms.RRR_3dof_kinematic import *
from algorithms.RRT import RRT,Map

SCREEN_SIZE = (800, 800)
FIELD_SIZE = (3.0, 3.0)
BASE_POINT = np.array([0.0, 0.0])
SCALE_FACTOR = np.array(SCREEN_SIZE)/np.array(FIELD_SIZE)
LINK_LENGHT = np.array([0.8, 0.7, 0.9])
METRICS_SCALE_FACTOR = np.linalg.norm(np.array(SCREEN_SIZE))/np.linalg.norm(np.array(FIELD_SIZE))

def draw_link(surf: pygame.display, chain: Chain):
    key_points = []
    for p in chain.points:
        lp = p*SCALE_FACTOR
        key_points.append((lp[0], lp[1],))
    # print(key_points)
    pygame.draw.lines(surf, (0,125, 185), False, key_points, width=5)
    for p in key_points:
        pygame.draw.circle(surf, (255, 80, 10), p, 5)

def draw_circle(surf: pygame.display, circle: Circle):
    pygame.draw.circle(surf, (100, 100, 100), circle.p*SCALE_FACTOR, circle.r*METRICS_SCALE_FACTOR)

chain = Chain( (np.array([0.0, 0.0]), np.array([0.3, 0.5]), np.array([0.3, 0.9]), np.array([0.6, 1.6]),) )
links_length = [1.2,0.9,1.0]
th = np.array([np.pi/4, np.pi/3, np.pi/4])

if __name__ == "__main__":


    map3Drand = Map(dim=3, obs_num=2,
        obs_size_min=0.05, 
        obs_size_max=0.1, 
        xinit=np.array([0.1, 0.3, 0.1]), 
        xgoal=np.array([1.5, 0.0, 0.5]), 
        field_range = np.array([-np.pi/2 , np.pi]), 
        links_length=LINK_LENGHT
    )

    rrt = RRT(_map=map3Drand, method="RRT-Connect", maxIter=1000000)
    rrt.Search()
    
    points = []
    for i in range(0, len(rrt.path)):
        point = get_point(rrt.path[i], LINK_LENGHT)[-1]
        points.append(point)
    data = np.array(points)

    print(rrt.path)

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
        chain = Chain(tuple([BASE_POINT] +list(get_point(th, links_length))))
        
        for ob in map3Drand.obstacles:
            draw_circle(screen, ob)
        draw_link(screen, chain)

        pygame.display.flip()
        step+=1
        time.sleep(0.1)

        if step>=len(rrt.path):
            step=0

    pygame.quit()