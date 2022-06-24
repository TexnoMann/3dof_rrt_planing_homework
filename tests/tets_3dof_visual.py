import pygame
import time
import numpy as np
from typing import Tuple
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.collision_checking import Line, Circle, Chain
from algorithms.RRR_3dof_kinematic import *

SCREEN_SIZE = (800, 800)
FIELD_SIZE = (3.0, 3.0)
BASE_POINT = np.array([0.0, 0.0])
SCALE_FACTOR = np.array(SCREEN_SIZE)/np.array(FIELD_SIZE)
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

c1 = Circle(np.array([0.6, 1.5]), 0.4)

if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE)
    screen.fill((255, 255, 255))

    running = True
    time_start = time.time()
    while running:
        screen.fill((255, 255, 255))
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        pos = pygame.mouse.get_pos()
        pressed1 = pygame.mouse.get_pressed()[0]
        # Check if rectangle collided with pos and if the left mouse button was pressed
        if pressed1:
            print("You have opened a chest!")
            th = get_angles2(th, pos/SCALE_FACTOR, links_length)

        if th is None:
            break

        chain = Chain(tuple([BASE_POINT] +list(get_point(th, links_length))))

        draw_circle(screen, c1)
        draw_link(screen, chain)
        pygame.display.flip()
        time.sleep(0.01)

    pygame.quit()