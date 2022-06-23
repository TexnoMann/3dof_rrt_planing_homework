import pygame
import numpy as np
from typing import Tuple
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from algorithms.collision_checking import Line, Circle, Chain

SCREEN_SIZE = (800, 800)
FIELD_SIZE = (3.0, 3.0)
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
c1 = Circle(np.array([0.6, 1.5]), 0.4)

if __name__ == "__main__":
    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE)
    screen.fill((255, 255, 255))

    running = True
    while running:

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
        
        draw_circle(screen, c1)
        draw_link(screen, chain)
        pygame.display.flip()

    pygame.quit()