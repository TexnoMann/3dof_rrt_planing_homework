import numpy as np
from typing import Tuple
from scipy.optimize import fsolve

class Line:
    def __init__(self, p1: np.array, p2: np.array):
        self.__p1 = p1
        self.__p2 = p2
        self.__A = p2[1] - p1[1]
        self.__B = -(p2[0] - p1[0])
        self.__C = p1[0]*(p2[1]-p1[1]) - p1[1]*(p2[0]-p1[0])
    
    @property
    def A(self) -> float:
        return self.__A
    
    @property
    def B(self) -> float:
        return self.__B
    
    @property
    def C(self) -> float:
        return self.__C
    
    @property
    def points(self) -> Tuple[np.array, np.array]:
        return (self.__p1, self.__p2)

class Circle:
    def __init__(self, p: np.array, radius: float):
        self.__p = p
        self.__r = radius
    
    @property
    def r(self) -> float:
        return self.__r
    
    @property
    def p(self) -> float:
        return self.__p

class Chain:
    def __init__(self, points: Tuple[np.array]):
        self.__points = points
        self.__lines = [Line(self.__points[i-1], self.__points[i]) for i in range(1, len(self.__points))]
    
    @property
    def lines(self):
        return self.__lines

    @property
    def points(self):
        return self.__points


def check_line_circle_collision(line: Line, circle: Circle) -> bool:
    dist = np.abs(line.A*circle.p[0] + line.B*circle.p[1] + line.C)/ np.sqrt(line.A**2 + line.B**2)
    if circle.r >= dist:
        return True
    else:
        return False

def check_chain_circle_collision(chain: Chain, circle: Circle) -> bool:
    for line in chain.lines:
        if check_line_circle_collision(line, circle):
            return True
    return False

def check_point_circle_collision(point: np.array, circle: Circle) -> bool:
    if np.linalg.norm(point - circle.p) <= circle.r:
        return True
    return False



if __name__ == "__main__":
    l1 = Line(np.array([0.0, 0.0]), np.array([.9, 0.0]))
    c1 = Circle(np.array([0.0, 0.0]), 1.0)
    result = check_line_circle_collision(l1, c1)
    print(result)

    result2 = check_chain_circle_collision( Chain((np.array([-1.0, 1.0]), np.array([1.0, 1.0]), np.array([0.0, 0.9]), )), c1)
    print(result2)