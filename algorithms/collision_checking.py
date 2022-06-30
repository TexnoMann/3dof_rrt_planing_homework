import numpy as np
from typing import Tuple
from scipy.optimize import fsolve
# from __future__ import division

BASE_POINT = np.array([0.0, 0.0])

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


# def check_line_circle_collision(line: Line, circle: Circle) -> bool:
#     dist = np.abs(line.A*circle.p[0] + line.B*circle.p[1] + line.C)/ np.sqrt(line.A**2 + line.B**2)
#     if circle.r >= dist:
#         return True
#     else:
#         return False

def solve_polinom(a, b, c):

  D = b**2 - 4*a*c
  if D > 0:
    x1 = (-b + np.sqrt(D))/2/a
    x2 = (-b - np.sqrt(D))/2/a
    return 2, np.array([x1, x2])
  if D == 0:
    x = -b/2/a
    return 1, x
  else:
    return 0

# a, b, c - массивы координат точек a и b (конечная и начальная точка отрезка) и координаты центра окружности
def check_line_circle_collision(line: Line, circle: Circle) -> bool:
    points = line.points
    a = points[0]
    b = points[1]
    r = circle.r
    circle = circle.p

    # print('a: ', a)
    # print('b: ', b)
    # print('r: ', r)
    # print('circle: ', circle)

    x_ab = a[0] - b[0]
    y_ab = a[1] - b[1]
    if y_ab != 0 and x_ab != 0:
        C = circle[0] - (b[0]*y_ab - b[1]*x_ab)/y_ab
        try:
            [n, y_root] = solve_polinom((x_ab/y_ab)**2+1, -2*(C*x_ab/y_ab + circle[1]), C**2 + circle[1]**2 - r**2)
        except:
            n = solve_polinom((x_ab/y_ab)**2+1, -2*(C*x_ab/y_ab + circle[1]), C**2 + circle[1]**2 - r**2)
      # если получилась вертикальная прямая
    elif x_ab != 0:
        try:
            [n, x_root] = solve_polinom(1, -2*circle[0], circle[0]**2 + (b[1] - circle[1])**2 - r**2)
            y_root = b[1]
        except:
            n = solve_polinom(1, -2*circle[0], circle[0]**2 + (b[1] - circle[1])**2 - r**2)
    #если горизонтальная
    else:
        try:
            [n, y_root] = solve_polinom(1, -2*circle[1], circle[1]**2 + (b[0] - circle[0])**2 - r**2)
            x_root = b[0]
        except:
            n = solve_polinom(1, -2*circle[1], circle[1]**2 + (b[0] - circle[0])**2 - r**2)

    if n == 0:
        return False
    if n == 1:
        if y_ab != 0:
            x_root = (b[0]*y_ab - b[1]*x_ab + y_root*x_ab)/y_ab

        a_root = np.array([a[0]-x_root, a[1]-y_root])
        b_root = np.array([b[0]-x_root, b[1]-y_root])
        if np.inner(a_root, b_root) < 0:
            return True
        else:
            return False
    if n == 2:
        if y_ab != 0 and x_ab != 0:
            x_root = np.array([(b[0]*y_ab - b[1]*x_ab + y_root[0]*x_ab)/y_ab, (b[0]*y_ab - b[1]*x_ab + y_root[1]*x_ab)/y_ab])
        elif x_ab != 0:
            y_root = np.array([y_root, y_root])
        else:
            x_root = np.array([x_root, x_root])

        a_root1 = np.array([a[0]-x_root[0], a[1]-y_root[0]])
        b_root1 = np.array([b[0]-x_root[0], b[1]-y_root[0]])
        a_root2 = np.array([a[0]-x_root[1], a[1]-y_root[1]])
        b_root2 = np.array([b[0]-x_root[1], b[1]-y_root[1]])
        if np.inner(a_root1, b_root1)*np.inner(a_root2, b_root2) < 0:
            return True
        else:
            rho = (y_ab*circle[0] + x_ab*circle[1] + (b[0]*y_ab - b[1]*x_ab))/np.sqrt(x_ab**2 + y_ab**2)
            a_c = np.array([a[0]-circle[0], a[1]-circle[1]])
            b_c = np.array([b[0]-circle[0], b[1]-circle[1]])
            angle = np.arccos(np.inner(a_c, b_c)/np.linalg.norm(a_c)/np.linalg.norm(b_c))
            if rho < r and -np.pi/2 < angle < np.pi/2 or rho > r:
                return False
            else:
                return True

# def check_line_circle_collision(line: Line, circle: Circle) -> bool:
#     Q = circle.p                  # Centre of circle
#     r = circle.r                  # Radius of circle
#     P1 = line.points[0]      # Start of line segment
#     V = line.points[1] - P1  # Vector along line segment
#
#     a = V.dot(V)
#     b = 2 * V.dot(P1 - Q)
#     c = P1.dot(P1) + Q.dot(Q) - 2 * P1.dot(Q) - r**2
#     disc = b**2 - 4 * a * c
#     if disc < 0:
#         return True
#     else:
#         return False
    # dist = np.abs(line.A*circle.p[0] + line.B*circle.p[1] + line.C)/ np.sqrt(line.A**2 + line.B**2)
    # if circle.r >= dist:
    #     return True
    # else:
    #     return False

def check_line_circle_tuple_collision(line: Line, circle: Tuple[Circle]) -> bool:
    for c in circle:
        if check_line_circle_collision(line, c):
            return True
    return False

def check_chain_circle_tuple_collision(chian: Chain, circle: Tuple[Circle]) -> bool:
    for c in circle:
        if check_chain_circle_collision(chian, c):
            return True
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
