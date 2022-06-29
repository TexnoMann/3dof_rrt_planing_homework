import numpy as np
from numpy.linalg import norm, det, inv

#theta - массив углов, l - массив длин сочленений
def get_point(theta, l):

  points = np.zeros((3,2))

  points[0, 0] = l[0]*np.cos(theta[0])
  points[0, 1] = l[0]*np.sin(theta[0])
  points[1, 0] = points[0, 0] + l[1]*np.cos(theta[0] + theta[1])
  points[1, 1] = points[0, 1] + l[1]*np.sin(theta[0] + theta[1])
  points[2, 0] = l[2]*np.cos(theta[0] + theta[1] + theta[2]) + points[1, 0]
  points[2, 1] = l[2]*np.sin(theta[0] + theta[1] + theta[2]) + points[1, 1]

  return points

def trans_Jacobian_matrix(theta, l):

  f = np.zeros((2,3))

  f[0, 0] = -l[0]*np.sin(theta[0]) - l[1]*np.sin(theta[0] + theta[1]) - l[2]*np.sin(theta[0] + theta[1] + theta[2])
  f[1, 0] = l[2]*np.cos(theta[0] + theta[1] + theta[2]) + l[1]*np.cos(theta[0] + theta[1]) + l[0]*np.cos(theta[0])
  f[0, 1] = -l[0]*np.sin(theta[0]) - l[1]*np.sin(theta[0] + theta[1])
  f[1, 1] = l[1]*np.cos(theta[0] + theta[1]) + l[0]*np.cos(theta[0])
  f[0, 2] = -l[0]*np.sin(theta[0])
  f[1, 2] = l[0]*np.cos(theta[0])

  return f.transpose()

def get_angles(point, l, theta0: np.array = np.array([0,0,0])):

  k = 0
  alpha = 0.2
  point_init = get_point(theta0, l)[2]
  e = point - point_init
  theta = theta0
  while norm(e) > 10**(-5):
    theta = theta0 + alpha*trans_Jacobian_matrix(theta0, l) @ e
    theta0 = theta
    point_init = get_point(theta0, l)[2]
    e = point - point_init
    k += 1
    # print(k, e)
    if k>=1e6:
      return None

  return theta

def get_angles2(point, l, theta0: np.array = np.array([0, 0, 0])):
    init_theta = theta0
    for i in range(0, 50):
        theta0 = np.random.rand(theta0.shape[0])*np.pi-np.pi/2
        k = 0
        alpha = 0.2
        point_init = get_point(theta0, l)[2]
        e = point - point_init
        theta = theta0
        state = False
        while norm(e) > 10**(-3):
            theta = theta0 + alpha*trans_Jacobian_matrix(theta0, l) @ e
            theta0 = theta
            point_init = get_point(theta0, l)[2]
            e = point - point_init
            k += 1
      # print(k, e)
            if k>=5e3:
                break
        if norm(e) < 10**(-3):
            state = True
            return theta, state
    return init_theta, state
