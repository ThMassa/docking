#!/usr/bin/env python
# coding: utf-8

from test import Boat
from roblib import * 

c11, c12 = 5, 1  # constantes pour les champs de potentiels
c21, c22 = 10, 5  # constantes pour les champs de potentiels


def draw_dock(xdock, theta):
    L, l = 1.2, 1
    P = array([[-L/3, L, L, 0], [0, 0, l, l]])
    P[0, :] = P[0, :] + xdock[0, 0] - L/2
    P[1, :] = P[1, :] + xdock[1, 0] - l / 2
    R = np.array([[cos(theta), -sin(theta)],
                  [sin(theta), cos(theta)]])
    P = np.dot(R,P)
    P = P.T
    draw_polygon(ax, P, None)


"""def f(x, u):
    x, u = x.flatten(), u.flatten()
    v, theta = x[2], x[3]
    return array([[v * cos(theta)], [v * sin(theta)], [u[0]], [u[1]]])"""
    

s = 10
theta = pi/4
ax = init_figure(-s, s, -s, s)
e = array([0])
sum = 0
value = 3
start = True
x = array([[-3, -3, 0, 1, 2]]).T
boat = Boat(x)
dt = .05

boat.init_kalman()
n = len(x)
for t in arange(0, 50, dt):
    clear(ax)
    theta += pi/50*randn()
    phat = array([[0], [0]])
    draw_dock(phat, theta)

    # Sans Kalman
    """noise = .08 * np.random.randn(5, 1)
    boat.x = x + noise
    boat.controller(phat, theta)
    x = x + dt*boat.f()
    draw_tank(x[[0, 1, 4]], 'red', 0.2)  # x,y,θ"""
    
    # Avec Kalman
    noise = .01 * np.random.randn(3, 1)
    y = np.array([x[0], x[1], x[-1]]) + noise
    psi = boat.x[4, 0]
    theta = 0
    B = np.array([[cos(theta)*cos(psi), 0],
                      [cos(theta)*sin(psi), 0],
                      [-sin(theta)        , 0],
                      [1                  , 0],
                      [0                  , 1]], dtype=np.float64)
    Q = 1*np.identity(n)
    C = np.array([[1, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 1]])
    R = .01*np.identity(len(y))
    boat.kalman_predict(0, B, Q, dt)
    boat.kalman_correc(y, C, R, dt)
    boat.controller(phat, theta)
    x = x + dt*boat.f()
    draw_tank(x[[0, 1, 4]], 'red', 0.2)  # x,y,θ
    draw_ellipse_cov(ax, boat.x[:2], boat.Gx[:2, :2], 0.9, [1, 0.8, 0.8])
pause(1)
