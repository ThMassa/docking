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


def f(x, u):
    x, u = x.flatten(), u.flatten()
    v, theta = x[2], x[3]
    return array([[v * cos(theta)], [v * sin(theta)], [u[0]], [u[1]]])
    

s = 10
theta = pi/4
ax = init_figure(-s, s, -s, s)
e = array([0])
sum = 0
value = 3
start = True
x = array([[-3, -3, 1, 2]]).T
boat = Boat(x)
dt = .05
for t in arange(0, 50, dt):
    clear(ax)
    theta += pi/50*randn()
    phat = array([[0], [0]])
    draw_dock(phat, theta)

    # u, e, sum, value, start = controller(x, phat, theta, e, sum, value, start)
    noise = .08 * np.random.randn(4, 1)
    boat.x = x + noise
    u = boat.controller(phat, theta)
    x = x + dt*f(x, u)
    draw_tank(x[[0, 1, 3]], 'red', 0.2)  # x,y,θ
pause(1)
