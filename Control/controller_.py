#!/usr/bin/env python
# coding: utf-8

from test import Boat
from roblib import * 


def draw_dock(xdock, theta):
    """_summary_

    Args:
        xdock (_type_): _description_
        theta (_type_): _description_
    """
    L, l = 1.2, 1
    P = array([[-L/3, L, L, 0], [0, 0, l, l]])
    P[0, :] = P[0, :] + xdock[0, 0] - L/2
    P[1, :] = P[1, :] + xdock[1, 0] - l / 2
    R = np.array([[cos(theta), -sin(theta)],
                  [sin(theta), cos(theta)]])
    P = np.dot(R,P)
    P = P.T
    draw_polygon(ax, P, None)
    
    
c11, c12 = 5, 1  # constantes pour les champs de potentiels
c21, c22 = 10, 5  # constantes pour les champs de potentiels


"""def f(x, u):
    x, u = x.flatten(), u.flatten()
    v, theta = x[2], x[3]
    return array([[v * cos(theta)], [v * sin(theta)], [u[0]], [u[1]]])"""
    

s = 30
theta_dock = pi/4
ax = init_figure(-s, s, -s, s)
e = array([0])
sum = 0
value = 3
start = True
#----------------------
x = array([[-3, -3, 0, 2]]).T*10
boat = Boat(x)
dt = .05

boat.init_kalman()
n = len(x)
noise = .01 * np.random.randn(3, 1)
y = np.array([x[0], x[1], x[-1]]) + noise
y1 = y

theta_receiv = 0
phat_receiv = 0
k = 0
kmax = 20
wave = kmax/100*randn()
for t in arange(0, 50, dt):
    clear(ax)
    # Ajoute un mouvement de rotation sinusoidal au dock
    if k < kmax:
        k += 1
    else:
        wave = kmax/200*randn()
        k = 0
    theta_dock += pi*wave/kmax
    phat = array([[0], [0]])
    draw_dock(phat, theta_dock)

    # Sans Kalman
    """noise = .08 * np.random.randn(5, 1)
    boat.x = x + noise
    boat.controller(phat, theta)
    x = x + dt*boat.f()
    draw_tank(x[[0, 1, 4]], 'red', 0.2)  # x,y,θ"""
    
    # Avec Kalman
    noise = .01 * np.random.randn(3, 1)
    if np.random.rand() < 1-.02*norm(x[:2]-phat):
        theta_receiv = theta_dock
        phat_receiv = phat
    if np.random.rand() < .9:
        y1 = np.array([x[0], x[1], x[-1]]) + noise
    psi = boat.x[-1, 0]
    theta = 0
    B = np.array([[cos(theta)*cos(psi), 0],
                  [cos(theta)*sin(psi), 0],
                  [-sin(theta)        , 0],
                  [0                  , 1]], dtype=np.float64)
    Q = 1*np.identity(n)
    C = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 0, 1]])
    R = .01*np.identity(len(y))
    # /!\ Controller avant le predict sinon effet bizarre sur simu; à voir en réalité
    boat.controller(phat, theta_dock, marge = .2)
    boat.kalman_predict(0, B, Q, dt)
    if not(np.array_equal(y1, y)):
        y = y1
        boat.kalman_correc(y, C, R, dt)
    x = x + dt*boat.f()
    draw_tank(x[[0, 1, -1]], 'red', 0.2)  # x,y,θ
    draw_ellipse_cov(ax, boat.x[:2], boat.Gx[:2, :2], 0.9, [1, 0.8, 0.8])
pause(1)
