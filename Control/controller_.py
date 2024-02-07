#!/usr/bin/env python
# coding: utf-8
import numpy as np

from classBoat import Boat
from roblib import *
from time import time


def draw_dock(xdock, theta):
    """_summary_

    Args:
        xdock (_type_): _description_
        theta (_type_): _description_
    """
    L, l = 1.2, 1
    P = array([[-L / 3, L, L, 0], [0, 0, l, l]])
    P[0, :] = P[0, :] + xdock[0, 0] - L / 2
    P[1, :] = P[1, :] + xdock[1, 0] - l / 2
    R = np.array([[cos(theta), -sin(theta)],
                  [sin(theta), cos(theta)]])
    P = np.dot(R, P)
    P = P.T
    draw_polygon(ax, P, None)


def f(x, u, theta=0):
    """
    Fonction d'evolution du bateau. Pour la comparaison du kalman avec le deadreckoning
    Args:
        x: vecteur d'etat
        u: commande
        theta (int, optional): Assiete du bateau. Defaults to 0.

    Returns:
        np.ndarray: dx/dy
    """
    psi = x[-1, 0]
    B = np.array([[cos(theta) * cos(psi), 0],
                  [cos(theta) * sin(psi), 0],
                  [-sin(theta), 0],
                  [0, 1]], dtype=np.float64)
    return np.dot(B, u)


c11, c12 = 5, 1  # constantes pour les champs de potentiels
c21, c22 = 10, 5  # constantes pour les champs de potentiels

"""def f(x, u):
    x, u = x.flatten(), u.flatten()
    v, theta = x[2], x[3]
    return array([[v * cos(theta)], [v * sin(theta)], [u[0]], [u[1]]])"""

s = 30
theta_dock = pi / 4
ax = init_figure(-s, s, -s, s)
e = array([0])
sum = 0
value = 3
start = True
# ----------------------

x = np.random.rand(4, 1) * 50 - 25
xdr = x.copy()
boat = Boat(x, vmax=5)
dt = .02

boat.init_kalman()
n = len(x)
noise = .01 * np.random.randn(3, 1)
y = np.array([x[0], x[1], x[-1]]) + noise
y1 = y

theta_receiv = 0
phat_receiv = 0
k = 0
kmax = 20
wave = kmax / 100 * randn()
t0GPS = 0
t0 = time()
dt_command = .999*dt*2
t0_command = 0
logs_xdr = np.array([[0], [0], [0], [0]])
logs_x = np.array([[0], [0], [0], [0]])
logs_ekf_x = np.array([[0], [0], [0], [0]])
logs_dock = np.array([[0], [0]])
for t in arange(0, 50, dt):
    # print(t)
    clear(ax)
    ax.xmin = -s
    ax.xmax = s
    ax.ymin = -s
    ax.ymax = s
    # Ajoute un mouvement de rotation sinusoidal au dock
    
    if k < kmax:
        k += 1
    else:
        wave = kmax / 200 * randn()
        k = 0
    theta_dock += pi * wave / kmax
    phat = array([[0], [0]])
    draw_dock(phat, theta_dock)
    unit = np.array([[cos(theta_dock)], [sin(theta_dock)]])
    # Sans Kalman
    """noise = .08 * np.random.randn(5, 1)
    boat.x = x + noise
    boat.controller(phat, theta)
    x = x + dt*boat.f()
    draw_tank(x[[0, 1, 4]], 'red', 0.2)  # x,y,θ"""

    # Avec Kalman
    
    d = norm(x[:2] - phat)
    if s > 10:
        s = 1 * d + boat.L
    if np.random.rand() < 1 - .02 * d:
        theta_receiv = theta_dock
        phat_receiv = phat
    if np.random.rand() < .9 and t - t0GPS >= 1:
        noise = .33 * np.random.randn(3, 1)
        noise[2, 0] = 0.07 * np.random.randn()
        y1 = np.array([x[0], x[1], x[-1]]) + noise
        t0GPS = t
    psi = boat.x[-1, 0]
    theta = 0
    B = np.array([[cos(theta) * cos(psi), 0],
                  [cos(theta) * sin(psi), 0],
                  [-sin(theta), 0],
                  [0, 1]], dtype=np.float64)
    C = np.array([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 0, 1]])
    # /!\ Controller avant le predict sinon effet bizarre sur simu; à voir en réalité
    # print('theta:', theta_receiv)
    
    if t - t0_command >= dt_command:
        Q = np.square(.05) * np.identity(n)
        Q[2, 2] = np.square(.1)
        boat.controller(phat_receiv, theta_receiv, marge=.2)
        boat.kalman_predict(0, B, Q, dt_command)
        t0_command = t
        xdr = xdr + dt_command * f(xdr, boat.u)

        if not (np.array_equal(y1, y)):
            R = np.square(.33) * np.identity(len(y))
            R[2, 2] = np.square(.07)
            y = y1
            boat.kalman_correc(y, C, R, dt_command)
            xdr[:2] = y[:2]
            xdr[-1] = y[-1]
    
    
    # boat.dead_reckoning(0, dt)
    logs_x = np.append(logs_x, x, axis=1)
    logs_ekf_x = np.append(logs_ekf_x, boat.x, axis=1)
    logs_xdr = np.append(logs_xdr, xdr, axis=1)
    logs_dock = np.append(logs_dock, phat, axis=1)

    if norm(phat - x[:2]) < .4 * boat.L:
        break
    x = x + dt * boat.f()
    
    draw_tank(x[[0, 1, -1]], 'red', 0.2)  # x,y,θ
    draw_ellipse_cov(ax, boat.x[:2], boat.Gx[:2, :2], 0.9, [1, 0.8, 0.8])
    if time() - t0 < dt:
        pause(dt - time() + t0)
pause(1)
np.save('logs_x.npy', logs_x)
np.save('logs_ekf_x.npy', logs_ekf_x)
np.save('logs_xdr.npy', logs_xdr)
np.save('logs_dock.npy', logs_dock)
