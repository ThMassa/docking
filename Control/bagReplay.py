import rosbag
import os
import numpy as np
from roblib import *


"""Code permettant de rejouer les rosbags. La seule chose à modifier est la variable 'freq'. Elle définit la rapidité
à la quelle les logs sont rejoués (on affiche 1 log sur n (=freq)). """


freq = 5
def draw_dock(xdock, theta):
    L, l = 1.2, 1
    P = array([[-L/3, L, L, 0], [0, 0, l, l]])
    P[0, :] = P[0, :] + xdock[0, 0] - L/2
    P[1, :] = P[1, :] + xdock[1, 0] - l / 2
    R = np.array([[cos(theta), -sin(theta)],
                  [sin(theta), cos(theta)]])
    P = R@P
    P = P.T
    draw_polygon(ax, P, None)


def msgToArray(msg):
    result = np.zeros((6, 1))
    result[0] = msg.pose.position.x
    result[1] = msg.pose.position.y
    result[2] = msg.pose.position.z
    result[3] = msg.pose.orientation.x
    result[4] = msg.pose.orientation.y
    result[5] = msg.pose.orientation.z
    return result


def bag_to_array(file_path):
    # Ouvrir le fichier .bag en mode lecture
    bag = rosbag.Bag(file_path)
    rover_pose = np.zeros((6, 1))
    rover_kalman = np.zeros((6, 1))
    dock_pose = np.zeros((6, 1))
    idx = np.array([['rover_pose', 0]])
    t0 = None
    for topic, msg, t in bag.read_messages():
        # Afficher le nom du topic et le timestamp du message
        if t0 is None:
            t0 = t.to_sec()
        if 14.792 > t.to_sec() - t0 > 14.702 and topic == "/rover_pose":
            print(topic)
            print(msg.pose.position)
            print(t.to_sec()-t0)
        if topic == "/rover_pose" and not np.array_equal(msgToArray(msg), rover_pose[:, [-1]]):
            rover_pose = np.append(rover_pose, msgToArray(msg), axis=1)
            idx = np.append(idx, [['rover_pose', t.to_sec()-t0]], axis=0)
        elif topic == "/dock_pose":
            dock_pose = np.append(dock_pose, msgToArray(msg), axis=1)
            idx = np.append(idx, [['dock_pose', t.to_sec()-t0]], axis=0)
        elif topic == "/rover_kalman" and not np.array_equal(msgToArray(msg), rover_kalman[:, [-1]]):
            rover_kalman = np.append(rover_kalman, msgToArray(msg), axis=1)
            idx = np.append(idx, [['rover_kalman', t.to_sec()-t0]], axis=0)
    # Fermer le fichier .bag
    bag.close()
    assert np.array_equal(np.sort(idx[:, 1].astype(float)), idx[:, 1].astype(float))
    # rover_pose = np.unique(rover_pose, axis=1)

    return rover_pose[:, 1:], rover_kalman[:, 1:], dock_pose[:, 1:], idx


def f1(x1, x2):
    prod = n[0, 0]*(x1 - phat[0, 0]) + n[1, 0]*(x2 - phat[1, 0])
    x10 = -c11 * n[0, 0] * prod + c12 * cos(theta + pi)
    x20 = -c11 * n[1, 0] * prod + c12 * sin(theta + pi)
    return x10, x20


def f2(x1, x2):
    x10 = c21*(x1 - phat[0, 0]) / ((x1 - phat[0, 0]) ** 2 + (x2 - phat[1, 0]) ** 2) ** (3 / 2) + c22 * cos(theta)
    x20 = c21*(x2 - phat[1, 0]) / ((x1 - phat[0, 0]) ** 2 + (x2 - phat[1, 0]) ** 2) ** (3 / 2) + c22 * sin(theta)
    return x10, x20


c11, c12 = 5, 2
c21, c22 = 10, 5
file_path = "./../logs_rosbag/echec1.bag"
rover_pose, rover_kalman, dock_pose, idx = bag_to_array(file_path)
x_raw = np.zeros((6, 1))
x = np.zeros((6, 1))  # x avec kalman du rover
phat = np.zeros((2, 1))  # position du dock
theta = 0  # angle du dock
idx_raw = 0
idx_kalman = 0
idx_dock = 0

s = 20
ax = init_figure(-s, s, -s, s)
idxstart = np.argwhere(idx=='rover_kalman')
# idx = idx[idxstart[0, 0]:]
phat0 = dock_pose[:2, [1]]
t0 = 0
L = 1
value = 0
start = True
draw1, draw2 = True, True
count = 0
assert np.array_equal(np.sort(idx[:, 1].astype(float)), idx[:, 1].astype(float))

for id in idx:
    if id[0] == 'rover_pose':
        x_raw = rover_pose[:, [idx_raw]]
        x_raw[:2] -= phat0
        idx_raw += 1
    elif id[0] == 'rover_kalman':
        x = rover_kalman[:, [idx_kalman]]
        x[:2] -= phat0
        idx_kalman += 1
    elif id[0] == 'dock_pose':
        phat = dock_pose[:2, [idx_dock]] - phat0
        theta = dock_pose[-1, idx_dock]
        idx_dock += 1
    unit = array([[cos(theta)], [sin(theta)]])
    n = array([[cos(theta + pi / 2)], [sin(theta + pi / 2)]])
    if count%freq == 0:
        plt.cla()
    if unit.T@(x[:2] - phat) < value and start:
        # Avec retour
        if count%freq==0:
            draw_field(ax, f2, -s, s, -s, s, 0.5)
        if value == 0:
            value = 3*L
    else:
        # Directe
        if start:
            start = False
        if np.dot(unit.T, x[:2] - phat) < -value / 3:
            start = True
        if count%freq==0:
            draw_field(ax, f1, -s, s, -s, s, 0.5)
    if count%freq == 0:
        draw_dock(phat, theta)
        # draw_tank(x[[0, 1, 5]], 'red', 0.2)
        # draw_tank(x_raw[[0, 1, 5]], 'black', 0.2)
        plt.scatter(x_raw[0], x_raw[1], color='blue')
        plt.scatter(x[0], x[1], color='red')
        plt.plot(rover_pose[0, 1:]-phat0[0, 0], rover_pose[1, 1:]-phat0[1, 0], color='blue', label='x')
        plt.plot(rover_kalman[0, 1:]-phat0[0, 0], rover_kalman[1, 1:]-phat0[1, 0], color='red', label='ekf_x')
        plt.scatter(phat[0], phat[1], label='dock')


        ax.set(xlim=(-s, s), ylim=(-s, s))
        plt.pause((id[1].astype(float) - t0)/8+.001)  # plt.pause(0) met en pause definitivement d'où le + 0.001
        t0 = id[1].astype(float)
    count += 1
plt.pause()
