import numpy as np
import matplotlib.pyplot as plt
from roblib import *

x = np.arange(-10,10)
y = x**2

fig = plt.figure()
ax = fig.add_subplot(111)
custom_x_limits = (0, 12)  # Example x-axis limits
custom_y_limits = (0, 12)  # Example y-axis limits
ax.set_xlim(custom_x_limits)
ax.set_ylim(custom_y_limits)
ax.set_aspect('equal')
# ax.plot(x,y)

coords = []

start = None
end = None
ax.cla()

def obstacleRepulsif(obstacles, X,Y):
    """
    Fonction qui retourne les valeurs du champs de gradiant associé à un potentiel répulsif crée par des obstacles
    param obstacles : liste d'array numpy correspondant aux coordonnées des obstacles [(x1,y1),(x2,y2),...,(xn,yn)]
    param p : point (x,y) de calcul du champ
    return : les tableaux VX/R et VY/R (normés du coup) correspondants aux valeurs du champ de gradiant
    """
    # def field(x1, x2):
    #     """
    #     Fonction de tracé du champ de gradiant associé au champ de potentiel de la situation
    #     param x1 : coordonnées x du robot
    #     param x2 : coordonnées y du robot
    #     return : la transformation du plan (x,y) associée au gradient
    #     """
    #     kobst = 25 #pour modifier la répulsivité des obstacles #TODO : à modifier si besoin pour régler mieux

    #     x, y = 0, 0

    #     for o in obstacles:
    #         #pour chaque obstacle, on ajoute le champ correspondant, cf. cours
    #         o1, o2 = o[0, 0], o[1, 0]
    #         n = np.sqrt((x1 - o1) ** 2 + (x2 - o2) ** 2)
    #         x += -(x1 - o1) / (n ** 3) * kobst

    #         y += -(x2 - o2) / (n ** 3) * kobst

    def field(x1, x2):
        kobst = 40
        x, y = np.zeros(X.shape), np.zeros(X.shape)
        for obstacle in obstacles:
            n = np.sqrt((x1-obstacle[0])**2+(x2-obstacle[1])**2)
            x += -(x1-obstacle[0])/(n**3)*kobst
            y += -(x2-obstacle[1])/(n**3)*kobst
        return -x, -y
    return field(X, Y)

def pointAttractif(obstacles, X,Y):

    def field(x1, x2):
        kobst = 40
        x, y = np.zeros(X.shape), np.zeros(X.shape)
        for obstacle in obstacles:
            n = np.sqrt((x1-obstacle[0])**2+(x2-obstacle[1])**2)
            x += -(x1-obstacle[0])/(n**2)*kobst
            y += -(x2-obstacle[1])/(n**2)*kobst
        return x, y
    return field(X, Y)

def calculate_field(obstacles, attractifs, X, Y):
    repulse_x, repulse_y = obstacleRepulsif(obstacles, X, Y)
    attract_x, attract_y = pointAttractif(attractifs, X, Y)
    return repulse_x+attract_x, repulse_y+attract_y

obstacles = []
attractifs = []
first_click = False

def onclick(event):
    global ix, iy, start, end, first_click
    ix, iy = event.xdata, event.ydata
    print (f'x = {ix}, y = {iy}')

    if event.button == 1:
        # if not first_click:
        #     first_click = True
        #     return
        start = [ix, iy]
        obstacles.append([ix, iy])
        ax.cla()
        draw_field(ax, lambda X,Y: calculate_field(obstacles,attractifs,X, Y), custom_x_limits[0], custom_x_limits[1], custom_y_limits[0], custom_y_limits[1], 0.3)
    elif event.button == 3:
        end = [ix, iy]
        attractifs.append(end)
        ax.cla()
        draw_field(ax, lambda X,Y: calculate_field(obstacles,attractifs,X, Y), custom_x_limits[0], custom_x_limits[1], custom_y_limits[0], custom_y_limits[1], 0.3)


    # if start is not None and end is not None:
    #     ax.cla()
    #     ax.set_xlim(custom_x_limits)
    #     ax.set_ylim(custom_y_limits)
        # ax.plot([start[0], end[0]], [start[1], end[1]])
        


    # ax.scatter(ix,iy)
    plt.pause(0.2)

    global coords
        # fig.canvas.mpl_disconnect(cid)
    return coords
cid = fig.canvas.mpl_connect('button_press_event', onclick)

# plt.pause(0.2)
ax.scatter([0], [0])
ax.cla()
plt.show()