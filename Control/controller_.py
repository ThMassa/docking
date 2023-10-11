from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


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


def f(x, u):
    x, u = x.flatten(), u.flatten()
    v, θ = x[2], x[3]
    return array([[v * cos(θ)], [v * sin(θ)], [u[0]], [u[1]]])


def f1(x1, x2):
    prod = n[0, 0]*(x1 - phat[0, 0]) + n[1, 0]*(x2 - phat[1, 0])
    x10 = -c11 * n[0, 0] * prod + c12 * cos(theta + pi)
    x20 = -c11 * n[1, 0] * prod + c12 * sin(theta + pi)
    return x10, x20


def f2(x1, x2):
    x10 = c21*(x1 - phat[0, 0]) / ((x1 - phat[0, 0]) ** 2 + (x2 - phat[1, 0]) ** 2) ** (3 / 2) + c22 * cos(theta)
    x20 = c21*(x2 - phat[1, 0]) / ((x1 - phat[0, 0]) ** 2 + (x2 - phat[1, 0]) ** 2) ** (3 / 2) + c22 * sin(theta)
    return x10, x20


L, l = 1, 1  # taille Longueur largeur du dock
c11, c12 = 5, 1  # constantes pour les champs de potentiels
c21, c22 = 10, 5  # constantes pour les champs de potentiels

def controller(x, phat, theta, e, sum=0, value=0, start=True):
    """
    x : vecteur d'état du robot x = (px, py, v, theta)
    phat : vecteur d'état du dock phat = (px, py)
    theta : cap du dock
    e : liste des erreurs de vitesse, e[i] = vbar - v
    value : 
    """
    
    dt = 0.05  # à déterminer
    vmax = 5
    vhat = array([[0], [0]])
    
    phat = array([phat[0], phat[1]])
    u = np.array([[0], [0]])
    k_ = 1
    
    unit = array([[cos(theta)], [sin(theta)]])
    n = np.array([[cos(theta + pi / 2)], [sin(theta + pi / 2)]])
    phat0 = phat + 1.5*unit  # marge de sécurité pour que le bateau ne rentre pas dans le dock
    if unit.T@(x[:2] - phat) < value and start:
        vbar = c21 * (x[:2]-phat)/norm(x[:2]-phat)**3 + c22 * unit 
        if value == 0:
            value = 3
            sum = 0
    else:
        if value == 3:
            sum = 0
            value = 0
        k_ = -sign(unit.T@(phat-x[:2]))
        vbar = -c11*n@n.T@(x[:2]-phat) + c12*np.array([[cos(theta+pi)], [sin(theta+pi)]])

    thetabar = np.arctan2(vbar[1, 0], vbar[0, 0])
    vbar = min(norm(vbar), k_*norm(phat0 - x[:2]))
    print(vbar)
    if len(e) < 5:
        e = np.append(e, vbar - x[2, 0])
    else:
        e[:-1] = e[1:]
        e[-1] = vbar - x[2, 0]
    if norm(phat - x[:2]) < .1:
        start = False
    sum += e[-1]
    u[0] = 2 * e[-1] + 18 * (e[-1] - e[-2]) + .01*sum
    u[1] = 10*sawtooth(thetabar - x[3, 0])
    return u, e, sum, value , start
    

if __name__=="__main__":
    s = 10
    theta = pi/4
    ax = init_figure(-s, s, -s, s)
    e = array([0])
    sum = 0
    value = 3
    start = True
    x = array([[-3, -3, 1, 2]]).T
    dt = .05
    for t in arange(0, 50, dt):
        clear(ax)
        theta += pi/50*randn()
        phat = array([[0], [0]])
        draw_dock(phat, theta)

        u, e, sum, value, start = controller(x, phat, theta, e, sum, value, start)
        x = x + dt * f(x, u)
        draw_tank(x[[0, 1, 3]], 'red', 0.2)  # x,y,θ
    pause(1)
