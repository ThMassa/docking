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

c11, c12 = 10, 20
c21, c22 = 10, 5
x = array([[-3, -3, 1, 2]]).T  # x,y,v,θ
dt = 0.05
s = 10
ax = init_figure(-s, s, -s, s)
vhat = array([[0], [0]])
value = 0
u = np.array([[0], [0]])
e = np.array([0])
vmax = 5
start = True
sum = 0
k_ = 1
theta = pi/4
for t in arange(0, 50, dt):
    clear(ax)
    # theta += pi/50*randn()
    unit = array([[cos(theta)], [sin(theta)]])
    phat = array([[0], [0]])
    draw_dock(phat, theta)

    # qhat = array([[2 * cos(t / 5)], [2 * sin(t / 5)]])
    qhat = array([[-10], [-10]])
    draw_disk(ax, qhat, 0.3, "magenta")
    n = np.array([[cos(theta + pi / 2)], [sin(theta + pi / 2)]])

    if unit.T@(x[:2] - phat) < value and start:
        print('State 1')
        vbar = c21 * (x[:2]-phat)/norm(x[:2]-phat)**3 + c22 * unit 
        draw_field(ax, f2, -s, s, -s, s, 0.8)
        if value == 0:
            value = 6
    else:
        print('State 2')
        if start:
            sum = 0
            start = False
        k_ = -sign(unit.T@(phat-x[:2]))
        print("----")
        vbar = -c11*n@n.T@(x[:2]-phat) + c12*np.array([[cos(theta+pi)], [sin(theta+pi)]])
        draw_field(ax, f1, -s, s, -s, s, 0.8)
        if unit.T@(x[:2] - phat) < -value:
            start = True
            k_ = 1
            
    thetabar = np.arctan2(vbar[1, 0], vbar[0, 0])
    # vbar = norm(vbar)
    vbar = min(norm(vbar), k_*norm(phat - x[:2]))
    vbar = max(min(vmax, vbar), -vmax)
    print(vbar)
    if len(e) < 5:
        e = np.append(e, vbar - x[2, 0])
    else:
        e[:-1] = e[1:]
        e[-1] = vbar - x[2, 0]
    # u[0] = e[-1]
    """if norm(phat - x[:2]) > .1:
        u[0] = e[-1] + 18*(e[-1] - e[-2])
    else:
        u[0] = 0"""
    print(norm(phat - x[:2]))
    if norm(phat - x[:2]) < .1:
        x[:2] -= 2*unit
        print('----------------------------------------------------------------------------------------------')
    sum += e[-1]
    u[0] = 2 * e[-1] + 18 * (e[-1] - e[-2]) + .01*sum
    u[1] = 10*sawtooth(thetabar - x[3, 0])
    x = x + dt * f(x, u)
    draw_tank(x[[0, 1, 3]], 'red', 0.2)  # x,y,θ


pause(1)
