from roblib import *  # available at https://www.ensta-bretagne.fr/jaulin/roblib.py


L, l = 1, 1  # taille Longueur largeur du dock
c11, c12 = 5, 1  # constantes pour les champs de potentiels
c21, c22 = 10, 5  # constantes pour les champs de potentiels

def controller(x, phat, theta, value=0, start=True):
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
    phat0 = phat + 1.5*unit
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
    
    if norm(phat - x[:2]) < .1:
        start = False
    u[0] = vbar
    u[1] = thetabar
    return u, value, start
