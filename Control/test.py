from numpy import cos, sin, array, sign, pi
import numpy as np
from numpy.linalg import norm


def sawtooth(x):
    return (x+pi)%(2*pi)-pi


class Boat:
    def __init__(self, x, vmax=1):
        self.x = x
        self.vmax = vmax
        
    def controller(self, phat, theta, marge=1.5):
        """Controleur utilisant les champs de potentiels

        Args:
            phat (_type_): Position a atteindre
            theta (_type_): Cap souhaite
            marge (_type): marge de securite, plus elle est elevee, plus le bateau s'arretera loin du dock et donc moins il aura de chance de se cogner contre le dock
        """
        
        if not hasattr(self, "_Boat__start"):
            self.__start = True
            self.__value = 0
            self.__ecap = array([0])
            self.__scap = 0
            
        c11, c12 = 5, 1  # constantes pour les champs de potentiels
        c21, c22 = 10, 5  # constantes pour les champs de potentiels
        u = np.array([[0], [0]])
        k_ = 1
        unit = np.array([[cos(theta)], [sin(theta)]])
        n = np.array([[cos(theta + pi / 2)], [sin(theta + pi / 2)]])
        phat0 = phat + marge*unit
        if np.dot(unit.T, self.x[:2] - phat) < self.__value and self.__start:
            vbar = c21 * (self.x[:2]-phat)/norm(self.x[:2]-phat)**3 + c22 * unit
            if self.__value == 0:
                self.__value = 3
                self.__scap = 0
        else:
            if self.__value == 3:
                self.__scap = 0
                self.__value = 0
            k_ = -sign(np.dot(unit.T, phat-self.x[:2]))[0, 0]
            nn = np.dot(n, n.T)
            vbar = -c11*np.dot(nn, self.x[:2]-phat) + c12*np.array([[cos(theta+pi)], [sin(theta+pi)]])

        thetabar = np.arctan2(vbar[1, 0], vbar[0, 0])
        
        vbar = min(norm(vbar), k_*self.vmax*norm(phat0 - self.x[:2]))
        if norm(phat - self.x[:2]) < .1:
            self.__start = False
        
        ecap = sawtooth(thetabar - self.x[3, 0])
        self.__scap += ecap
        if len(self.__ecap) < 5:
            self.__ecap = np.append(self.__ecap, ecap)
        else:
            self.__ecap[:-1] = self.__ecap[1:]
            self.__ecap[-1] = ecap
        u[0,0] = vbar - self.x[2, 0]
        u[1,0] = 5*ecap + 0*(self.__ecap[-1] - self.__ecap[-2]) + .05*self.__scap
        # u[1,0] = 5*sawtooth(thetabar - self.x[3, 0])
        return u
        
if __name__=="__main__":
    boat = Boat(np.array([[0], [0], [2], [1]]))
    u = boat.controller(np.array([[2], [2]]), pi/4)
    print(u)