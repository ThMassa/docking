# coding: latin-1

from numpy import cos, sin, array, sign, pi
import numpy as np
from numpy.linalg import norm


"""Si rover pour Kalman A = 0 et B = [[cos(theta), 0],
                                      [sin(theta), 0],
                                      [0         , 1]]
                        C = [[1, 0, 0, 0],
                             [0, 1, 0, 0],
                             [0, 0, 0, 1]]
   Si bateau A = 0
             B = [[cos(theta)*cos(psi), 0],
                  [cos(theta)*sin(psi), 0],
                  [-sin(theta)        , 0]]
             C = [[1, 0, 0, 0, 0],
                  [0, 1, 0, 0, 0],
                  [0, 0, 0, 0, 1]]
                                      """


def sawtooth(x):
    return (x+pi)%(2*pi)-pi


class Boat:
    def __init__(self, x, u=np.array([[0], [0]]), L = 1,vmax=1):
        """_summary_

        Args:
            x (colunm numpy array): State vector (px, py, v, heading)
            u (colunm numpy array): Control vector (v, yaw)
            L (int, optional): Length of the boat. Defaults to 1.
            vmax (int, optional): Maximum speed of the boat. Defaults to 1.
        """
        self.x = x
        self.vmax = vmax
        self.L = L
        self.u = u
        
    
    def init_kalman(self, Gx=None):
        if Gx == None:
            self.Gx = 100*np.identity(len(self.x))
        else:
            self.Gx = Gx
            
    
    def kalman_predict(self, A, B, Q, dt):
        A = np.identity(len(self.x)) + dt*A
        B = dt*B
        Q = dt*Q
        self.Gx = np.dot(A, self.Gx)
        self.Gx = np.dot(self.Gx, A.T) + Q
        self.x = np.dot(A, self.x) + np.dot(B, self.u)
        self.__predict = True
    
    
    def kalman_correc(self, y, C, R, dt):
        R = 1/dt*R
        S = np.dot(C, self.Gx)
        S = np.dot(S, C.T) + R
        K = np.dot(self.Gx, C.T)
        K = np.dot(K, np.linalg.inv(S))
        ytilde = y - np.dot(C, self.x)
        self.Gx = np.dot(np.eye(len(self.x))-np.dot(K, C), self.Gx) 
        self.x = self.x + np.dot(K, ytilde)
        self.__predict = False
    
    
    def dead_reckoning(self):
        self.x += np.array([[],
                            [],
                            [],
                            []])
        
    
    
    def kalman(self, y, A, B, C, Q, R):
        if self.__predict == False:
            self.kalman_predict(y, A, B, Q)
        else:
            self.kalman_correc(y, C, R)
            self.kalman_predict(y, A, B, Q)
    
    
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
        if norm(phat - self.x[:2]) < .2*self.L:
            self.__start = False
        
        ecap = sawtooth(thetabar - self.x[3, 0])
        self.__scap += ecap
        if len(self.__ecap) < 5:
            self.__ecap = np.append(self.__ecap, ecap)
        else:
            self.__ecap[:-1] = self.__ecap[1:]
            self.__ecap[-1] = ecap
        u[0,0] = vbar
        u[1,0] = 5*ecap + 0*(self.__ecap[-1] - self.__ecap[-2]) + .0*self.__scap
        # u[1,0] = 5*sawtooth(thetabar - self.x[3, 0])
        return u

if __name__=="__main__":
    boat = Boat(np.array([[0], [0], [2], [1]]))
    u = boat.controller(np.array([[2], [2]]), pi/4)
    print(u)