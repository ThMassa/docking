# coding: latin-1

from numpy import cos, sin, array, sign, pi
import numpy as np
from numpy.linalg import norm
from EKF import EKF

#TODO l'IMU du bateau n'est pas bien orienté


def sawtooth(x):
    return (x+pi)%(2*pi)-pi


class Rover:
    """Classe bateau permettant de contrôler un bateau mais également un véhicule à deux roues.
    
    Pour un bon fonctionnement il faut :
    
    - Une faible accélération de sorte que la vitesse est égale à la vitesse désirée à chaque instant car le contrôle se commande en vitesse et en lacet
    
    - Un faible gîte de sorte que la dérivée de la position du bateau de dépend pas du gîte
    """
    def __init__(self, x, u=np.array([[0.], [0.]]), L = 1,vmax=1, dthetamax = 5.):
        """Initialise l'instance

        Args:
            x (numpy.ndarray): Vecteur d'état (px, py, pz, heading)
            u (numpy.ndarray): Commande (v, yaw)
            L (int, optional): Longueur du bateau. Defaults to 1.
            vmax (int, optional): Vitesse maximale du bateau. Defaults to 1.
        """
        self.x = x
        self.vmax = vmax
        self.L = L
        self.u = u
        self.dthetamax = dthetamax
        
        self.zk = np.array([[0,0.]]).T
        
    
    def init_kalman(self, Gx=None):
        """Initialise la matrice de covariance liée au vecteur d'état du bateau

        Args:
            Gx (numpy.ndarray, optional): Matrice de covariance liée au vecteur d'état. Defaults to None.
        """
        if Gx is None:
            self.Gx = 100*np.identity(len(self.x))
        else:
            self.Gx = Gx
            
    
    def kalman_predict(self, A, B, Q, dt):
        """Prédit la position du bateau avec le filtre de Kalman. L'équation d'évolution considérée est dx/dt = Ax + Bu + w

        Args:
            A (numpy.ndarray): Matrice d'évolution
            B (numpy.ndarray): Matrice de commande
            Q (numpy.ndarray): Matrice de covariance du bruit d'évolution w
            dt (float): Période d'une itération dans la boucle principale 
        """
        A = np.identity(len(self.x)) + dt*A
        B = dt*B
        Q = dt*Q
        self.Gx = np.dot(A, self.Gx)
        self.Gx = np.dot(self.Gx, A.T) + Q
        self.x = np.dot(A, self.x) + np.dot(B, self.u)
        self.__predict = True
    
    
    def kalman_correc(self, y, C, R, dt):
        """Corrige la position du bateau avec le filtre de Kalman. Le processus d'observation considéré est y = Cx + v

        Args:
            y (numpy.ndarray): Vecteur des observations
            C (numpy.ndarray): Matrice d'observation
            R (numpy.ndarray): Matrice de covariance du bruit de mesure v
            dt (float): Période d'une itération dans la boucle principale 
        """
        R = 1/dt*R
        S = np.dot(C, self.Gx)
        S = np.dot(S, C.T) + R
        K = np.dot(self.Gx, C.T)
        K = np.dot(K, np.linalg.inv(S))
        ytilde = y - np.dot(C, self.x)
        self.Gx = np.dot(np.eye(len(self.x))-np.dot(K, C), self.Gx) 
        self.x = self.x + np.dot(K, ytilde)
        self.__predict = False
    
    
    def f(self, theta=0):
        """Fonction d'évolution du bateau. (Obselète)

        Args:
            theta (int, optional): Assiete du bateau. Defaults to 0.

        Returns:
            np.ndarray: dx/dy
        """
        psi = self.x[-1, 0]
        B = np.array([[cos(theta)*cos(psi), 0],
                      [cos(theta)*sin(psi), 0],
                      [-sin(theta)        , 0],
                      [0                  , 1]], dtype=np.float64)
        return np.dot(B, self.u)
        

    def fc(self,X,u):
        x,y,th= X.flatten()
        u1,u2 = u.flatten()
        return np.array([[u1*cos(th),u1*sin(th),u2]]).T
    
    
    def dead_reckoning(self, theta, dt):
        """Dead reckoning (utilisez plutôt la fonction kalman_predict si possible)

        Args:
            theta (float): Assiete du bateau
            dt (float): Période d'une itération dans la boucle principale 
        """
        self.x = self.x + dt*self.f(theta)
        
    
    def kalman(self, y, A, B, C, Q, R, dt):
        """Utilise kalman_correc si nécessaire et kalman_predict. Pour plus d'information voir ces deux fonctions

        Args:
            y (np.ndarray): Vecteur des observations
            A (np.ndarray): Matrice d'évolution
            B (np.ndarray): Matrice de commande
            C (np.ndarray): Matrice d'observation
            Q (np.ndarray): Matrice de covariance du bruit d'évolution
            R (np.ndarray): Matrice de covariance du bruit de mesure
            dt (float): Période d'une itération dans la boucle principale 
        """
        if not self.__predict:
            self.kalman_predict(y, A, B, Q,dt)
        else:
            self.kalman_correc(y, C, R, 1)
            self.kalman_predict(y, A, B, Q, dt)
    
    def extended_kalman(self,u,y,yk_1,dt):
        X = np.array([[self.x[0,0],self.x[1,0],self.x[-1,0]]]).T
        Gx = np.zeros((3,3))
        Gx[0,0] = self.Gx[0,0]
        Gx[1,1] = self.Gx[1,1]
        Gx[2,2] = self.Gx[-1,-1]
        # print(X)

        X, Gx = EKF(X,Gx,y,u,self.fc,dt,yk_1)

        print(X)
        print("##############")
        # print(self.x)
        self.x[:2] = X[:2]
        self.x[-1,0] = X[2,0]

        self.Gx[0,0] = Gx[0,0]
        self.Gx[1,1] = Gx[1,1]
        self.Gx[-1,-1] = Gx[2,2]


    
    def controller(self, phat, theta, marge=.5):
        """Controleur du bateau utilisant les champs de potentiels

        Args:
            phat (np.ndarray): Position à atteindre
            theta (float): Cap souhaité
            marge (float): Marge de securité, plus elle est elevée, plus le bateau s'arretera loin du dock et donc moins il aura de chance de se cogner contre le dock
        """
        
        if not hasattr(self, "_Boat__start"):
            self.__start = True
            self.__value = 0
            self.__ecap = array([0.])
            self.__scap = 0
            
        c11, c12 = 5, 2  # constantes pour les champs de potentiels
        c21, c22 = 10, 5  # constantes pour les champs de potentiels
        k_ = 1
        unit = np.array([[cos(theta)], [sin(theta)]])
        n = np.array([[cos(theta + pi / 2)], [sin(theta + pi / 2)]])
        phat0 = phat + marge*unit
        if np.dot(unit.T, self.x[:2] - phat)[0,0] < self.__value and self.__start:
            vbar = c21 * (self.x[:2]-phat)/norm(self.x[:2]-phat)**3 + c22 * unit
            if self.__value == 0:
                self.__value = 3*self.L
                self.__scap = 0
        else:
            if self.__value == 3*self.L:
                self.__scap = 0
                self.__value = 0
            k_ = -sign(np.dot(unit.T, phat-self.x[:2]))[0, 0]
            nn = np.dot(n, n.T)
            vbar = -c11*np.dot(nn, self.x[:2]-phat) + c12*np.array([[cos(theta+pi)], [sin(theta+pi)]])

        thetabar = np.arctan2(vbar[1, 0], vbar[0, 0])
        
        vbar = min(norm(vbar), k_*norm(phat0 - self.x[:2]))
        vbar = min(self.vmax, vbar)
        if norm(phat - self.x[:2]) < .2*self.L:
            # self.__scap = 0
            self.__start = False
        
        ecap = sawtooth(thetabar - self.x[-1])
        self.__scap += ecap
        if len(self.__ecap) < 5:
            self.__ecap = np.append(self.__ecap, ecap)
        else:
            self.__ecap[:-1] = self.__ecap[1:]
            self.__ecap[-1] = ecap
        self.u[0,0] = vbar
        self.u[1,0] = (5*ecap + .0*self.__scap)/20
        self.u[1,0] = min(self.dthetamax, self.u[1,0])
        self.u[1,0] = max(-self.dthetamax, self.u[1,0])
        # u[1,0] = 5*sawtooth(thetabar - self.x[4, 0])
        # print(self.u[1, 0])
        # return self.u


if __name__=="__main__":
    rover = Rover(np.array([[0], [0], [2], [1]]))
    u = rover.controller(np.array([[2], [2]]), pi/4)
    print(u)