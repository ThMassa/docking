# coding: latin-1


import numpy as np

def kalman_predict(xup,Gup,u,Ga,A):
    G1 = np.dot(A, np.dot(Gup , A.T)) + Ga
    x1 = np.dot(A , xup) + u    
    return(x1,G1)

def kalman_correc(x0,G0,y,Gb,C):
    S = np.dot(C , np.dot(G0 , C.T)) + Gb        
    K = np.dot(G0 , np.dot(C.T , np.linalg.inv(S)))           
    ytilde = y - np.dot(C , x0)        
    Gup = np.dot(np.eye(len(x0))-np.dot(K , C), G0) 
    xup = x0 + np.dot(K,ytilde)
    return(xup,Gup) 
    
def kalman(x0,G0,u,y,Ga,Gb,A,C):
    xup,Gup = kalman_correc(x0,G0,y,Gb,C)
    x1,G1=kalman_predict(xup,Gup,u,Ga,A)
    return(x1,G1)   

sigma_x,sigma_y,sigma_psi, = 1,1,0.1
sigma_beta = 1

dt = 1/5

Galpha = dt*np.diag([sigma_x,sigma_y,sigma_psi])
Gbeta = (sigma_beta/dt)*np.eye(2)


# def g(x):
#     p = x[:2]
#     return x_intruder - p + np.random.normal(scale=sigma_beta,size=(2,1))
# C = -array([[1,0,0],
#            [0,1,0]])

def g(x):
    x = x.flatten()
    p = np.array([[x[0],x[1],x[-1]]]).T
    return p
C = np.array([[1,0,0],
              [0,1,0],
              [0,0,1]])


def v(x,u,fc,dt):
    f = x + dt*fc(x,u)
    return np.dot(f-A(x,u,dt),x)

def z(y,x_hat):
    return y - g(x_hat) + np.dot(C,x_hat)

def A(X,u,dt):
    x1,x2,x3= X.flatten()
    u1,u2 = u.flatten()
    dfc = np.array([[0,0,-u1*np.sin(x3)],
                    [0,0, u1*np.cos(x3)],
                    [0,0, 0]])
    return np.eye(3)+dt*dfc


def EKF(X_hat,Gx,y,u,fc,dt,yk_1):
    vk = v(X_hat,u,fc,dt)
    zk = z(y,X_hat)
    Ak = A(X_hat,u,dt)
    Ck = C
    
    if np.linalg.norm(y-yk_1)>1E-2:
        xup, Gup = kalman_correc(X_hat,Gx,zk,Gbeta,Ck)
        X_hat, Gx = kalman_predict(xup,Gup,vk,Galpha,Ak)
    
    else:
        X_hat, Gx = kalman_predict(X_hat,Gx,vk,Galpha,Ak)
    return X_hat,Gx