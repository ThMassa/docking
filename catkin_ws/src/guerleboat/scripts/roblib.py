# MIT License

# Copyright (c) [2022] [Luc Jaulin]

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.




#available at https://www.ensta-bretagne.fr/jaulin/roblib.py
# For help : https://www.ensta-bretagne.fr/jaulin/python.html  
# used in KalMOOC :  https://www.ensta-bretagne.fr/kalmooc/
# used in RobMOOC :  https://www.ensta-bretagne.fr/robmooc/
# used in KalMOOC :  https://www.ensta-bretagne.fr/inmooc/


import numpy as np
import matplotlib.pyplot as plt
from numpy import mean,pi,cos,sin,sinc,sqrt,tan,arctan,arctan2,tanh,arcsin,arccos,\
                    exp,dot,array,log,inf, eye, zeros, ones, inf,size,\
                    arange,reshape,vstack,hstack,diag,median,\
                    sign,sum,meshgrid,cross,linspace,append,round,trace,rint
from matplotlib.pyplot import *
from matplotlib.cbook import flatten
from numpy.random import randn,rand,uniform
from numpy.linalg import inv, det, norm, eig,qr
from scipy.linalg import sqrtm,expm,logm,norm,block_diag

from scipy.signal import place_poles
from mpl_toolkits.mplot3d import Axes3D
from math import factorial
from matplotlib.patches import Ellipse,Rectangle,Circle, Wedge, Polygon, Arc
from matplotlib.collections import PatchCollection



# Unicode https://en.wikipedia.org/wiki/List_of_Unicode_characters
# αβδεthetaλΛμρτphipsiωΓ




def scalarprod(u,v): # scalar product
    u,v=u.flatten(),v.flatten()
    return sum(u[:]*v[:])


def eulermat(phi,theta,psi):
    return expw([0,0,psi]) @ expw([0,theta,0]) @ expw([phi,0,0])


def eulermat2angles(R):
    phi=arctan2(R[2,1],R[2,2])
    theta=-arcsin(R[2,0])
    psi=arctan2(R[1,0],R[0,0])
    return phi,theta,psi


def rot2w(R): return adjoint_inv(logm(R))

def eulerderivative(phi,theta,psi):
    cphi,sphi,ctheta,stheta,ttheta,cpsi,spsi = cos(phi),sin(phi),cos(theta),sin(theta),sin(theta)/cos(theta),cos(psi),sin(psi)        
    return array([[1,sphi*ttheta,cphi*ttheta],[0, cphi,-sphi],[0,sphi/ctheta,cphi/ctheta]])    
    
def angle(x):
    x=x.flatten()
    return arctan2(x[1],x[0])


def angle2d(u,v):
    u1,u2=u[0,0],u[1,0]
    v1,v2=v[0,0],v[1,0]
    dx=u1*v1+u2*v2
    dy=u1*v2-u2*v1
    return arctan2(dy,dx)



#def angle3dold(u,v):  #returns theta*w such that  v=expw(theta*w)*u  with theta minimal in [0,pi]
#    u=(1/norm(u))*array(u)
#    v=(1/norm(v))*array(v)
#    c=scalarprod(u,v)
#    w=adjoint(u)@v
#    if c > 0.99: return w
#    if c <-0.99:return angle3d(u,v+0.01*randn(3,1))
#    return (arccos(c)/norm(w))*w

def rotuv(u,v): #returns rotation with minimal angle  such that  v=R*u
            # see https://en.wikipedia.org/wiki/Rotation_matrix#Vector_to_vector_formulation
    u=array(u).reshape(3,1)
    v=array(v).reshape(3,1)
    u=(1/norm(u))*u
    v=(1/norm(v))*v
    c=scalarprod(u,v)
    A=v@u.T-u@v.T
    return eye(3,3)+A+(1/(1+c))*A@A

def angle3d(u,v):  #returns theta*w such that  v=expw(theta*w)*u  with theta minimal in [0,pi]
    u=array(u).reshape(3,1)
    v=array(v).reshape(3,1)
    if norm(u)<0.0001: return angle3d(u+0.01*randn(3,1),v)
    if norm(v)<0.0001: return angle3d(u,v+0.01*randn(3,1))
    u=(1/norm(u))*u
    v=(1/norm(v))*v
    if scalarprod(u,v) <-0.999: return angle3d(u+0.01*randn(3,1),v+0.01*randn(3,1))
    return logw(rotuv(u,v))

    
def add1(M):
    M=array(M)
    return vstack((M,ones(M.shape[1])))

def tolist(w):
    if isinstance(w,(list)): return w
    return list(flatten(w))

def adjoint(w):
    if isinstance(w, (float, int)): return array([[0,-w] , [w,0]])
    #print('w=',w)
    w=tolist(w)
    #print('tolist(w)=',w)
    return array([[0,-w[2],w[1]] , [w[2],0,-w[0]] , [-w[1],w[0],0]])

def ad(w):
    return adjoint(w)


def adjoint_inv(A):
    if size(A)==4:  return A[1,0]  # A is 2x2
    return array([[A[2,1]],[A[0,2]],[A[1,0]]]) # A is 3x3

def expw(w): return expm(adjoint(w))
def expwH(w): return ToH(expw(w))
def logw(R): return adjoint_inv(logm(R))

def Rlatlong(lx,ly): 
    return eulermat(0,0,lx)@eulermat(0,-pi/2+ly,-pi/2).T
        

def latlong2cart(ρ,lx,ly): 
    return ρ*array([[cos(ly)*cos(lx)],[cos(ly)*sin(lx)],[sin(ly)] ])  

def cart2latlong(x,y,z):
    r=norm(array([x,y,z]))
    ly=arcsin(z/r) 
    lx=arctan2(y,x)
    return (r,lx,ly)            

def tran2H(x,y):
    return array([[1,0,x],[0,1,y],[0,0,1]])

def rot2H(a):
    return array([[cos(a),-sin(a),0],[sin(a),cos(a),0],[0,0,1]])

def arrow2H(L):
    e=0.2
    return add1(L*array([[0,1,1-e,1,1-e],[0,0,-e,0,e]]))



def clean3D(ax, x1=-10, x2=10, y1=-10, y2=10, z1=-10, z2=10):
    ax.clear()
    ax.set_xlim3d(x1, x2)
    ax.set_ylim3d(y1, y2)
    ax.set_zlim3d(z1, z2)


def draw_arrow3D(ax, x, y, z, wx, wy, wz, col,mirror=1):  # initial point : x ; final point x+w
    ax.quiver(mirror*x, y, mirror*z, mirror*wx, wy, mirror*wz, color=col, lw=1, pivot='tail', length=norm([wx, wy, wz]))


def draw_axis3D(ax, x=0, y=0, z=0, R=eye(3), zoom=1,mirror=1):
    ax.scatter(mirror*x, y, mirror*z, color='magenta')
    R = zoom * R
    draw_arrow3D(ax, x, y, z, R[0, 0], R[1, 0], R[2, 0], "red",mirror)
    draw_arrow3D(ax, x, y, z, R[0, 1], R[1, 1], R[2, 1], "green",mirror)
    draw_arrow3D(ax, x, y, z, R[0, 2], R[1, 2], R[2, 2], "blue",mirror)



def ToH(R,v=array([[0],[0],[0]])):  # transformation matrix to homogenous
    H = hstack((R,v))
    V = vstack((H, array([0,0,0,1])))
    return V

def tran3H(x, y, z):
    return array([[1, 0, 0, x], [0, 1, 0, y], [0, 0, 1, z], [0, 0, 0, 1]])

"""
def rot3H(wx, wy, wz):
    return ToH(expm(adjoint(array([[wx], [wy], [wz]]))))
"""

def rot3H(wx, wy, wz): return ToH(expw([wx,wy,wz]))


def eulerH(phi,theta,psi):
    return ToH(expw([0,0,psi]) @ expw([0,theta,0]) @ expw([phi,0,0]))


def draw3H(ax, M, col, shadow=False, mirror=1):  # mirror=-1 in case z in directed downward
    ax.plot(mirror * M[0], M[1], mirror * M[2], color=col)
    if shadow: ax.plot(mirror * M[0], M[1], 0 * M[2], color='gray')



def cube3H():
    return array([[0,1,1,0,0,0,1,1,0,0,0,0,1,1,1,1],
               [0,0,0,0,0,1,1,1,1,1,1,0,0,1,1,0],
               [0,0,1,1,0,0,0,1,1,0,1,1,1,1,0,0],
               [1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1]])

def wheel3H(r):
    n = 12
    W0 = [[0, 0, 0], [r, 0, r], [0, 0, 0], [1, 1, 1]]
    W = W0
    R = rot3H(2 * pi / n, 0, 0)
    for i in range(n + 1):
        W0 = R @ W0
        W = hstack((W, W0))
    return W


def circle3H(r):
    n = 20
    theta = linspace(0, 2 * pi, n)
    x = r * cos(theta) + array(n * [0])
    y = r * sin(theta) + array(n * [0])
    z = zeros(n)
    return add1(array([x, y, z]))


def auv3H():
    return add1(array([ [0.0,0.0,10.0,0.0,0.0,10.0,0.0,0.0],
                   [-1.0,1.0,0.0,-1.0,-0.2,0.0,0.2,1.0],
                   [0.0,0.0,0.0,0.0,1.0,0.0,1.0,0.0]]))


def boat3H():
    b=1.5
    return add1(array([ [-2, 6, 6,-2,-2,-2,   9, 9, -2,-2,-2,-2,-2, 9,9,6, 6,9],
                        [1,  1,-1,-1, 1, b,   b,-b, -b, b, 1,-1,-b,-b,b,1,-1,-b],
                        [0,  0, 0, 0, 0, 1,   1, 1,  1, 1, 0, 0, 1, 1,1,0, 0,1 ]]))



def earth3H(ρ):
    a = pi/10
    Lx = arange(0, 2*pi+a, a)
    Ly = arange(-pi/2, pi/2+a, a)
    M1 = ρ*array([[cos(-pi/2)*cos(0)],[cos(-pi/2)*sin(0)],[sin(-pi/2)]])
    M2=M1
    for ly1 in Ly:
        for lx1 in Lx:
            M1 = hstack((M1, ρ*array([[cos(ly1)*cos(lx1)],[cos(ly1)*sin(lx1)],[sin(ly1)]])))
    for lx1 in Lx:
        for ly1 in Ly:
            M2 = hstack((M2, ρ*array([[cos(ly1)*cos(lx1)],[cos(ly1)*sin(lx1)],[sin(ly1)]])))
    M=hstack((M1, M2))
    return add1(M)


def cylinder3H(r,L):
    n = 25
    W=[[0.3,0],[0,0],[0,0]]
    for i in range(n+1):
        P1=[[0],[r*cos(2*pi*i/n)],[r*sin(2*pi*i/n)]]
        P2=[[L],[r*cos(2*pi*i/n)],[r*sin(2*pi*i/n)]]
        W=hstack((W,P1,P2,P1,[[0],[0],[0]]))
    return add1(array(W))


def draw_earth3D(ax,r,R,col='gray'):
    plot3D(ax,ToH(R)@earth3H(r),"gray")
    ax.scatter(*(R@array([[r],[0],[0]])),color='red')
    
def draw_wheel3D(ax,x,y,z,phi,theta,psi,r=1,col='blue',size=1):
    M=tran3H(x,y,z)@eulerH(phi,theta,psi)@wheel3H(r)
    draw3H(ax,M,col,True,1)
    p=array([[x],[y],[z]])+eulermat(phi,theta,psi)@array([[0],[1],[0]])
    ax.scatter(*p,color='red')

def draw_robot3D(ax,p,R,col='blue',size=1):
    M=tran3H(*p[0:3,0])@diag([size,size,size,1])@ ToH(R) @ auv3H()
    draw3H(ax, M, col, True, 1)
    pause(0.001)

def draw_auv3D(ax,x,y,z,phi,theta,psi,col='blue',size=1):
    draw_robot3D(ax,array([[x],[y],[z]]),eulermat(phi,theta,psi),col,size)
    
def draw_boat3D(ax,p,R,col='blue',size=1):
    p=array(p).reshape(3,1)
    M=tran3H(*p[0:3,0])@diag([size,size,size,1])@ ToH(R) @ boat3H()
    draw3H(ax, M, col, True, 1)

def axis3D(x1,x2,y1,y2,z1,z2):
    ax = Axes3D(figure())
    ax.set_xlim3d(x1,x2); ax.set_ylim3d(y1,y2); ax.set_zlim3d(z1,z2)
    return ax


def draw_quadrotor3D(ax,p,R,α,l,mirror=-1):
    Ca=hstack((circle3H(0.3*l),[[0.3*l,-0.3*l],[0,0],[0,0],[1,1]])) # the disc + the blades
    T = tran3H(p[0,0],p[1,0],p[2,0]) @ ToH(R)   #I replaced tran3H(*) to avoid warning
    C0= T @ tran3H(0,l,0)@eulerH(0,0,α[0,0])@Ca  # we rotate the blades
    C1= T @ tran3H(-l,0,0) @eulerH(0,0,-α[1,0])@Ca
    C2= T @ tran3H(0,-l,0) @eulerH(0,0,α[2,0])@Ca
    C3= T @ tran3H(l,0,0) @eulerH(0,0,-α[3,0])@Ca
    M = T @ add1(array([[l,-l,0,0, 0],[0,0,0,l,-l],[0,0,0,0,0]]))
    draw3H(ax,M,'grey',True,mirror)  #body
    draw3H(ax,C0,'green',True,mirror)
    draw3H(ax, C1, 'black', True,mirror)
    draw3H(ax, C2, 'red', True,mirror)
    draw3H(ax, C3, 'blue', True, mirror)



def draw_riptide3D(ax,pos,R,u,α):
    u=u.flatten()
    R=ToH(R)
    T=tran3H(*pos[0:3,0])
    flap  = add1(array([[-1,  0, 0, -1, -1],[ 0,  0, 0,  0,  0],[0, 0, 1, 1, 0]]))
    flap1= tran3H(0,0,1)@rot3H(0,0,u[1])@flap
    flap2 = rot3H(2*pi/3, 0, 0) @tran3H(0, 0, 1) @ rot3H(0, 0, u[2]) @ flap
    flap3 = rot3H(-2*pi/3, 0, 0) @tran3H(0, 0, 1) @ rot3H(0, 0, u[3]) @ flap
    Ca=hstack((circle3H(1.5), [[1.5, -1.5], [0, 0], [0, 0], [1, 1]])) # the disc + the blades
    C0=tran3H(-1,0,0)@rot3H(0,1.57,0)@eulerH(0,0,α)@Ca  # we rotate the blades
    draw3H(ax,T@R@C0,'green')
    draw3H(ax,T@R@flap1,'red')
    draw3H(ax,T@R@flap2,'magenta')
    draw3H(ax,T@R@flap3,'magenta')
    M=T@R @ cylinder3H(1,10)
    ax.plot(M[0],M[1],1*M[2],color='orange')
    ax.plot(M[0],M[1],0*M[2],color='grey')
    pause(0.001)

def draw_riptide(ax,x,u,α):  #obsolete
    R=eulermat(*x[3:6,0])
    pos=array([[x[0,0]],[x[1,0]],[x[2,0]]])
    draw_riptide3D(ax, pos, R, u, α)


def plot2D(M,col='black',w=1):
    plot(M[0, :], M[1, :], col, linewidth = w)    

def plotScalarFunction(f,tmin,tmax,dt=0.01,col='red',w=1):
    M=np.empty((2,0))
    for t in arange(tmin,tmax,dt):
        v=array([[t],[f(t)]])
        M=hstack((M,v))
    plot2D(M,col,w)
     
    
def plot3D(ax,M,col='black',w=1):
    ax.plot(M[0, :], M[1, :],M[2, :], col, linewidth = w)         

def draw_segment(a,b,col='darkblue',w=1):
    plot2D(hstack((a,b)),col, w)
    #plot2D(a,'ro')
    #plot2D(b,'ro')      

def draw_box_border(a1,a2,b1,b2,col='darkblue',w=1):
    M1=array([[a1],[b1]])
    M2=array([[a1],[b2]])
    M3=array([[a2],[b2]])
    M4=array([[a2],[b1]])
    draw_segment(M1,M2,col,w)
    draw_segment(M2,M3,col,w)
    draw_segment(M3,M4,col,w)
    draw_segment(M4,M1,col,w)

def draw_point(A,col='darkblue',w=0.1):
    draw_box_border(A[0,0]-w,A[0,0]+w,A[1,0]-w,A[1,0]+w,col,10*w)



def draw_ellipse0(ax, c, Γ, a, col,coledge='black'):  # classical ellipse (x-c)T * invΓ * (x-c) <a^2
    # draw_ellipse0(ax,array([[1],[2]]),eye(2),a,[0.5,0.6,0.7])
    A = a * sqrtm(Γ)
    w, v = eig(A)
    v1 = array([[v[0, 0]], [v[1, 0]]])
    v2 = array([[v[0, 1]], [v[1, 1]]])
    f1 = A @ v1
    f2 = A @ v2
    phi = (arctan2(v1[1, 0], v1[0, 0]))
    α = phi * 180 / 3.14
    e = Ellipse(xy=c, width=2 * norm(f1), height=2 * norm(f2), angle=α)
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)

    e.set_alpha(0.7)
    e.set_facecolor(col)
    e.set_edgecolor(coledge)

    # e.set_fill(False)
    # e.set_alpha(1)
    # e.set_edgecolor(col)




def draw_ellipse_cov(ax,c,Γ,η,col,coledge='black'): # Gaussian confidence ellipse with artist
    #draw_ellipse_cov(ax,array([[1],[2]]),eye(2),0.9,[0.5,0.6,0.7])
    if (norm(Γ)==0):
        Γ=Γ+0.001*eye(len(Γ[1,:]))
    a=sqrt(-2*log(1-η))
    draw_ellipse0(ax, c, Γ, a, col,coledge)

    

def draw_disk(ax,c,r,col,alph=0.7,w=1):
    #draw_disk(ax,array([[1],[2]]),0.5,"blue")
    e = Ellipse(xy=c, width=2*r, height=2*r, angle=0,linewidth = w)   
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(alph)  # transparency
    e.set_facecolor(col)
    
    

def draw_box(ax,x1,x2,y1,y2,col):
    # print("x1,x2,y1,y2,col",x1,x2,y1,y2,col)
    c=array([[x1],[y1]])    
    rect = Rectangle(c, width=x2-x1, height=y2-y1, angle=0)
    rect.set_facecolor(array([0.4,0.3,0.6]))   
    ax.add_patch(rect)
    rect.set_clip_box(ax.bbox)
    rect.set_alpha(0.7)
    rect.set_facecolor(col)    


def draw_polygon(ax, P, col):
    patches = []
    patches.append(Polygon(P, True))
    p = PatchCollection(patches, cmap=matplotlib.cm.jet, alpha=0.4, color=col)
    ax.add_collection(p)


def draw_arc(c,a,theta,col):
    s = arange(0,abs(theta),0.01)
    s = sign(theta) * s
    d = a-c
    r = norm(d)
    alpha = angle(d)
    w = c@ones((1,size(s))) + r*array([[cos(alpha), -sin(alpha)],[sin(alpha), cos(alpha)]])@array([cos(s),sin(s)])
    plot2D(w,col,3)  
    
def draw_pie(ax,c,ρ1,ρ2,theta1,theta2,col):
    n = 12
    W0 = array([[ρ1*np.cos(theta1)], [ρ1*np.sin(theta1)]])
    W = W0
    dtheta=(theta2-theta1)/n
    R = array([[np.cos(dtheta),-np.sin(dtheta)],[np.sin(dtheta),np.cos(dtheta)]])
    for i in range(n + 1):
        W0 = R @ W0
        W = hstack((W, c+W0))
    W0 = [[ρ2 * np.cos(theta2)], [ρ2 * np.sin(theta2)]]
    R = array([[np.cos(dtheta), np.sin(dtheta)], [-np.sin(dtheta), np.cos(dtheta)]])
    for i in range(n + 1):
        W0 = R @ W0
        W = hstack((W, c+W0))
    draw_polygon(W.T, ax, col)    
    
    
def draw_arrow(x,y,theta,L,col='darkblue',w=1):
    plot2D(tran2H(x,y)@rot2H(theta)@arrow2H(L),col,w)



def draw_sailboat(x,δs,δr,psi,awind):
    mx,my,theta,v,w=list(x[0:5,0])
    hull=add1(array([[-1,5,7,7,5,-1,-1,-1],[-2,-2,-1,1,2,2,-2,-2]]))
    sail=array([[-7,0],[0,0],[1,1]])
    rudder=array([[-1,1],[0,0],[1,1]])
    R=tran2H(mx,my)@rot2H(theta)
    Rs=tran2H(3,0)@rot2H(δs)
    Rr=tran2H(-1,0)@rot2H(δr)
    draw_arrow(mx+5,my,psi,5*awind,'red')
    plot2D(R@hull,'black');
    plot2D(R@Rs@sail,'red',2);
    plot2D(R@Rr@rudder,'red',2);

def draw_tank(x,col='darkblue',r=1,w=2):
    mx,my,theta=tolist(x)[0:3]
    M = r*array([[1,-1,0,0,-1,-1,0,0,-1,1,0,0,3,3,0], [-2,-2,-2,-1,-1,1,1,2,2,2,2,1,0.5,-0.5,-1]])
    M=add1(M)
    plot2D(tran2H(mx,my)@rot2H(theta)@M,col,w)

def draw_tank_trailer(x1,x2,x3,x4,x5):
    r=0.07
    M = add1(r*np.array([[1,-1,0,0,-1,-1,0,0,-1,1,0,0,4,4,0], [-3,-3,-3,-2,-2,2,2,3,3,3,3,2,1,-1,-2]]))
    R1=tran2H(x1,x2)@rot2H(x3)
    MT = add1(r*np.array([[1,-1,0,0,1,-1,0,0,1/r], [-3,-3,-3,3,3,3,3,0,0]]))
    plot2D(R1@M,'blue',2)
    plot2D(R1@rot2H(x4-x3)@tran2H(-1,0)@MT,'magenta',2)
    pause(0.001)

    
def draw_invpend(ax,x): #inverted pendulum
    s,theta=x[0,0],x[1,0]
    draw_box(ax,s-0.7,s+0.7,-0.25,0,'blue')
    plot( [s,s-sin(theta)],[0,cos(theta)],'magenta', linewidth = 2)
    

    	
    
    
def draw_car(x,col='darkblue',L=1,w=2): # the car has a length L
    mx,my,theta,v,δ=list(x[0:5,0])
    M = add1(L*array([[-0.3, 1.3, 1.6,1.6,1.3,-0.3,-0.3,-0.3, 0, 0,-0.3, 0.3,0,0,-0.3,0.3, 0, 0,1,1,1],  
                      [-0.7,-0.7,-0.3,0.3,0.7, 0.7,-0.7,-0.7,-0.7,-1,-1,-1,-1,1, 1,1, 1, 0.7,0.7,1,-1]]))                
    R=tran2H(mx,my)@rot2H(theta)
    W = add1(L*array([[-0.3, 0.3], [0, 0]])) #Front Wheel                
    plot2D(R@M,col,w)          
    plot2D(R@tran2H(L,L)@rot2H(δ)@W,col,1)
    plot2D(R@tran2H(L,-L)@rot2H(δ)@W,col,1)
    

def tondarray(M):
    if type(M)==float:
        return array([[M]])
    elif type(M)==int:
        return array([[M]])        
    else:
        return M    


def mvnrnd(xbar,Γ,n): 
    X=randn(2,n)
    X = (xbar @ ones((1,n))) + sqrtm(Γ) @ X
    return(X)    



def mvnrnd2(x,G): 
    n=len(x)
    x1=x.reshape(n)
    y = np.random.multivariate_normal(x1,G).reshape(n,1)
    return(y)    

def mvnrnd1(G):
    G=tondarray(G)
    n=len(G)
    x=array([[0]] * n)
    return(mvnrnd2(x,G))  
    

def kalman_predict(xup,Gup,u,Γα,A):
    Γ1 = A @ Gup @ A.T + Γα
    x1 = A @ xup + u    
    return(x1,Γ1)

def kalman_correc(x0,Γ0,y,Γβ,C):
    S = C @ Γ0 @ C.T + Γβ        
    K = Γ0 @ C.T @ inv(S)           
    ytilde = y - C @ x0        
    Gup = (eye(len(x0))-K @ C) @ Γ0 
    xup = x0 + K@ytilde
    return(xup,Gup) 
    
def kalman(x0,Γ0,u,y,Γα,Γβ,A,C):
    xup,Gup = kalman_correc(x0,Γ0,y,Γβ,C)
    x1,Γ1=kalman_predict(xup,Gup,u,Γα,A)
    return(x1,Γ1)     


def place(A,B,poles):
    return place_poles(A,B,poles).gain_matrix
  
def demo_draw():  
    ax=init_figure(-15,15,-15,15)
    
    c=array([[5],[0]])
    e = Ellipse(xy=c, width=13.0, height=2.0, angle=45)  
    ax.add_artist(e)
    e.set_clip_box(ax.bbox)
    e.set_alpha(0.9)
    e.set_facecolor(array([0.7,0.3,0.6]))   
    
    rect = Rectangle( (1,1), width=5, height=3)
    rect.set_facecolor(array([0.4,0.3,0.6]))   
    ax.add_patch(rect)    
        
    pause(0.2)    
    draw_tank(array([[-7],[5],[1],[3]]))
    draw_tank(array([[-7],[5],[1]]),'red',0.2)

    
    draw_car(array([[1],[2],[3],[4],[0.5]]),'blue',3)   
    
    c = array([[-2],[-3]])
    G = array([[2,-1],[-1,4]])
    draw_ellipse_cov(ax,c,G,0.9,[0.8,0.8,1])
    P=array([[5,-3],[9,-10],[7,-4],[7,-6]])
    draw_polygon(P,ax,'green')   
    draw_disk(ax,array([[-8],[-8]]),2,"blue")
    draw_arc(array([[0],[5]]),array([[4],[6]]),2,'red')   
    show()  # only at the end. Otherwize, it closes the figure in a terminal mode

def loadcsv(file1):
    fichier = open(file1,'r')
    D = fichier.read().split("\n")
    fichier.close()
    for i in range(len(D)):
        D[i] = D[i].split(";")
    D = array([[float(elt) for elt in Ligne] for Ligne in D])
    return D


def init_figure(xmin,xmax,ymin,ymax): 
    fig = figure()
    ax = fig.add_subplot(111, aspect='equal')	
    ax.xmin=xmin
    ax.xmax=xmax
    ax.ymin=ymin
    ax.ymax=ymax
    clear(ax)
    return ax

def clear(ax):
    pause(0.001)
    cla()
    ax.set_xlim(ax.xmin,ax.xmax)
    ax.set_ylim(ax.ymin,ax.ymax)

def draw_field(ax,f,xmin,xmax,ymin,ymax,a):
    Mx    = arange(xmin,xmax,a)
    My    = arange(ymin,ymax,a)
    X1,X2 = meshgrid(Mx,My)
    VX,VY=f(X1,X2) 
    R=sqrt(VX**2+VY**2)
    quiver(Mx,My,VX/R,VY/R)  
    
    

def demo_animation():    
    ax=init_figure(-15,15,-15,15)
    for t in arange(0,5,0.1) :
        clear(ax)
        draw_car(array([[t],[2],[3+t],[4],[5+t]]),'blue',3)    
        c = array([[-2+2*t],[-3]])
        G = array([[2+t,-1],[-1,4+t]])
        draw_ellipse_cov(ax,c,G,0.9,[0.8,0.8,1])
#       if (k%10==0): savefig('name'+str(k)+'.png')
    show()


def demo_random():  
    N=1000
    xbar = array([[1],[2]])
    Γx = array([[3,1],[1,3]])
    X=randn(2,N)
    Y=rand(2,3)
    print("Y=",Y)
    X = (xbar @ ones((1,N))) + sqrtm(Γx) @ X
    xbar_ = mean(X,axis=1)
    Xtilde = X - xbar @ ones((1,N))
    Γx_ = (Xtilde @ Xtilde.T)/N
    ax=init_figure(-20,20,-20,20)
    draw_ellipse_cov(ax,xbar,Γx,0.9,[1,0.8,0.8])
    pause(0.5)    
    ax.scatter(*X)    
    pause(1)
    plot()  

def demo_field():
    def f(x1,x2): return -(x1**3+x2**2*x1-x1+x2),-(x2**3+x1**2*x2-x1-x2)
    xmin,xmax,ymin,ymax=-2.5,2.5,-2.5,2.5 
    ax=init_figure(xmin,xmax,ymin,ymax)
    draw_field(ax,f,xmin,xmax,ymin,ymax,0.3)
    pause(1)


def sawtooth(x):
    return (x+pi)%(2*pi)-pi   # or equivalently   2*arctan(tan(x/2))

def projSO3(M):   # return a rotation matrix close to M
    Q,R = np.linalg.qr(M)
    return Q@diag(diag(sign(R)))


if __name__ == "__main__":
    demo_draw()

#    A=array([[2,1],[-1,2]])
#    print(logm(A))

#    print('main program to test the functions of roblib.py')
#    u=array([[1],[0],[0]])
#    v=array([[1.1],[0],[0]])
#    w=angle3d(u, v)
#    print('w=',w)

#    R=eulermat(1,2,3)
#    D=diag((1,2,3))
#    A=R@D@R.T
    #ax = Axes3D(figure())
    #clean3D(ax, -10, 10, -10, 10,-10, 10)
    #draw_axis3D(ax, 0, 0, 0, 3 * eye(3, 3))
    #draw_robot3D(ax, array([[2],[3],[4]]), eye(3, 3), 'blue', 0.3)
    #pause(1)

        #phi, theta, psi=-0.1,-0.2,-0.3
    #R=eulermat(phi, theta, psi)
    #print(R)
    #phi1, theta1, psi1=eulermat2angles(R)
    #print(phi1, theta1, psi1)


    #     phi,theta,psi=list(x[3:6])
    #    s,theta,ds,dtheta =list(x[0:4,0])  #select the components of a vector
    # p = x[[0,1,3]] # forms the subvector associated to comppnents 1,2,4

#    print("v=",v)
#    print("R1=",R1)
#    w=R1[:,2] # rotation vector
#    print('R@w=',R@w) #  R*w % we check that it an eigen vector associated to 1 
#
#    α=arccos(0.5*(trace(R)-1))
#    print ("angle associated to w:",α)
#    M = expm(α*adjoint(w))
#    print(M)
#    print(R)
         

    
    
    
#    np.set_printoptions(threshold=np.nan)  # print vectors in the console without "..."
#    R=zeros((3,4))
#    x=[[1],[2],[3]]
#    print('R1=',R1)
#          
#    demo_animation()
#    demo_random()

#    demo_field()
    
#
#    M=array([[1,2],[5,6],[9,10]])
#    print(M)
#    x=array([[1], [2]])    
#    x2= M@x  #multiplication dans Python 3
#    
##
#    G = array([[1, 0], [0, 1]])
#    x3=mvnrnd2(x,G)
#    print("x3=",x3)
#    
#    x4=mvnrnd1(G)
#    print(x4)
#    
#    
#    
