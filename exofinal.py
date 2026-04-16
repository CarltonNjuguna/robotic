import numpy as np
import modern_robotics as mr

def invMatT(T):
    R = T[:3, :3].T  #inv mat rot
    t = -(R@T[:3, 3:])  # translation produit mat rot inv
    # important bien prendre l'inverse car on reviens en arriere 
    RT = np.hstack((R, t))
    return(np.vstack((RT,np.array([[0,0,0,1]]))))

def Pvm(p,T): #prend un point et calcul matrice homogene 
    p = np.append(p,1) # on rajoute 1 si c'est un point
    # 0 si c'est un vecteur c'est les coordones homogene
    return(T@p)

def skew_symmetric(v): #Crée la matrice antisymétrique associée au vecteur v
    return np.array([
        [0, -v[2], v[1]],
        [v[2], 0, -v[0]],
        [-v[1], v[0], 0]
    ])

def FourToSix(T):
    R = T[:3, :3]
    t = T[:3, 3]
    rt = skew_symmetric(t)@R
    return(np.block(([[R,np.zeros((3,3))],[rt,R]]))) #matrice 6x6 

def torsion(V,T): #torsion dans une base dans une autre
    #premiere etape 4X4 ---< 6X6
    return(FourToSix(T)@V) #matrice adjointe x le vecteur pour le changement de base 

def radians(m): #formule de diag R = 1 + 2 cos(theta)
    x = (m[0][0] + m[1][1] + m[2][2] - 1)/2
    return(np.arccos(x))

Tsa = np.array([
    [0, -1,  0,  0],
    [0,  0, -1,  0],
    [1,  0,  0,  1],
    [0,  0,  0,  1]
])

Tsb = np.array([
    [1,  0,  0,  0],
    [0,  0,  1,  2],
    [0, -1,  0,  0],
    [0,  0,  0,  1]
])

Ts = np.array([
    [1,  0,  0,  0],
    [0,  1,  0,  0],
    [0,  0,  1,  0],
    [0,  0,  0,  1]
])

T11 = np.array([
    [0,  -1, 0, 3],
    [1,  0,  0, 0],
    [0,  0,  1, 1],
    [0,  0,  0, 1]
])

M14 = np.array([
    [0, -1.5708, 0, 2.3562],
    [1.5708, 0, 0, -2.3562],
    [0, 0, 0, 1],
    [0, 0, 0, 0]
])

M15 = np.array([
    [0, -1, 0, 3],
    [1,  0, 0, 0],
    [0,  0, 1, 1],
    [0,  0, 0, 1]])

Tab = invMatT(Tsa)@Tsb
pb = np.array([1,2,3])
Vs = np.array([3,2,1,-1,-2,-3])
fb = np.array([1,0,0,2,1,0])
#print(Pvm(pb,Tsb))
#print(torsion(Vs,invMatT(Tsa)))
#print(radians(Tsa))
#print(mr.MatrixExp6(mr.VecTose3(np.array([0,1,2,3,0,0]))))
#print(FourToSix(Tsb)@fb)
#print(mr.TransInv(T11))
#print(mr.VecTose3(np.array([1,0,0,0,2,3])))
#print(mr.ScrewToAxis(np.array([1,0,0]),np.array([0,0,2]),2))
#print(mr.MatrixExp6(M14))
print(mr.MatrixLog6(M15))
