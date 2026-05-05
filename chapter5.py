import numpy as np
import modern_robotics as mr

r = np.sqrt(2)/2
n = np.pi / 2

J1 = np.array([[1,1,1],[0,0,r],[0,-1,-1-r]])
F1 = np.array([0,2,0])

L1 = L2 = L3 = L4 = 1

Jb = np.array([
    [1, 1, 1, 1],
    [-L3 , -L3, -L3, 0],
    [L4 + L2+L1, L4+L2, L4, L4]
])
Fb = np.array([10,10,10])

Slist = np.array([[0, 1, 0],
                  [0, 0, 0],
                  [1, 0, 0],
                  [0, 0, 0],
                  [0, 2, 1],
                  [0, 0, 0]])

tlist = np.array([n,n,1])

Blist = np.array([[0, -1, 0],
                  [1, 0, 0],
                  [0, 0, 0],
                  [3, 0, 0],
                  [0, 3, 0],
                  [0, 0, 1]])

Jv = np.array([
    [-0.105, 0.000, 0.006, -0.045, 0.000, 0.006, 0.000],
    [-0.889, 0.006, 0.000, -0.844, 0.006, 0.000, 0.000],
    [ 0.000, -0.105, 0.889, 0.000, 0.000, 0.000, 0.000]
])

A = (Jv@Jv.T)

# On utilise eigh car A est symétrique (plus précis et rapide)
valeurs_propres, vecteurs_propres = np.linalg.eigh(A)

# 2. Calculer les longueurs des demi-axes (racine carrée des valeurs propres)
longueurs = np.sqrt(valeurs_propres)

#print("Valeurs propres (lambda) :", valeurs_propres)
#print("Longueurs des demi-axes :", longueurs)
#print(mr.JacobianBody(Blist,tlist))
#print(mr.JacobianSpace(Slist,tlist))
#print(Jb.T@Fb)
#print(J1.T@F1)