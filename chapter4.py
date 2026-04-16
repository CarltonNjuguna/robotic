import numpy as np
import modern_robotics as mr

r3 = np.sqrt(3)
M = np.array([[1,0,0,2+r3],[0,1,0,0],[0,0,1,1+r3],[0,0,0,1]])
Ws = np.array([[0,0,1], [0,1,0], [0,1,0], [0,1,0], [0,0,0], [0,0,1]]) 
Ps = np.array([[1,0,0], [1,0,0], [1+r3,0,-1], [2+r3,0,r3-1], [0,0,0], [2+r3,0,r3+1]])
Vs = [np.cross(-Ws[i],Ps[i]) for i in range(6)]

Wb = np.array([[0,0,1], [0,1,0], [0,1,0], [0,1,0], [0,0,0], [0,0,1]])
Pb = np.array([[-1-r3,0,-1-r3], [-1-r3,0,-1-r3], [-1,0,-2-r3], [0,0,-2], [0,0,0], [0,0,0]])
Vb = [np.cross(-Wb[i],Pb[i]) for i in range(6)]
Vs[4],Vb[4] = np.array([0,0,1]),np.array([0,0,-1])

axeS = [np.hstack((Ws[i],Vs[i])) for i in range(6)]
axeB = [np.hstack((Wb[i],Vb[i])) for i in range(6)]

for vecteur in axeB:
    print(vecteur)


