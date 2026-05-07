import numpy as np
import modern_robotics as mr

T_sd = np.array([
    [-0.585, -0.811, 0, 0.076],
    [ 0.811, -0.585, 0, 2.608],
    [ 0.0,    0.0,    1, 0.0  ],
    [ 0.0,    0.0,    0, 1.0  ]
])

Slist = np.array([
    [0, 0, 1, 0,  0, 0], # S1
    [0, 0, 1, 0, -1, 0], # S2
    [0, 0, 1, 0, -2, 0]  # S3
]).T #----- algo demande transposer pas oublier 

M = np.array([
    [1, 0, 0, 3],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
]) #matrice de rotation + translation ZZZZzz

thetalist0 = np.array([np.pi/4, np.pi/4, np.pi/4])

print(mr.IKinSpace(Slist,M,T_sd,thetalist0,0.001,0.0001))