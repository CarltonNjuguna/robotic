import numpy as np
import modern_robotics as mr

def IKinBody(Blist, M, T, thetalist0, eomg, ev):

    thetalist = np.array(thetalist0).copy()
    i = 0
    maxiterations = 20
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, \
                                                      thetalist)), T)))
    err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
          or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    while err and i < maxiterations:
        thetalist = thetalist \
                    + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, \
                                                         thetalist)), Vb)
        i = i + 1
        Vb \
        = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(mr.FKinBody(M, Blist, \
                                                       thetalist)), T)))
        err = np.linalg.norm([Vb[0], Vb[1], Vb[2]]) > eomg \
              or np.linalg.norm([Vb[3], Vb[4], Vb[5]]) > ev
    return (thetalist, not err)

W1, W2 = 109, 82
L1, L2 = 425, 392
H1, H2 = 89, 95

M = np.array([
    [-1, 0, 0, L1 + L2],
    [ 0, 0, 1, W1 + W2],
    [ 0, 1, 0, H1 - H2],
    [ 0, 0, 0, 1      ]
])

T_sd = np.array([
    [ 0.0,  1.0,  0.0, -0.5],
    [ 0.0,  0.0, -1.0,  0.1],
    [-1.0,  0.0,  0.0,  0.1],
    [ 0.0,  0.0,  0.0,  1.0]
])

blist = np.array([
    [0, 1, 0, W1 + W2, 0, L1 + L2],        # Articulation 1
    [0, 0, 1, H2, -L1 - L2, 0],            # Articulation 2
    [0, 0, 1, H2, -L2, 0],                 # Articulation 3
    [0, 0, 1, H2, 0, 0],                   # Articulation 4
    [0, -1, 0, -W2, 0, 0],                 # Articulation 5
    [0, 0, 1, 0, 0, 0]                     # Articulation 6
]).T

thetalist = np.array([3.062079, -1.481344, 0.059104, 0.059775, 0.037360, 0.029888])

print(IKinBody(blist,M,T_sd,thetalist,0.001,0.0001))