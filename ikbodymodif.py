import numpy as np
import modern_robotics as mr

W1, W2 = 0.109, 0.082
L1, L2 = 0.425, 0.392
H1, H2 = 0.089, 0.095

M = np.array([
    [-1, 0, 0, L1 + L2],
    [ 0, 0, 1, W1 + W2],
    [ 0, 1, 0, H1 - H2],
    [ 0, 0, 0, 1      ]
])

blist = np.array([
    [0, 1, 0, W1 + W2, 0, L1 + L2],        # Articulation 1
    [0, 0, 1, H2, -L1 - L2, 0],            # Articulation 2
    [0, 0, 1, H2, -L2, 0],                 # Articulation 3
    [0, 0, 1, H2, 0, 0],                   # Articulation 4
    [0, -1, 0, -W2, 0, 0],                 # Articulation 5
    [0, 0, 1, 0, 0, 0]                     # Articulation 6
]).T

T_sd = np.array([
    [ 0.0,  1.0,  0.0, -0.5],
    [ 0.0,  0.0, -1.0,  0.1],
    [-1.0,  0.0,  0.0,  0.1],
    [ 0.0,  0.0,  0.0,  1.0]
])

thetalist = np.array([0.0, -1.0, 1.0, 0.0, 1.0, 0.0])
eomg = 0.001
ev = 0.0001
thetalist_matrix = np.array([thetalist])
i = 0
err = True

while err:
    T_curr = mr.FKinBody(M, blist, thetalist)
    Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(T_curr), T_sd)))
    
    err_angular = np.linalg.norm(Vb[:3])
    err_linear = np.linalg.norm(Vb[3:])
    err = err_angular > eomg and err_linear > ev
    
    print(f"--- Iteration {i} ---")
    print("End-config (SE3):\n", np.round(T_curr, 4))
    print("Joint vector:", np.round(thetalist, 4))
    print("Error twist (Vb):", np.round(Vb, 4))
    print(f"Angular error: {err_angular:.4f}\nLinear error: {err_linear:.4f}\n")
    
    if err:
        thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(blist, thetalist)), Vb)
        thetalist_matrix = np.vstack((thetalist_matrix, thetalist))
        i += 1

print(f"Convergence reached in {i} iterations.")
print("Thetalist matrix over iterations:\n", thetalist_matrix)

# Expected result: -3.69706963, -18.23390307, 42.2551894, -24.02125206, -0.55547097, -1.57083577