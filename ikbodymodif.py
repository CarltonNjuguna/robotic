import numpy as np
import modern_robotics as mr
import contextlib

def IKinBodyIterates(Blist, M, T, thetalist0, eomg, ev, maxiterations):
    thetalist = np.array(thetalist0).copy()
    thetalist_matrix = np.array([thetalist])
    i = 0
    err = True
    # Iteratively update the joint angles until the error is within our tolerances
    while err and i < maxiterations:
        T_curr = mr.FKinBody(M, Blist, thetalist)
        Vb = mr.se3ToVec(mr.MatrixLog6(np.dot(mr.TransInv(T_curr), T)))
        
        err_angular = np.linalg.norm(Vb[:3])
        err_linear = np.linalg.norm(Vb[3:])
        err = err_angular > eomg and err_linear > ev
        
        print(f"--- Iteration {i} ---")
        print("Joint vector:", np.round(thetalist, 4))
        print("End-config (SE3):\n", np.round(T_curr, 4))
        print("Error twist (Vb):", np.round(Vb, 4))
        print(f"Angular error: {err_angular:.4f}\nLinear error: {err_linear:.4f}\n")
        
        if err:
            # Calculate the new joint angles using the Jacobian pseudo-inverse
            thetalist = thetalist + np.dot(np.linalg.pinv(mr.JacobianBody(Blist, thetalist)), Vb)
            thetalist_matrix = np.vstack((thetalist_matrix, thetalist))
            i += 1

    print(f"Convergence reached in {i} iterations.")
    print("Thetalist matrix over iterations:\n", thetalist_matrix)
        
    return thetalist_matrix, not err

# robot configuration
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
    [0, 1, 0, W1 + W2, 0, L1 + L2],        # Joint 1
    [0, 0, 1, H2, -L1 - L2, 0],            # Joint 2
    [0, 0, 1, H2, -L2, 0],                 # Joint 3
    [0, 0, 1, H2, 0, 0],                   # Joint 4
    [0, -1, 0, -W2, 0, 0],                 # Joint 5
    [0, 0, 1, 0, 0, 0]                     # Joint 6
]).T

# the desired target pose
T_sd = np.array([
    [ 0.0,  1.0,  0.0, -0.5],
    [ 0.0,  0.0, -1.0,  0.1],
    [-1.0,  0.0,  0.0,  0.1],
    [ 0.0,  0.0,  0.0,  1.0]
])

# error tolerances
eomg = 0.001
ev = 0.0001

thetalist = np.array([np.pi, -np.pi/4, np.pi/2, -np.pi/2, -np.pi/5, -np.pi/4]) #config elbow 90 deg

with open("ik_logs.txt", "w") as f:
    with contextlib.redirect_stdout(f):
        thetalist_matrix, success = IKinBodyIterates(blist, M, T_sd, thetalist, eomg, ev, 20)

with open("iterations.csv", "w") as f:
    for row in thetalist_matrix:
        f.write(",".join([str(val) for val in row]) + "\n")

print("The execution logs have been successfully saved to 'ik_logs.txt'.")
print("The matrix has been successfully saved in 'iterations.csv'.")