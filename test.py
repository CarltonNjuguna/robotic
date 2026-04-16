import modern_robotics as mr
import numpy as np

print(mr.VecToso3(np.array([1, 2, 0.5])))

R = np.array([[0,0,1],[-1,0,0],[0,-1,0]])
w = np.array([[0,0.5,-1],[-0.5,0,2],[1,-2,0]])
print(mr.MatrixExp3(w))
print(mr.MatrixLog3(R))

#print(mr.MatrixExp3(mr.VecToso3(np.array([1, 2, 0]))))