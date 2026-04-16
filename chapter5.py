import numpy as np

r = np.sqrt(2)/2

J1 = np.array([[1,1,1],[0,0,r],[0,-1,-1-r]])
F1 = np.array([0,2,0])

print(J1.T@F1)