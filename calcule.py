import numpy as np

a = np.array([
        [0,1,0],
        [0,0,1],
        [1,0,0]
    ])

b = np.array([
        [1,0,0],
        [0,0,1],
        [0,-1,0]
    ])

s = np.array([
        [1,0,0],
        [0,1,0],
        [0,0,1]
    ])

p = np.array([1,2,3])
ang = np.array([3,2,1])

print(a.T@ang)