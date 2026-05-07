import numpy as np

def fonction(v):
    x, y = v
    return np.array([
        x**2 - 9, 
        y**2 - 4
    ])

def deriver(v):
    x, y = v
    return np.array([
        [2*x, 0  ],
        [0,   2*y]
    ])

def newton_raphson_systeme(f, jacobienne, x_start, n_iterations):
    x = np.array(x_start, dtype=float)
    
    print(f"{x}")

    for i in range(1, n_iterations + 1):
        f_val = f(x)
        j_val = jacobienne(x)
        
        delta = np.linalg.solve(j_val, f_val)
        x = x - delta
        
        print(f"{i} : x = {np.round(x, 4)}")
        
    return x

newton_raphson_systeme(fonction, deriver, [1,1], 7)
