import numpy as np

def rotation_x(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [1, 0,  0],
        [0, c, -s],
        [0, s,  c]
    ])

def rotation_y(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c]
    ])

def rotation_z(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])


angle = np.radians(90)
Rz = rotation_z(angle)

print(f"Matrice de rotation Z pour 90° :\n{np.round(Rz, 2)}")
vecteur = np.array([1, 0, 0])
nouveau_vecteur = Rz @ vecteur

print(f"Vecteur après rotation : {np.round(nouveau_vecteur, 2)}")