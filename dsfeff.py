import numpy as np

def twist_to_transformation(S_theta):
    # 1. Extraction des composantes
    # S_theta = [w1, w2, w3, v1, v2, v3]
    w_theta = S_theta[0:3]
    v_theta = S_theta[3:6]
    
    # 2. Calcul de l'amplitude theta
    theta = np.linalg.norm(w_theta)
    
    if theta == 0:
        # Cas particulier : translation pure (si theta est nul)
        R = np.eye(3)
        p = v_theta
    else:
        # Normalisation pour obtenir l'axe unitaire
        w_hat_vec = w_theta / theta
        v_hat_vec = v_theta / theta
        
        # Matrice antisymétrique [w_hat]
        w_hat = np.array([[0, -w_hat_vec[2],  w_hat_vec[1]],
                          [w_hat_vec[2], 0, -w_hat_vec[0]],
                          [-w_hat_vec[1], w_hat_vec[0], 0]])
        
        # 3. Formule de Rodrigues pour la rotation R
        I = np.eye(3)
        R = I + np.sin(theta) * w_hat + (1 - np.cos(theta)) * np.dot(w_hat, w_hat)
        
        # 4. Calcul du vecteur de translation p
        # Formule : p = [I*theta + (1-cos)*[w_hat] + (theta-sin)*[w_hat]^2] * v_hat
        G_theta = I * theta + (1 - np.cos(theta)) * w_hat + (theta - np.sin(theta)) * np.dot(w_hat, w_hat)
        p = np.dot(G_theta, v_hat_vec)

    # 5. Assemblage de la matrice 4x4
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = p
    
    return T

# Coordonnées du problème : (0, 1, 2, 3, 0, 0)
S_theta_input = np.array([0, 1, 2, 3, 0, 0])

# Exécution
resultat = twist_to_transformation(S_theta_input)

# Affichage avec précision
print("Matrice de transformation homogène T :")
print(np.round(resultat, 4))