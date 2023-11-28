import math
import numpy as np

class Frameconv:
    def __init__(self):
        pass

    def euler2quaternion(phi,theta, psi):
        sinPhi   = math.sin(phi/2);    cosPhi   = math.cos(phi/2);
        sinTheta = math.sin(theta/2);  cosTheta = math.cos(theta/2);
        sinPsi   = math.sin(psi/2);    cosPsi   = math.cos(psi/2);

        return np.array([cosPhi*cosTheta*cosPsi + sinPhi*sinTheta*sinPsi,
                         sinPhi*cosTheta*cosPsi - cosPhi*sinTheta*sinPsi,
                         cosPhi*sinTheta*cosPsi + sinPhi*cosTheta*sinPsi,
                         cosPhi*cosTheta*sinPsi - sinPhi*sinTheta*cosPsi
                         ])

