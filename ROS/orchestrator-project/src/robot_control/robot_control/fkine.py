import numpy as np
import math
from inverse_kinematics import A_dh, DH_Robot

q_motores = [-45, 126, 41] # Reemplazar con valores de los motores!!
q_motores = [math.radians(q) for q in q_motores] # Convierte a radianes

q1 = q_motores[0]
q2 = np.pi/2 - q_motores[1]
q3 = -np.pi/2 + q_motores[1] - q_motores[2]
q4 = -(q2+q3)
q_articulares = [q1, q2, q3, q4]

def fkine(q, dh):
    T = np.eye(4)
    for i in range(4):
        T = T @ A_dh(dh[i], q[i]+offset[i])
    return T

dh, offset, Tool, Base, tool_z = DH_Robot()
T = fkine(q_articulares, dh)
xyz = T[:3, 3]
xyz[2] -= tool_z # Se le resta la herramienta para estar en la parte de abajo del electroiman
print("Posición del efector final:", xyz)

