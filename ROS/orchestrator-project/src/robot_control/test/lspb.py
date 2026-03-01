import numpy as np
from inverse_kinematics import cinv_geometrica, DH_Robot
from Bspline import b_spline_trajectory
import matplotlib.pyplot as plt
import time

t1 = time.time()

# -------------------------------
# Parametros
# -------------------------------
[dh, offset, Tool, Base, tool_z] = DH_Robot()

# Generar trayectoria B-Spline
x_fino, y_fino, z_fino = b_spline_trajectory()
N = len(x_fino)
# Trayectoria cartesiana ya generada (x_fino, y_fino, z_fino)
# ----------------------------------------------------------

# -------------------------------
# Cinematica inversa
# -------------------------------
q_eslabon = np.zeros((N, 3))

for k in range(N):
    T_obj = np.array([
        [1, 0, 0, x_fino[k]],
        [0, 1, 0, y_fino[k]],
        [0, 0, 1, z_fino[k] + tool_z],
        [0, 0, 0, 1]
    ])

    q_eslabon[k, :] = cinv_geometrica(dh, T_obj, Base, Tool, offset)

# -------------------------------
# Parametro geometrico
# -------------------------------
s_geom = np.linspace(0, 1, N)
ds = s_geom[1] - s_geom[0]

dq_ds  = np.gradient(q_eslabon, ds, axis=0)
ddq_ds = np.gradient(dq_ds, ds, axis=0)

# -------------------------------
# Limites articulares
# -------------------------------
qd_max  = np.array([39.5833, 84.375, 84.375])     # [deg/s]
qdd_max = np.array([300.0, 300.0, 300.0])         # [deg/s^2]

s_dot_lim  = np.inf
s_ddot_lim = np.inf

for i in range(3):
    s_dot_lim  = min(s_dot_lim,
                     np.min(qd_max[i] / np.maximum(np.abs(dq_ds[:, i]), 1e-6)))
    s_ddot_lim = min(s_ddot_lim,
                     np.min(qdd_max[i] / np.maximum(np.abs(ddq_ds[:, i]), 1e-6)))

# -------------------------------
# LSPB REAL (equivalente a trapveltraj)
# -------------------------------
s_dot_max  = s_dot_lim
s_ddot_max = s_ddot_lim

Ta = s_dot_max / s_ddot_max
Tf = (1.0 / s_dot_max) + Ta

t = np.linspace(0, Tf, N)
s      = np.zeros(N)
s_dot  = np.zeros(N)
s_ddot = np.zeros(N)

for k, tk in enumerate(t):
    if tk < Ta:
        # Aceleracion
        s[k]      = 0.5 * s_ddot_max * tk**2
        s_dot[k]  = s_ddot_max * tk
        s_ddot[k] = s_ddot_max

    elif tk < Tf - Ta:
        # Velocidad constante
        s[k]      = s_dot_max * (tk - Ta/2)
        s_dot[k]  = s_dot_max
        s_ddot[k] = 0.0

    else:
        # Deceleracion
        td = Tf - tk
        s[k]      = 1 - 0.5 * s_ddot_max * td**2
        s_dot[k]  = s_ddot_max * td
        s_ddot[k] = -s_ddot_max

# Normalizacion numerica (por seguridad)
s[-1] = 1.0
s_dot[0] = s_dot[-1] = 0.0

# -------------------------------
# Reparametrizacion temporal
# -------------------------------
q   = np.zeros((N, 3))
qd  = np.zeros((N, 3))
qdd = np.zeros((N, 3))

for i in range(3):
    q[:, i]   = np.interp(s, s_geom, q_eslabon[:, i])
    dq_i      = np.interp(s, s_geom, dq_ds[:, i])
    ddq_i     = np.interp(s, s_geom, ddq_ds[:, i])

    qd[:, i]  = dq_i * s_dot
    qdd[:, i] = ddq_i * s_dot**2 + dq_i * s_ddot

# -------------------------------
# Verificacion clave
# -------------------------------
print("Tiempo total:", Tf)
print("Tiempo de ejecucion:", time.time() - t1)

# Graficar resultados
plt.figure()
plt.subplot(3, 1, 1)
plt.plot(t, q)
plt.ylabel('q [deg]')
plt.grid(True)
plt.subplot(3, 1, 2)
plt.plot(t, qd)
plt.ylabel('qd [deg/s]')
plt.grid(True)
plt.subplot(3, 1, 3)
plt.plot(t, qdd)
plt.ylabel('qdd [deg/s²]')
plt.xlabel('t [s]')
plt.grid(True)
plt.tight_layout()
plt.show()

plt.figure()
plt.subplot(3, 1, 1)
plt.plot(t, s)
plt.ylabel('s')
plt.grid(True)
plt.subplot(3, 1, 2)
plt.plot(t, s_dot)
plt.ylabel('ṡ')
plt.grid(True)
plt.subplot(3, 1, 3)
plt.plot(t, s_ddot)
plt.ylabel('s_ddot')
plt.xlabel('t [s]')
plt.grid(True)
plt.tight_layout()
plt.show()

def verificar_workspace_articular(q_eslabon, csv_file="Tabla_angulos_Robot.csv"):
    """
    Verifica si una trayectoria está dentro del workspace articular.
    q_eslabon: ndarray (N,3) -> [q1, q2, q3] en grados
    csv_file: archivo CSV de la tabla del workspace
    """

    # -----------------------------
    # Verificación GDL 1
    # -----------------------------
    q1 = q_eslabon[:, 0]

    if np.min(q1) < -45 or np.max(q1) > 90:
        print("No es posible realizar la trayectoria, no verifica el GDL 1")
        return False

    T = np.genfromtxt(csv_file, delimiter=';', dtype=int) # Lectura tabla CSV
    M = T[1:, 1:]

    offset_filas = int(T[1, 0] - 1)
    offset_columnas = int(T[0, 1] - 1)

    n_filas, n_cols = M.shape

    # -----------------------------
    # Verificación de límites tabla
    # -----------------------------
    max_q3 = int(np.ceil(np.max(q_eslabon[:, 2]))) - offset_filas
    max_q2 = int(np.ceil(np.max(q_eslabon[:, 1]))) - offset_columnas

    if max_q3 >= n_filas or max_q2 >= n_cols:
        print("No es posible realizar la trayectoria, fuera del WS articular")
        return False

    # -----------------------------
    # Verificación punto a punto
    # -----------------------------
    for i in range(q_eslabon.shape[0]):
        fila = int(np.ceil(q_eslabon[i, 2])) - offset_filas
        col  = int(np.ceil(q_eslabon[i, 1])) - offset_columnas

        # Seguridad extra (Python no perdona)
        if fila < 0 or col < 0 or fila >= n_filas or col >= n_cols:
            print("No es posible realizar la trayectoria, fuera del WS articular")
            return False

        if M[fila, col] == 0:
            print("No es posible realizar la trayectoria, no verifican los GDL 2 y 3")
            return False

    print("Trayectoria correcta, todos los puntos verifican")

    """ nj = q.shape[1]

    fig, axs = plt.subplots(3, 1, sharex=True)

    for j in range(nj):
        axs[0].plot(tt, q[:, j], label=f'q{j+1}')
        axs[1].plot(tt, qd[:, j], label=f'q̇{j+1}')
        axs[2].plot(tt, qdd[:, j], label=f'q̈{j+1}')

    axs[0].set_ylabel('q [deg]')
    axs[1].set_ylabel('q̇ [deg/s]')
    axs[2].set_ylabel('q̈ [deg/s²]')
    axs[2].set_xlabel('t [s]')

    for ax in axs:
        ax.grid(True)
        ax.legend()

    plt.tight_layout()
    plt.show() """

    return True

verificar_workspace_articular(q)