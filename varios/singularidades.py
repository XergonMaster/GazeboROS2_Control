#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

# Modelo Bicicleta
# Suponemos la velocidad en la llanta trasera

def x_dot(v, phi):
    return v * np.cos(phi)

def y_dot(v, phi):
    return v * np.sin(phi)

def theta_dot(v, phi, L):
    return v * np.tan(phi) / L

def v_from_xy_dot(x_dot, y_dot):
    return np.sqrt(x_dot**2 + y_dot**2)

def phi_from_xy_dot(x_dot, y_dot):
    return np.arctan2(y_dot, x_dot)

def phi_(v, theta_dot, L):
    RA2DE = 180 / np.pi
    if v == 0:
        return np.nan  # Evitar división por cero
    return np.arctan(theta_dot * L / v) * RA2DE

# Parámetros del modelo
L = 1.0  # Longitud entre ejes

# Rango de valores
step = 201
rango = 2
v_values = np.linspace(-rango, rango, step)
phi_values = np.linspace(-np.pi/4 + 0.1, np.pi/4 - 0.1, step)

x_dot_values = np.linspace(-rango, rango, step)
y_dot_values = np.linspace(-rango, rango, step)
theta_dot_values = np.linspace(-np.pi/4 + 0.1, np.pi/4 - 0.1, step)

# Inicializar matrices de resultados
x_dot_values_img = np.zeros((len(v_values), len(phi_values)))
y_dot_values_img = np.zeros((len(v_values), len(phi_values)))
theta_dot_values_img = np.zeros((len(v_values), len(phi_values)))

v_values_img = np.zeros((len(x_dot_values), len(y_dot_values)))
phi_dot_values_img = np.zeros((len(x_dot_values), len(y_dot_values)))
phi_values_img = np.zeros((len(v_values), len(theta_dot_values)))

# Verificar singularidades cinemática directa
for i, v in enumerate(v_values):
    for j, phi in enumerate(phi_values):
        x_dot_values_img[i, j] = x_dot(v, phi)
        y_dot_values_img[i, j] = y_dot(v, phi)
        theta_dot_values_img[i, j] = theta_dot(v, phi, L)

# Invertir los valores de x_dot_values_img, y_dot_values_img y theta_dot_values_img
x_dot_values_img = np.flipud(x_dot_values_img)
y_dot_values_img = np.flipud(y_dot_values_img)
theta_dot_values_img = np.flipud(theta_dot_values_img)

# Calcular v y phi_dot para las imágenes
for i, vx in enumerate(x_dot_values):
    for j, vy in enumerate(y_dot_values):
        v_values_img[i, j] = v_from_xy_dot(vx, vy)
        phi_dot_values_img[i, j] = phi_from_xy_dot(vx, vy)

v_values_img = np.flipud(v_values_img)
phi_dot_values_img = np.flipud(phi_dot_values_img)

for i, v in enumerate(v_values):
    for j, vt in enumerate(theta_dot_values):
        try:
            phi_values_img[i, j] = phi_(v, vt, L)
        except:
            phi_values_img[i, j] = np.nan

phi_values_img = np.flipud(phi_values_img)

# Crear figura y subplots
fig = plt.figure(figsize=(12, 8))

# Subplot X_Dot
ax1 = fig.add_subplot(3, 2, 1)
ax1.set_title('X_Dot (m/s)')
ax1.set_xlabel('Phi (rad)')
ax1.set_ylabel('V (m/s)')
im1 = ax1.imshow(x_dot_values_img, extent=[phi_values[0], phi_values[-1], v_values[0], v_values[-1]], aspect='auto')
fig.colorbar(im1, ax=ax1)

# Identificar singularidades y graficar puntos rojos
x_dot_singularities = np.isnan(x_dot_values_img) | np.isinf(x_dot_values_img)
singularities_y, singularities_x = np.where(x_dot_singularities)
ax1.scatter(phi_values[singularities_x], v_values[singularities_y], color='red', s=1)

# Subplot Y_Dot
ax2 = fig.add_subplot(3, 2, 2)
ax2.set_title('Y_Dot (m/s)')
ax2.set_xlabel('Phi (rad)')
ax2.set_ylabel('V (m/s)')
im2 = ax2.imshow(y_dot_values_img, extent=[phi_values[0], phi_values[-1], v_values[0], v_values[-1]], aspect='auto')
fig.colorbar(im2, ax=ax2)

# Identificar singularidades y graficar puntos rojos
y_dot_singularities = np.isnan(y_dot_values_img) | np.isinf(y_dot_values_img)
singularities_y, singularities_x = np.where(y_dot_singularities)
ax2.scatter(phi_values[singularities_x], v_values[singularities_y], color='red', s=1)

# Subplot Theta_Dot
ax3 = fig.add_subplot(3, 2, 3)
ax3.set_title('Theta_Dot (rad/s)')
ax3.set_xlabel('Phi (rad)')
ax3.set_ylabel('V (m/s)')
im3 = ax3.imshow(theta_dot_values_img, extent=[phi_values[0], phi_values[-1], v_values[0], v_values[-1]], aspect='auto')
fig.colorbar(im3, ax=ax3)

# Identificar singularidades y graficar puntos rojos
theta_dot_singularities = np.isnan(theta_dot_values_img) | np.isinf(theta_dot_values_img)
singularities_y, singularities_x = np.where(theta_dot_singularities)
ax3.scatter(phi_values[singularities_x], v_values[singularities_y], color='red', s=1)

# Subplot V
ax4 = fig.add_subplot(3, 2, 4)
ax4.set_title('V (m/s)')
ax4.set_xlabel('X_Dot (m/s)')
ax4.set_ylabel('Y_Dot (m/s)')
im4 = ax4.imshow(v_values_img, extent=[x_dot_values[0], x_dot_values[-1], y_dot_values[0], y_dot_values[-1]], aspect='auto')
fig.colorbar(im4, ax=ax4)

# Identificar singularidades y graficar puntos rojos
v_singularities = np.isnan(v_values_img) | np.isinf(v_values_img)
singularities_y, singularities_x = np.where(v_singularities)
ax4.scatter(x_dot_values[singularities_x], y_dot_values[singularities_y], color='red', s=1)

# Subplot Phi_Dot
ax5 = fig.add_subplot(3, 2, 5)
ax5.set_title('Phi_Dot (rad)')
ax5.set_xlabel('X_Dot (m/s)')
ax5.set_ylabel('Y_Dot (m/s)')
im5 = ax5.imshow(phi_dot_values_img, extent=[x_dot_values[0], x_dot_values[-1], y_dot_values[0], y_dot_values[-1]], aspect='auto')
fig.colorbar(im5, ax=ax5)

# Identificar singularidades y graficar puntos rojos
phi_dot_singularities = np.isnan(phi_dot_values_img) | np.isinf(phi_dot_values_img)
singularities_y, singularities_x = np.where(phi_dot_singularities)
ax5.scatter(x_dot_values[singularities_x], y_dot_values[singularities_y], color='red', s=1)

# Subplot Phi_
ax6 = fig.add_subplot(3, 2, 6)
ax6.set_title('Phi_ (deg)')
ax6.set_ylabel('V (m/s)')
ax6.set_xlabel('Theta_dot (rad/s)')
im6 = ax6.imshow(phi_values_img, extent=[theta_dot_values[0], theta_dot_values[-1], v_values[0], v_values[-1]], aspect='auto')
fig.colorbar(im6, ax=ax6)

# Identificar singularidades y graficar puntos rojos
phi_singularities = np.isnan(phi_values_img) | np.isinf(phi_values_img)
singularities_y, singularities_x = np.where(phi_singularities)
ax6.scatter(theta_dot_values[singularities_x], v_values[singularities_y], color='red', s=1)

plt.tight_layout()
plt.show()
