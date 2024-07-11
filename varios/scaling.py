import matplotlib.pyplot as plt
import numpy as np

def dead_zone_scaling(input_value, lower_limit, upper_limit, dead_zone_min, dead_zone_max, al=1.0, ah=1.0):
    """
    Escalización con zona muerta en el centro.
    
    Args:
    input_value (float): El valor de entrada.
    lower_limit (float): El límite inferior de la salida.
    upper_limit (float): El límite superior de la salida.
    dead_zone_min (float): El límite inferior de la zona muerta.
    dead_zone_max (float): El límite superior de la zona muerta.
    al (float, optional): Factor de escala para valores negativos. Default es 1.0.
    ah (float, optional): Factor de escala para valores positivos. Default es 1.0.
    
    Returns:
    float: El valor de salida escalado.
    """
    mid_dead_zone = (dead_zone_min + dead_zone_max) / 2.0

    if input_value == 0:
        return mid_dead_zone
    elif input_value < 0:
        # Escalización lineal para reversa (debajo de la zona muerta)
        return dead_zone_min + (input_value * (dead_zone_min - lower_limit) * al)
    else:
        # Escalización lineal para avanzar (encima de la zona muerta)
        return dead_zone_max + (input_value * (upper_limit - dead_zone_max) * ah)

# Parámetros
lower_limit = 70
upper_limit = 130
dead_zone_min = 85
dead_zone_max = 101

# Crear un rango de valores de entrada
input_values = np.linspace(-1, 1, 500)
output_values = [dead_zone_scaling(x, lower_limit, upper_limit, dead_zone_min, dead_zone_max) for x in input_values]

# Graficar
plt.figure(figsize=(10, 6))
plt.plot(input_values, output_values, label='Dead Zone Scaling')
plt.axhline(y=dead_zone_min, color='r', linestyle='--', label='Dead Zone Min')
plt.axhline(y=dead_zone_max, color='r', linestyle='--', label='Dead Zone Max')
plt.xlabel('Input Value (Velocity Command)')
plt.ylabel('Output Value (PWM)')
plt.title('Dead Zone Scaling Function for Motor Control')
plt.legend()
plt.grid(True)
plt.show()
