#!/usr/bin/env python3
import numpy as np
import math as m
class Bicycle:
    """
    A class representing a bicycle.

    Attributes:
        None

    Methods:
        kinematic_direct: Calculates the kinematic values in the direct kinematics model.
        kinematic_inverse: Calculates the kinematic values in the inverse kinematics model.
    """
    @staticmethod
    def kinematic_direct( v, phi, L,vGap=[0,0],phiGap=[0,0]):
        """
        Calculates the kinematic values in the direct kinematics model.

        Args:
            v (float): The velocity of the bicycle.
            phi (float): The steering angle of the bicycle.
            L (float): The length of the bicycle.

        Returns:
            tuple: A tuple containing the x_dot, y_dot, and theta_dot values.
        """
        if vGap[0]<v<vGap[1]:
            v=0
        if phiGap[0]<phi<phiGap[1]:
            phi=0
        x_dot = v * m.cos(phi)
        y_dot = v * m.sin(phi)
        theta_dot = v * m.tan(phi) / L
        return x_dot, y_dot, theta_dot

    @staticmethod
    def kinematic_inverse( x_dot, y_dot, theta_dot, L):
        """
        Calculates the kinematic values in the inverse kinematics model.

        Args:
            x_dot (float): The x-axis velocity [m/s] of the bicycle.

            y_dot (float): The y-axis velocity [m/s] of the bicycle.

            theta_dot (float): The angular velocity [rad/s] of the bicycle.

            L (float): The length of the bicycle.

        Returns:
            tuple: A tuple containing the v and phi values.

            v: The linear speed of the wheel [m/s].

            phi: The steering angle of the bicycle [rad].
        """
        # v = m.sqrt(x_dot**2 + y_dot**2) * x_dot/abs(x_dot)
        v=x_dot
        if v == 0:
            phi = 0.0  # Avoid division by zero
        else:
            phi = m.atan(theta_dot * L / v)
        return v, phi
    
    def limited(x,low,high):
        return max(low,min(x,high))
    def dead_zone_scaling(input_value, lower_limit, upper_limit, dead_zone_min, dead_zone_max,al=1.0,ah=1.0):
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
            return dead_zone_min + (input_value * (dead_zone_min - lower_limit) *al)
        else:
            # Escalización lineal para avanzar (encima de la zona muerta)
            return dead_zone_max + (input_value * (upper_limit - dead_zone_max)*ah)

        