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

    def kinematic_direct(self, v, phi, L,vGap=[0,0],phiGap=[0,0]):
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


    def kinematic_inverse(self, x_dot, y_dot, theta_dot, L):
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
        v = np.sqrt(x_dot**2 + y_dot**2)
        if v == 0:
            phi = np.nan  # Avoid division by zero
        else:
            phi = np.arctan(theta_dot * L / v)
        return v, phi
    

    