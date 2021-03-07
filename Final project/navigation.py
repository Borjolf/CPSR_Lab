from typing import List, Tuple
import time
import numpy as np
import math as math
class Navigation:
    """Class for short-term path planning."""

    def __init__(self, dt: float, z_us:List[float]):
        """Navigation class initializer.

        Args:
            dt: Sampling period [s].
            z_us: distances sensed so we can determine the first state

        """

        if z_us[7] < 0.9 and z_us[8] < 0.9:
            self.estado = 0
        elif z_us[3] < 0.6 and z_us[4] < 0.6:
            self.estado = 11
        else:
            self.estado = 4


    def explore(self, z_us: List[float], z_v: float, z_w: float) -> Tuple[float, float]:
        """Wall following exploration algorithm.

        Args:
            z_us: Distance from every ultrasonic sensor to the closest obstacle [m].
            z_v: Linear velocity of the robot center [m/s].
            z_w: Angular velocity of the robot center [rad/s].

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """
        
        media_delanteros = (z_us[3] + z_us[4]) / 2.0
        
        kpa = 20.0
        kpd = 5.0
        
        '''
        0: avanzar recto siguiendo la pared derecha
        11: girar hacia la izquierda 1
        12: girar hacia la izquierda 2
        21: girar hacia la derecha 1
        22: girar hacia la derecha 2
        3: avanzar un poco tras girar a la derecha

        4: estado solo accesible al iniciar, si no detectamos pared a la derecha ni en frente
        '''

        if self.estado == 0:
            error_angulo = z_us[7] - z_us[8]
            error_distancia = (z_us[7] + z_us[8]) / 2.0 - 0.35

            w = -kpa * error_angulo - kpd * error_distancia
            v = 0.9
            
            if z_us[7] >= 0.8:
                self.estado = 21
            elif media_delanteros <= 0.75 and z_us[7] < 2:
                self.estado = 11
        
        elif self.estado == 11:
            w = 1.8
            v = 0
            if media_delanteros > 2:
                self.estado = 12

        elif self.estado == 12:
            w = 0.5
            v = 0
            if -0.01 < (z_us[7] - z_us[8]) and z_us[7] - z_us[8] < 0.005:
                self.estado = 0

        elif self.estado == 21:
            w = -2.5
            v = 0
            if  z_us[7] < 0.8:
                self.estado = 22

        elif self.estado == 22:
            w = -0.5
            v = 0
            if z_us[7] > 2.0:
                self.estado = 3

        elif self.estado == 3:
            w = 0
            v = 0.5
            if  z_us[7] < 0.9:
                self.estado = 0
            elif media_delanteros < 0.05:
                self.estado = 11

        elif self.estado == 4:
            w = 0.0
            v = 0.4
            if  z_us[7] <= 0.95:
                self.estado = 0
            elif media_delanteros <= 0.6:
                self.estado = 11

        return v, w
    
    def move_control(self,centroid: Tuple[float,float,float], goal: Tuple[float,float],z_us: List[float])-> Tuple[float, float]:
        """Path following algorithm.

        Args:
            z_us: Distance from every ultrasonic sensor to the closest obstacle [m].
            goal: next node of the path that our robot has to reach
            centroid: current location of the robot calculated by our particle filter

        Returns:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].

        """

        kpa_th = 1.8
        kpa = 2.0
        kpd = 1.0
        
        #next node orientation control
        error_angulo = centroid[2] - math.atan2(goal[1]-centroid[1],goal[0]-centroid[0])

        if error_angulo > np.pi:
            error_angulo -= 2*np.pi
        elif error_angulo < -np.pi:
            error_angulo += 2*np.pi
        
        w_th = -kpa_th * error_angulo

        #right-hand wall orientation and distance control, we only use ir if our ultrasonic readings are valid
        if z_us[7] < 0.8 and z_us[8] < 0.8:
            error_angulo_p = z_us[7] - z_us[8]
            error_distancia = (z_us[7] + z_us[8]) / 2.0 - 0.35

            w_p = -kpa * error_angulo_p - kpd * error_distancia
        else:
            w_p = 0
        
        #we add both control so they can fight each other
        w = w_th + w_p

        #we limit the w:
        w_limit = 2.2
        if w > w_limit:
            w = w_limit
        elif w < -w_limit:
            w = -w_limit

        #if we are too awry from the next node, we reduce the linear velocity (specially useful in curves)
        if(abs(error_angulo)) > math.pi/6.0:
            v = 0.0
        else:
            v = 0.85

        return v,w
    
