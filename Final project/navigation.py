from typing import List, Tuple
import time
import numpy as np
import math as math
class Navigation:
    """Class for short-term path planning."""

    estado = 11

    def __init__(self, dt: float):
        """Navigation class initializer.

        Args:
            dt: Sampling period [s].

        """



        pass

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
        '''

        if self.estado == 0:
            error_angulo = z_us[7] - z_us[8]
            error_distancia = (z_us[7] + z_us[8]) / 2.0 - 0.35
            '''
            if error_angulo > 10.0:
                error_angulo = 10.0
            elif error_angulo < -10.0:
                error_angulo = -10.0
            if error_distancia > 10.0:
                error_distancia = 10.0
            elif error_distancia < -10.0:
                error_distancia = -10.0
                
            '''
            w = -kpa * error_angulo - kpd * error_distancia
            v = 0.9
            
            if z_us[7] >= 0.8:
                self.estado = 21
                self.t2 = time.time()
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
                #self.t3 = time.time()

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
        
        

        '''
        media_delanteros = (z_us[3] + z_us[4]) / 2.0

        if self.estado == 0:
            error = z_us[0] - z_us[7]
            w = error * 5
            v = 0.3
            
            if media_delanteros <= 0.3:
                self.estado = 1
        
        if self.estado == 1:
            w = 1.8
            v = 0
            if media_delanteros > 2:
                self.estado = 0

        '''

        return v, w
    
    def move_control(self,centroid: Tuple[float,float,float], goal: Tuple[float,float],z_us: List[float])-> Tuple[float, float]:
        kpa_th = 1.8
        kpa = 2.0
        kpd = 1.0
        w = 0
        
        error_angulo = centroid[2] - math.atan2(goal[1]-centroid[1],goal[0]-centroid[0])
        if error_angulo > np.pi:
            error_angulo -= 2*np.pi
        
        w_th = -kpa_th * error_angulo


        if z_us[7] < 0.8 and z_us[8] < 0.8:
            error_angulo_p = z_us[7] - z_us[8]
            error_distancia = (z_us[7] + z_us[8]) / 2.0 - 0.35

            w = -kpa * error_angulo_p - kpd * error_distancia
        
        

        w+=w_th

        w_limit = 2.2


        if w > w_limit:
            w = w_limit
        elif w < -w_limit:
            w = -w_limit


        if(abs(error_angulo)) > math.pi/6.0:
            v = 0.0
        else:
            v = 0.8

        
        return v,w
    
