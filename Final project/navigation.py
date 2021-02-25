from typing import List, Tuple
import time

class Navigation:
    """Class for short-term path planning."""

    estado = 0

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
            w = -kpa * error_angulo - kpd * error_distancia
            v = 1
            
            if z_us[7] >= 0.8:
                self.estado = 21
                self.t2 = time.time()
            elif media_delanteros <= 0.75 and z_us[7] < 2:
                self.estado = 11
        
        if self.estado == 11:
            w = 1.8
            v = 0
            if media_delanteros > 2:
                self.estado = 12

        if self.estado == 12:
            w = 0.5
            v = 0
            if -0.01 < (z_us[7] - z_us[8]) and z_us[7] - z_us[8] < 0.005:
                self.estado = 0

        if self.estado == 21:
            w = -3.5
            v = 0
            if  z_us[7] < 0.8:
                self.estado = 22
                #self.t3 = time.time()

        if self.estado == 22:
            w = -0.7
            v = 0
            if z_us[7] > 2.0:
                self.estado = 3

        if self.estado == 3:
            w = 0
            v = 0.8
            if  z_us[7] < 0.8:
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

        print(self.estado)
        

        return v, w
