import math
import numpy as np
import os
from numpy import random

from map import Map
from matplotlib import pyplot as plt
from typing import List, Tuple


class ParticleFilter:
    """Particle filter implementation."""

    def __init__(self, map_object: Map, sensors: List[Tuple[float, float, float]],
                 sensor_range: float, particle_count: int = 1200, sense_noise: float = 0.5*1.2,
                 v_noise: float = 0.05*10, w_noise: float = 0.05*10, figure_size: Tuple[float, float] = (7, 7)):
        """Particle filter class initializer.

        Args:
            map_object: Map of the environment.
            sensors: Robot sensors location [m] and orientation [rad] in the robot coordinate frame (x, y, theta).
            sensor_range: Sensor measurement range [m]
            particle_count: Number of particles.
            sense_noise: Measurement standard deviation [m].
            v_noise: Linear velocity standard deviation [m/s].
            w_noise: Angular velocity standard deviation [rad/s].
            figure_size: Figure window dimensions.

        """
        self._map = map_object
        self._sensors = sensors
        self._sense_noise = sense_noise
        self._sensor_range = sensor_range
        self._v_noise = v_noise
        self._w_noise = w_noise
        self._iteration = 0
        self.localized = False

        self._particles = self._init_particles(particle_count)
        self._ds, self._phi = self._init_sensor_polar_coordinates(sensors)
        self._figure, self._axes = plt.subplots(1, 1, figsize=figure_size)

        self.centroid = (0.0,0.0,0.0)

    def move(self, v: float, w: float, dt: float):
        """Performs a motion update on the particles.

        Args:
            v: Linear velocity [m/s].
            w: Angular velocity [rad/s].
            dt: Sampling time [s].

        """
        self._iteration += 1

        for i in range(len(self._particles)):
            vn = v + random.normal(0, self._v_noise, None)
            wn = w + random.normal(0, self._w_noise, None)

            x1 , y1 , th1 = self._particles[i]
            th2 = (dt * wn) + th1
            if th2 >= 2*math.pi:
                th2 -= 2*math.pi
            elif th2 < 0:
                th2 += 2*math.pi
            x2 = (dt * vn * math.cos((th1+th2)/2.0)) + x1
            y2 = (dt * vn * math.sin((th1+th2)/2.0)) + y1 

            segment = [ (x1,y1) , (x2,y2) ]

            colision, _ = self._map.check_collision(segment, False)

            if colision:
                x2 = colision[0]
                y2 = colision[1]

            self._particles[i] = x2 , y2 , th2

        return


    def _center_found(self, threshold: float = 0.4 ):

        flag = 0

        for j in range(len(self._particles)):

            x_particle, y_particle, _ = self._particles[j]
            x_centroid, y_centroid, _ = self.centroid

            distance_x = abs(x_particle - x_centroid)
            distance_y = abs(y_particle - y_centroid)

            if distance_x > threshold and distance_y > threshold and flag == 0:
                flag = 1

        if flag == 0:
            self.localized = True
      
        return
        
           


    def resample(self, measurements: List[float]):
        """Samples a new set of set of particles using the resampling wheel method.

        Args:
            measurements: Sensor measurements [m].

        """
        # TODO: Complete with your code.

        new_particles = []

        beta = 0
        index = 1 + random.randint(len(self._particles)-1)
        p = [0] * len(self._particles)

        for i in range(len(self._particles)):
            p[i] = self._measurement_probability(measurements, self._particles[i])

        p_max = max(p)

        for j in range(len(p)):
            beta = beta + (random.rand() * 2*p_max)

            while p[index] < beta:
                beta = beta - p[index]
                index = index + 1
                if index >= len(self._particles):
                    index = 0
            
            new_particles.append(self._particles[index])

        self._particles = np.array(new_particles)

        localized_ant = self.localized

        self.cluster_centroid() #calculate centroid, and in that method we check if we have converged already
        
        if localized_ant != self.localized: #reduce to 3 particles only and change noises
            print("localized!")
            reduced_particles = []
            for k in range(40):
                #reduced_particles.append(new_particles[k])
                reduced_particles.append(self.centroid)
            self._particles = np.array(reduced_particles)
            self._sense_noise = self._sense_noise
            self._v_noise = self._v_noise *1.5
            self._w_noise = self._w_noise *1.5
        

        return

    def plot(self, axes, orientation: bool = True):
        """Draws particles.

        Args:
            axes: Figure axes.
            orientation: Draw particle orientation.

        Returns:
            axes: Modified axes.

        """
        if orientation:
            dx = [math.cos(particle[2]) for particle in self._particles]
            dy = [math.sin(particle[2]) for particle in self._particles]
            axes.quiver(self._particles[:, 0], self._particles[:, 1], dx, dy, color='b', scale=15, scale_units='inches')
            
    
            
            axes.quiver(self.centroid[0], self.centroid[1], math.cos(self.centroid[2]), math.sin(self.centroid[2]), color='r',scale=7, scale_units='inches')
        else:
            axes.plot(self._particles[:, 0], self._particles[:, 1], 'bo', markersize=1)

        return axes

    def show(self, title: str = '', orientation: bool = True, display: bool = True,
             save_figure: bool = False, save_dir: str = 'img'):
        """Displays the current particle set on the map.

        Args:
            title: Plot title.
            orientation: Draw particle orientation.
            display: True to open a window to visualize the particle filter evolution in real-time. Time consuming.
            save_figure: True to save figure to a .png file.
            save_dir: Image save directory.

        """
        figure = self._figure
        axes = self._axes
        axes.clear()

        axes = self._map.plot(axes)
        axes = self.plot(axes, orientation)

        axes.set_title(title + ' (Iteration #' + str(self._iteration) + ')')
        figure.tight_layout()  # Reduce white margins

        if display:
            plt.show(block=False)
            plt.pause(0.001)  # Wait for 0.1 ms or the figure won't be displayed

        if save_figure:
            if not os.path.isdir(save_dir):
                os.mkdir(save_dir)

            file_name = str(self._iteration).zfill(4) + ' ' + title.lower() + '.png'
            file_path = os.path.join(save_dir, file_name)
            figure.savefig(file_path)

    def cluster_centroid(self):

        x_centroid = 0
        y_centroid = 0
        th_centroid = 0

        for j in range(len(self._particles)):
            x_particle, y_particle, th_particle = self._particles[j]
            x_centroid += x_particle
            y_centroid += y_particle
            #th_centroid += (th_particle)
        
        x_centroid = x_centroid/len(self._particles)
        y_centroid = y_centroid/len(self._particles)
        #th_centroid = (th_centroid/len(self._particles))

        self.centroid = x_centroid,y_centroid,self._particles[0][2]

        self._center_found()



        return


    def _init_particles(self, particle_count: int) -> np.ndarray:
        """Draws N random valid particles.

        The particles are guaranteed to be inside the map and
        can only have the following orientations [0, pi/2, pi, 3pi/2].

        Args:
            particle_count: Number of particles.

        Returns: A numpy array of tuples (x, y, theta).

        """
        #particles = np.zeros((particle_count, 3), dtype=object)
        particles = []

        #x_min, y_min, x_max, y_max = self._map.bounds()
        count_x = [-4,-3,-2,-1,0,1,2,3,4]
        count_y = [-4,-3,-2,-1,0,1,2,3,4]
        count_th = [0, math.pi/2, math.pi, 3*math.pi/2]

        while particle_count > 0:
            for x in count_x:
                for y in count_y:
                    if self._map.contains((x,y)):
                        for th in count_th:
                            particles.append((x,y,th))
                            particle_count -= 1

        particles = np.array(particles)

        return particles

    @staticmethod
    def _init_sensor_polar_coordinates(sensors: List[Tuple[float, float, float]]) -> Tuple[List[float], List[float]]:
        """Converts the robots sensor location and orientation to polar coordinates wrt to the robot's coordinate frame.

        Args:
            sensors: Robot sensors location [m] and orientation [rad] (x, y, theta).

        Return:
            ds: List of magnitudes [m].
            phi: List of angles [rad].

        """
        ds = [math.sqrt(sensor[0] ** 2 + sensor[1] ** 2) for sensor in sensors]
        phi = [math.atan2(sensor[1], sensor[0]) for sensor in sensors]

        return ds, phi

    def _sense(self, particle: Tuple[float, float, float]) -> List[float]:
        """Obtains the predicted measurement of every sensor given the robot's location.

        Args:
            particle: Particle pose (x, y, theta) in [m] and [rad].

        Returns: List of predicted measurements; inf if a sensor is out of range.

        """
        rays = self._sensor_rays(particle)

        z_hat = [None] * len(rays)

        for i in range(len(rays)):
            _ , distance = self._map.check_collision(rays[i], True)
            z_hat[i] = distance
    
        # TODO: Complete with your code.

        return z_hat

    @staticmethod
    def _gaussian(mu: float, sigma: float, x: float) -> float:
        """Computes the value of a Gaussian.

        Args:
            mu: Mean.
            sigma: Standard deviation.
            x: Variable.

        Returns:
            float: Gaussian.

        """
        # TODO: Complete with your code.

        gaussian = (1/math.sqrt(2*math.pi*math.pow(sigma,2))) * math.exp(-0.5*math.pow((x-mu),2)/math.pow(sigma,2))
        return gaussian

    def _measurement_probability(self, measurements: List[float], particle: Tuple[float, float, float]) -> float:
        """Computes the probability of a set of measurements given a particle's pose.

        If a measurement is unavailable (usually because it is out of range), it is replaced with twice the sensor range
        to perform the computation. This value has experimentally been proven valid to deal with missing measurements.
        Nevertheless, it might not be the optimal replacement value.

        Args:
            measurements: Sensor measurements [m].
            particle: Particle pose (x, y, theta) in [m] and [rad].

        Returns:
            float: Probability.

        """
        # TODO: Complete with your code.

        p=1

        measurements_hat = self._sense(particle)

        for i in range(len(measurements)):
            if measurements_hat[i] > 2:
                x_hat = 1.5
            else:
                x_hat = measurements_hat[i]

            if measurements[i] > 2:
                x = 1.5
            else:
                x = measurements[i]

            p = p * self._gaussian(x, self._sense_noise, x_hat)

        return p

    def _sensor_rays(self, particle: Tuple[float, float, float]) -> List[List[Tuple[float, float]]]:
        """Determines the simulated sensor ray segments for a given particle.

        Args:
            particle: Particle pose (x, y, theta) in [m] and [rad].

        Returns: Ray segments.
                 Format: [[(x0_begin, y0_begin), (x0_end, y0_end)], [(x1_begin, y1_begin), (x1_end, y1_end)], ...]

        """
        x = particle[0]
        y = particle[1]
        theta = particle[2]

        # Convert sensors to world coordinates
        xw = [x + ds * math.cos(theta + phi) for ds, phi in zip(self._ds, self._phi)]
        yw = [y + ds * math.sin(theta + phi) for ds, phi in zip(self._ds, self._phi)]
        tw = [sensor[2] for sensor in self._sensors]

        rays = []

        for xs, ys, ts in zip(xw, yw, tw):
            x_end = xs + self._sensor_range * math.cos(theta + ts)
            y_end = ys + self._sensor_range * math.sin(theta + ts)
            rays.append([(xs, ys), (x_end, y_end)])

        return rays



def test():
    """Function used to test the ParticleFilter class independently."""
    import time
    from robot_p3dx import RobotP3DX

    # Measurements from sensors 1 to 8 [m]
    measurements = [
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.9343, float('inf'), float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, float('inf'), float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, float('inf'), float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.8582, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.7066, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.5549, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.4957, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.4957, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.4957, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.4957, float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.4957, 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8430, 0.4957, 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), 0.4957, 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), 0.4957, 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), 0.4957, 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), 0.4957, 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), float('inf'), 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), float('inf'), 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), float('inf'), 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), float('inf'), float('inf'), 0.3619),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.9920, float('inf'), float('inf')),
        (0.3618, 0.4895, 0.8337, float('inf'), float('inf'), 0.8795, float('inf'), float('inf')),
        (0.3832, 0.6021, float('inf'), float('inf'), 1.2914, 0.9590, float('inf'), float('inf')),
        (0.4207, 0.7867, float('inf'), float('inf'), 0.9038, float('inf'), float('inf'), 0.5420),
        (0.4778, float('inf'), float('inf'), float('inf'), 0.8626, float('inf'), float('inf'), 0.3648),
        (0.5609, float('inf'), float('inf'), 0.9514, 0.9707, float('inf'), float('inf'), 0.3669),
        (0.6263, float('inf'), float('inf'), 0.8171, 0.8584, float('inf'), float('inf'), 0.4199),
        (0.6918, float('inf'), 0.9942, 0.6828, 0.7461, float('inf'), float('inf'), 0.5652),
        (0.7572, 0.9544, 0.9130, 0.5485, 0.6338, float('inf'), float('inf'), 0.7106),
        (0.8226, 0.8701, 0.8319, 0.4142, 0.5215, float('inf'), float('inf'), 0.8559),
        (0.8880, 0.7858, 0.7507, 0.2894, 0.4092, float('inf'), float('inf'), float('inf')),
        (0.9534, 0.7016, 0.6696, 0.2009, 0.2969, float('inf'), float('inf'), float('inf')),
        (float('inf'), 0.6173, 0.5884, 0.1124, 0.1847, 0.4020, float('inf'), float('inf')),
        (0.9789, 0.5330, 0.1040, 0.0238, 0.0724, 0.2183, float('inf'), float('inf'))]

    # Wheel angular speed commands (left, right) [rad/s]
    motions = [(0, 0), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1),
               (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (0.5, 0), (0.5, 0), (0.5, 0), (0.5, 0),
               (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1), (1, 1)]

    dt = 1  # Sampling time [s]

    m = Map('map_pf.json', sensor_range=RobotP3DX.SENSOR_RANGE, compiled_intersect=True, use_regions=True)
    pf = ParticleFilter(m, RobotP3DX.SENSORS[:8], RobotP3DX.SENSOR_RANGE)

    for u, z in zip(motions, measurements):
        # Solve differential kinematics
        v = (u[0] + u[1]) * RobotP3DX.WHEEL_RADIUS / 2
        w = (u[1] - u[0]) * RobotP3DX.WHEEL_RADIUS / RobotP3DX.TRACK

        # Move
        start = time.time()
        pf.move(v, w, dt)
        move = time.time() - start

        start = time.time()
        pf.show('Move', save_figure=True)
        plot_move = time.time() - start

        # Sense
        start = time.time()
        pf.resample(z)
        sense = time.time() - start

        start = time.time()
        pf.show('Sense', save_figure=True)
        plot_sense = time.time() - start

        # Display timing results
        print('Particle filter: {0:6.3f} s  =  Move: {1:6.3f} s  +  Sense: {2:6.3f} s   |   Plotting: {3:6.3f} s  =  Move: {4:6.3f} s  +  Sense: {5:6.3f} s'.format(move + sense, move, sense, plot_move + plot_sense, plot_move, plot_sense))


# This "strange" function is only called if this script (particle_filter.py) is the program's entry point.
if __name__ == '__main__':
    try:
        test()
    except KeyboardInterrupt:  # Press Ctrl+C to gracefully stop the program
        pass
