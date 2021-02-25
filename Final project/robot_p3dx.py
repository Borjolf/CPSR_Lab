import numpy as np
import sim

from robot import Robot
from typing import Any, Dict, List, Tuple


class RobotP3DX(Robot):
    """Class to control the Pioneer 3-DX robot."""

    # Constants
    SENSOR_RANGE = 1.0     # Ultrasonic sensor range [m]
    TRACK = 0.33           # Distance between same axle wheels [m]
    WHEEL_RADIUS = 0.0975  # Radius of the wheels [m]

    # Sensor location and orientation (x, y, theta) in the robot coordinate frame
    SENSORS = [(0.1067, 0.1382, 1.5708),
               (0.1557, 0.1250, 0.8727),
               (0.1909, 0.0831, 0.5236),
               (0.2095, 0.0273, 0.1745),
               (0.2095, -0.0273, -0.1745),
               (0.1909, -0.0785, -0.5236),
               (0.1558, -0.1203, -0.8727),
               (0.1067, -0.1382, -1.5708),
               (-0.1100, -0.1382, -1.5708),
               (-0.1593, -0.1203, -2.2689),
               (-0.1943, -0.0785, -2.6180),
               (-0.2129, -0.0273, -2.9671),
               (-0.2129, 0.0273, 2.9671),
               (-0.1943, 0.0785, 2.6180),
               (-0.1593, 0.1203, 2.2689),
               (-0.1100, 0.1382, 1.5708)]

    def __init__(self, client_id: int, dt: float):
        """Pioneer 3-DX robot class initializer.

        Args:
            client_id: CoppeliaSim connection handle.
            dt: Sampling period [s].

        """
        Robot.__init__(self, client_id, track=self.TRACK, wheel_radius=self.WHEEL_RADIUS)
        self._dt = dt
        self._motors = self._init_motors()
        self._sensors = self._init_sensors()

    def move(self, v: float, w: float):
        """Solve inverse differential kinematics and send commands to the motors.

        Args:
            v: Linear velocity of the robot center [m/s].
            w: Angular velocity of the robot center [rad/s].

        """
        wr = (v + w* self.TRACK / 2 ) / self.WHEEL_RADIUS
        wl = (v - w* self.TRACK / 2 ) / self.WHEEL_RADIUS
        
        rc = sim.simxSetJointTargetVelocity ( self._client_id, self._motors['right'] , wr , sim.simx_opmode_oneshot )
        rc = sim.simxSetJointTargetVelocity ( self._client_id, self._motors['left'] , wl , sim.simx_opmode_oneshot )

        pass

    def sense(self) -> Tuple[List[float], float, float]:
        """Read ultrasonic sensors and encoders.

        Returns:
            z_us: Distance from every ultrasonic sensor to the closest obstacle [m].
            z_v: Linear velocity of the robot center [m/s].
            z_w: Angular velocity of the robot center [rad/s].

        """
        # Read ultrasonic sensors
        z_us = [float('inf')] * len(self.SENSORS)
            
        for i in range(len(self._sensors)):
            _ , is_valid , detected_point , _ , _ = sim.simxReadProximitySensor ( self._client_id , self._sensors[i] , sim.simx_opmode_buffer )
            
            if is_valid is True:
                distance = np.linalg.norm ( detected_point )
                z_us[i] = distance
      
        # Read encoders
        z_v, z_w = self._sense_encoders()

        return z_us, z_v, z_w

    def _init_encoders(self):
        """Initialize encoder streaming."""
        sim.simxGetFloatSignal(self._client_id, 'leftEncoder', sim.simx_opmode_streaming)
        sim.simxGetFloatSignal(self._client_id, 'rightEncoder', sim.simx_opmode_streaming)

    def _init_motors(self) -> Dict[str, int]:
        """Acquire motor handles.

        Returns: {'left': handle, 'right': handle}

        """
        motors = {'left': None, 'right': None}

        rc , handle_left = sim.simxGetObjectHandle ( self._client_id , "Pioneer_p3dx_leftMotor" , sim.simx_opmode_blocking )
        rc , handle_right = sim.simxGetObjectHandle ( self._client_id , "Pioneer_p3dx_rightMotor" , sim.simx_opmode_blocking )

        motors['left'] = handle_left
        motors['right'] = handle_right
        

        return motors

    def _init_sensors(self) -> List[Any]:
        """Acquire ultrasonic sensor handles and initialize US and encoder streaming.

        Returns: List with ultrasonic sensor handles.

        """
        self._init_encoders()
        
        sensors = [None] * len(self.SENSORS)
        
        for i in range(len(sensors)):
            sensor_name = "Pioneer_p3dx_ultrasonicSensor" + str(i+1)
            rc , handle = sim.simxGetObjectHandle ( self._client_id , sensor_name , sim.simx_opmode_blocking )
            sensors[i] = handle;
            sim.simxReadProximitySensor ( self._client_id , sensors[i] , sim.simx_opmode_streaming );
        
        return sensors

    def _sense_encoders(self) -> Tuple[float, float]:
        """Solve forward differential kinematics from encoder readings.

        Returns:
            z_v: Linear velocity of the robot center [m/s].
            z_w: Angular velocity of the robot center [rad/s].

        """

        _ , th_l = sim.simxGetFloatSignal ( self._client_id , 'leftEncoder' , sim.simx_opmode_buffer )
        _ , th_r = sim.simxGetFloatSignal ( self._client_id , 'rightEncoder' ,sim.simx_opmode_buffer )

        wr = th_r / self._dt
        wl = th_l / self._dt

        z_v = ((wr + wl) * self.WHEEL_RADIUS) / 2

        z_w = ((wr - wl) * self.WHEEL_RADIUS) / self.TRACK


        return z_v, z_w
