import math
import os
import sim
import time

from robot_p3dx import RobotP3DX
from navigation import Navigation
from typing import Tuple


def create_robot(client_id: int, x: float, y: float, theta: float):
    current_path = os.path.dirname(os.path.abspath(__file__))
    model_path = os.path.join(current_path, 'p3dx.ttm')

    rc, out_ints, _, _, _ = sim.simxCallScriptFunction(client_id, 'Maze', sim.sim_scripttype_childscript, 'createRobot', [], [x, y, theta], [model_path], "", sim.simx_opmode_blocking)
    robot_handle = out_ints[0]

    return rc, robot_handle


def goal_reached(robot_handle: int, goal: Tuple[float, float], localized: bool, tolerance: float = 0.1) -> bool:
    distance = float('inf')

    if localized:
        _, position = sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_buffer)
        distance = math.sqrt((position[0] - goal[0]) ** 2 + (position[1] - goal[1]) ** 2)

    return distance < tolerance


if __name__ == '__main__':
    # Connect to CoppeliaSim
    sim.simxFinish(-1)  # Close all pending connections
    client_id = sim.simxStart('127.0.0.1', 19997, True, True, 2000, 5)

    if client_id == -1:
        raise ConnectionError('Could not connect to CoppeliaSim. Make sure the application is open.')

    # Start simulation
    sim.simxSynchronous(client_id, True)
    sim.simxStartSimulation(client_id, sim.simx_opmode_blocking)

    # Initial and final locations
    start = (2, -3, -math.pi/2)
    goal = (2, 2)

    # Create the robot
    _, robot_handle = create_robot(client_id, start[0], start[1], start[2])
    sim.simxGetObjectPosition(client_id, robot_handle, -1, sim.simx_opmode_streaming)  # Initialize real position streaming

    # Execute a simulation step to get initial sensor readings
    sim.simxSynchronousTrigger(client_id)
    sim.simxGetPingTime(client_id)  # Make sure the simulation step has finished

    # Write initialization code here
    dt = 0.05
    steps = 0
    robot = RobotP3DX(client_id, dt)
    navigation = Navigation(dt)
    localized = False
    start_time = time.time()

    try:
        while not goal_reached(robot_handle, goal, localized):
            # Write your control algorithm here
            z_us, z_v, z_w = robot.sense()
            v, w = navigation.explore(z_us, z_v, z_w)
            robot.move(v, w)

            # Execute the next simulation step
            sim.simxSynchronousTrigger(client_id)
            sim.simxGetPingTime(client_id)  # Make sure the simulation step has finished
            steps += 1

    except KeyboardInterrupt:  # Press Ctrl+C to break the infinite loop and gracefully stop the simulation
        pass

    # Display time statistics
    execution_time = time.time() - start_time
    print('\n')
    print('Simulated steps: {0:d}'.format(steps))
    print('Simulated time:  {0:.3f} s'.format(steps * dt))
    print('Execution time:  {0:.3f} s ({1:.3f} s/step)'.format(execution_time, execution_time / steps))
    print('')

    # Stop the simulation and close the connection
    sim.simxStopSimulation(client_id, sim.simx_opmode_blocking)
    sim.simxGetPingTime(client_id)  # Make sure the stop simulation command had time to arrive
    sim.simxFinish(client_id)