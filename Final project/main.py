import math 
import os
import sim 
import time

from robot_p3dx import RobotP3DX
from navigation import Navigation
from typing import Tuple
from particle_filter import ParticleFilter
from map import Map
from planning import Planning


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
    start = (2, -3,-0*math.pi/2)
    goal = (4, 4)

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
    z_us, z_v, z_w = robot.sense()
    navigation = Navigation(dt, z_us)
    localized = False
    
    #particle filter
    m = Map('map_project.json', sensor_range=robot.SENSOR_RANGE, compiled_intersect=True, use_regions=True)
    pf = ParticleFilter(m, robot.SENSORS[:16], robot.SENSOR_RANGE)
    resample_count = 0
    pf.resample(z_us)


    #path planning
    action_costs = (10.0  , 12.0  ,  12.0 )
    path_followed = [] #this will be the path with the nodes left to reach (when we reach a node, we delete it)
    distance_tolerance = 0.2  #if we are closer to the next node than this distance, we will consider it has been reached

    start_time = time.time()

    try:
        while not goal_reached(robot_handle, goal, localized):
            
            if localized == False and pf.localized == True:
                #we enter here only when we pass from non-localized to localized
                localized = True  #set the flag

                #and create the path
                planning = Planning(m, action_costs)
                path = planning.a_star((pf.centroid[0],pf.centroid[1]), goal)
                smoothed_path = planning.smooth_path(path, data_weight=0.3, smooth_weight=0.1)
                path_followed = smoothed_path.copy() 
                planning.show(smoothed_path,block=False) 
                print("localized at ")   
                print(path_followed[0])
                #path_followed.pop(0)                     #delete the start node, as it has been already reached
                         
            
            if not localized:
                #if we are not localized yet, we explore the map and try to get our particle filter converged
                v, w = navigation.explore(z_us, z_v, z_w)
                robot.move(v, w)
                z_us, z_v, z_w = robot.sense()
                pf.move(z_v, z_w, dt)

                if resample_count >= 25: #we only resample every 25 steps, so more info is gathered and less time is lost
                    pf.resample(z_us)
                    resample_count = 0

                    #pf.show('Sense', save_figure=False)


            else:
                #once we are localized, we follow the path created previously, resampling this time at every simulation step
                if path_followed:
                    v, w = navigation.move_control(pf.centroid, path_followed[0], z_us)
                    robot.move(v, w)
                    z_us, z_v, z_w = robot.sense()
                    pf.move(z_v, z_w, dt)
                    pf.resample(z_us)

                    #pf.show('Sense', save_figure=False)

                    #check if the next node has been reached and delete it if so
                    distance = math.sqrt((pf.centroid[0] - path_followed[0][0]) ** 2 + (pf.centroid[1] - path_followed[0][1]) ** 2)
                    if distance <= distance_tolerance:
                        del path_followed[0]
                
                else:
                    #this will be executed in case our robot thinks it reached the goal, but it actually didn't
                    v, w = navigation.move_control(pf.centroid, goal, z_us)
                    robot.move(v, w)
                    z_us, z_v, z_w = robot.sense()
                    pf.move(z_v, z_w, dt)
                    pf.resample(z_us)



            # Execute the next simulation step
            sim.simxSynchronousTrigger(client_id)
            sim.simxGetPingTime(client_id)  # Make sure the simulation step has finished
            steps += 1
            
            resample_count +=1

        robot.move(0.0,0.0)  #stop the robot once we have reached the goal

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
