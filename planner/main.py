import pymavlink
import math

#start to scan environment
scanning_trajectory = [(0, 0, 0, 0), (0, 0, 1, 0), (0, 0, 1, math.pi/2), (0, 0, 1, 0)]

#static trajectory for now (todo: generate exploration trajectory)
exploration_trajectory = [(1, 0, 1, 0), (1, 1, 1, 0), (0, 1, 1, math.pi/2), (0, 0, 1, 0)]

def get_pose():
    pass

def get_target():
    #pure pursuit
    #pure pursuit needs the current drone location, we'll get this from SLAM first (if unreliable then use internal IMU)
    pass