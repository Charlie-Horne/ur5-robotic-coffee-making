# A slightly more advanced example with joint array and HT matrix movements
# NOTE: an HT matrix is not unique, there are many solutions for a given pose which could result in
# unexpected robot poses. Therefore, it is recommended that joint angle arrays are used as 
# intermediate points (as shown), since these are unique and will ensure the robot goes to an 
# expected position before going to a pose defined by a calculated HT matrix.

# K Paul, updated 5 Sept 2025
# version 2


import numpy as np
from time import sleep
from robodk.robolink import *
import robodk.robomath as rm

import tools

RDK = Robolink()
RDK.setRunMode(RUNMODE_SIMULATE)
UR5 = RDK.Item("UR5", ITEM_TYPE_ROBOT)

tls = tools.Tools(RDK)

# define a joint angle array for an intermediate point: theta_1, theta_2, ..., theta_6 (from base to tool)
J_intermediatepoint = [-139.034020, -77.862852, -123.118833, -68.988563, 89.998268, 3.296280]

# define an HT for a pose in the world frame, near the cup dispenser. This could be something calculated or copied from the RoboDK GUI
T_nearcupdispenser_np = np.array([[ -0.288352,     0.481536,    -0.827633,  -422.157000 ],
                                  [  0.922363,     0.371767,    -0.105054,  -151.004000 ],
                                  [  0.257100,    -0.793670,    -0.551351,   232.946000 ],
                                  [  0.000000,     0.000000,     0.000000,     1.000000 ]])

# convert numpy array into an RDK matrix
T_nearcupdispenser = rm.Mat(T_nearcupdispenser_np.tolist())

# reset the sim
robot_program = RDK.Item("Reset_Simulation_R", ITEM_TYPE_PROGRAM)
robot_program.RunCode()
robot_program.WaitFinished()

# pick up mazzer tool
tls.mazzer_tool_attach_r_ati()

# joint movement using a joint array
UR5.MoveJ(J_intermediatepoint, blocking=True)

time.sleep(1)

# linear movement using an HT matrix
UR5.MoveL(T_nearcupdispenser, blocking=True)

time.sleep(1)

UR5.MoveJ(J_intermediatepoint, blocking=True)

tls.mazzer_tool_detach_r_ati()

# go back home
UR5.MoveJ(RDK.Item("Home_R", ITEM_TYPE_TARGET), True)

