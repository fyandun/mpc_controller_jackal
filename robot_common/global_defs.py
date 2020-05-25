import numpy as np


NX = 4  # [x, y, v, yaw]
NY = 4  # reference state variables
NYN = 4  # reference terminal state variables
NU = 2  # [accel, delta]
T = 5  # horizon length

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
#Rd = np.diag([0.01, 1.0])  # input difference cost matrix -> bicycle
Rd = np.diag([0.01, 0.01])  # input difference cost matrix -> skid_steer
Q = np.diag([1.0, 1.0, 0.5, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 1000.0  # max simulation time

# iterative paramter
MAX_ITER = 30  # Max iteration
DU_TH = 0.5  # iteration finish param

TARGET_SPEED = 0.4#10.0 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number
T_RAMP_UP = 0.05
T_RAMP_DOWN = 0.3


# Vehicle parameters
LENGTH = 4.5  # [m]
WIDTH = 2.0  # [m]
BACKTOWHEEL = 1.0  # [m]
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.7  # [m]
WB = 1.32  # [m]

MAX_STEER = np.deg2rad(45.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(45.0)  # maximum steering speed [rad/s]
MAX_SPEED = 55.0 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]
MAX_JERK = 0.5 # max jerk [m/sss]