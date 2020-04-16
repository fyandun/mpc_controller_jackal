import numpy as np
import cubic_spline_planner
import math
import robot_common.global_defs as defs
import robot_common.robot_motion as state
import scipy.io as sio

def get_vineyard_course(dl=1.0):
    ax = [2.4583730063, 6.7518804365, 10.8136917663, 11.5632410271, 11.9180539295, 11.0372334458, 9.7314,
        8.4255910527, 4.1747550788, 1.33828323, 0.3561238461, 0.2816183203, 1.3911472059, 5.5725694352, 11.3138612183]
    ay = [-0.0001247327, -0.000342074, -0.0005405796, 0.4684504951, 1.4804265236, 2.7482054954, 2.9777, 3.2071415071, 
        2.9856358855, 2.9250751269, 3.4217359359, 4.6814972473, 5.579789281, 5.6680133088, 5.6021789348]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax,ay,ds=dl)

    #sio.savemat('np_vector.mat', {'cx':cx})
    #sio.savemat('cy.mat', {'cy':cy})
    #sio.savemat('cyaw.mat', {'cyaw':cyaw})
    return cx, cy, cyaw, ck

def get_straight_course(dl=1.0):
    ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course2(dl=1.0):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_straight_course3(dl=1.0):
    ax = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    ay = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    cyaw = [i - math.pi for i in cyaw]

    return cx, cy, cyaw, ck


def get_forward_course(dl=1.0):
    # ax = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    # ay = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]
    ax = [0.0, 6.0, 12.50, 5.0, 7.5, 3.0, -1.0]
    ay = [0.0, 0.0, 5.0, 6.50, 3.0, 5.0, -2.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck


def get_switch_back_course(dl=1.0):
    ax = [0.0, 30.0, 6.0, 20.0, 35.0]
    ay = [0.0, 0.0, 20.0, 35.0, 20.0]
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    ax = [35.0, 10.0, 0.0, 0.0]
    ay = [20.0, 30.0, 5.0, 0.0]
    cx2, cy2, cyaw2, ck2, s2 = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)
    cyaw2 = [i - math.pi for i in cyaw2]
    cx.extend(cx2)
    cy.extend(cy2)
    cyaw.extend(cyaw2)
    ck.extend(ck2)

    return cx, cy, cyaw, ck


def calc_speed_profile(cx, cy, cyaw, target_speed):

    speed_profile = [target_speed] * len(cx)
    direction = 1.0  # forward

    # Set stop point
    for i in range(len(cx) - 1):
        dx = cx[i + 1] - cx[i]
        dy = cy[i + 1] - cy[i]

        move_direction = math.atan2(dy, dx)

        if dx != 0.0 and dy != 0.0:
            dangle = abs(pi_2_pi(move_direction - cyaw[i]))
            if dangle >= math.pi / 4.0:
                direction = -1.0
            else:
                direction = 1.0

        if direction != 1.0:
            speed_profile[i] = - target_speed
        else:
            speed_profile[i] = target_speed

    speed_profile[-1] = 0.0

    return speed_profile 

def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle       


def calc_nearest_index(state, cx, cy, cyaw, pind):
    
    dx = [state.x - icx for icx in cx[pind:(pind + defs.N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + defs.N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind

def calc_ref_trajectory(state, cx, cy, cyaw, ck, sp, dl, dt, pind):
    xref = np.zeros((defs.NY, defs.T + 1))
    dref = np.zeros((1, defs.T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = sp[ind]
    xref[3, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(defs.T + 1):
        travel += abs(state.v) * dt
        dind = int(round(travel / dl))

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = sp[ind + dind]
            xref[3, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = sp[ncourse - 1]
            xref[3, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0

    return xref, ind, dref

def check_goal(current_pos, goal, tind, nind):

    # check goal
    dx = current_pos[0] - goal[0]
    dy = current_pos[1] - goal[1]
    d = math.sqrt(dx ** 2 + dy ** 2)

    isgoal = (d <= defs.GOAL_DIS)

    if abs(tind - nind) >= 5:
        isgoal = False

    # isstop = (abs(state.v) <= defs.STOP_SPEED)
    # if isgoal and isstop:
    #     return True
    if isgoal:
        return True

    return False    

def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw

def wrapTopm2Pi(yaw, yaw_prev):
    dyaw = yaw - yaw_prev
    if (dyaw >= math.pi / 2.0):
        yaw -= math.pi * 2.0
    elif(dyaw <= -math.pi / 2.0):
        yaw += math.pi * 2.0
    return yaw

def get_nparray_from_matrix(x):
    return np.array(x).flatten()