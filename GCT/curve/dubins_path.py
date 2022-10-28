import numpy as np
from math import pi, sin, cos, atan2, sqrt, acos, inf

"""
code author: Han Ruihua
reference: paper 'Classification of the Dubins set, 2001', github https://github.com/AndrewWalker/Dubins-Curves/blob/master/src/dubins.c
"""

def generate_dubins_path(start=np.zeros((3, 1)), end=np.ones((3, 1)), min_radius=1, step_size=0.1):

    """
    Arguments:
        start: the start point of the curve: 3* 1 matrix
        end: the end point of the curve: 3 * 1 matrix
        min_radius: The minimum turning radius of the car
        step_size: the distance between each point
    """
    alpha, beta, d = preprocess(start, end, min_radius)
    admissible_path = [dubins_LSL, dubins_RSR, dubins_RSL, dubins_LSR, dubins_RLR, dubins_LRL]

    min_length = inf
        
    for ap in admissible_path:
        t, p, q, mode = ap(alpha, beta, d)

        if t == None:
            continue

        total_length = abs(t) + abs(p) + abs(q)
        if total_length < min_length:
            min_length = total_length
            sh_t = t
            sh_p = p
            sh_q = q
            sh_mode = mode
    
    path_point_list = path_generate(start, sh_t, sh_p, sh_q, sh_mode, min_radius, step_size)

    return path_point_list

def preprocess(self, start, end, min_radius):
        
    assert start.shape == (3, 1) and end.shape == (3, 1)

    dis, radian = self.relative(start[0:2], end[0:2])

    d = dis / min_radius

    start_theta = start[2, 0]
    goal_theta = end[2, 0]

    alpha = (start_theta - radian) % (2*pi)
    beta = (goal_theta - radian) % (2*pi)

    return alpha, beta, d

def path_generate(start, t, p, q, mode, min_radius, step_size):
    # generate a series points from the (t, p, q) set by the mode

    length = [t, p, q]

    path_point_list = [start]
    init_point = start

    for i in range(3):
        if length[i] != 0:
            path_list, end_point = element_sample(length[i], init_point, mode[i], min_radius, step_size)
            path_point_list = path_point_list + path_list
            init_point = end_point

    return path_point_list

def element_sample(length, start_point, steer_mode, min_radius, step_size):
    # generate a series points from the an element

    cur_x = start_point[0, 0]
    cur_y = start_point[1, 0]
    cur_theta = start_point[2, 0]

    path_list = []

    endpoint = np.zeros((3, 1))

    if steer_mode == 'L':
        steer = 1
    elif steer_mode == 'R':
        steer = -1
    if steer_mode == 'S':
        steer = 0

    curvature = steer * 1/ min_radius
    real_length = length * min_radius

    rot_theta = real_length * curvature

    # calculate end point
    if curvature == 0:
        endpoint = trans_point(start_point, cur_theta, real_length)

        # end_x = cur_x + cos(cur_theta) * real_length
        # end_y = cur_y + sin(cur_theta) * real_length
    else:
        center_theta = cur_theta + steer*pi/2
        center_x = cur_x + cos(center_theta) * min_radius
        center_y = cur_y + sin(center_theta) * min_radius

        center_point = np.array( [ [center_x], [center_y], [center_theta] ])

        end_cir_theta = cur_theta + rot_theta - steer*pi/2
        endpoint = trans_point(center_point, end_cir_theta, min_radius)

    cur_length = 0
    
    # loop to generate the points
    while cur_length <= real_length - step_size:
        
        next_theta = cur_theta + curvature * step_size
        cur_length = cur_length + step_size

        if curvature == 0:
            next_x = cur_x + cos(next_theta) * cur_length
            next_y = cur_y + sin(next_theta) * cur_length
        else:
            temp_theta = next_theta - steer*pi/2
            next_x = center_x + cos(temp_theta) * min_radius
            next_y = center_y + sin(temp_theta) * min_radius

        next_theta = wraptopi(next_theta)

        next_point = np.array([[next_x], [next_y], [next_theta]])

        path_list.append(next_point)

        cur_theta = next_theta

    path_list.append(endpoint)

    return path_list, endpoint

def trans_point(start_point, theta, length):
    trans_matrix = np.array([ [ cos(theta) * length ], [ sin(theta) * length ], [ theta ] ])
    end_point = start_point + trans_matrix
    end_point[2, 0] = wraptopi(end_point[2, 0])

    return end_point






def dubins_LSL(self, alpha, beta, d):

    mode = ['L', 'S', 'L']

    temp0 = atan2(cos(beta)- cos(alpha), d+sin(alpha)-sin(beta))
    t = (-alpha + temp0) % (2*pi)

    temp1 = 2 + d**2 - 2*cos(alpha-beta) + 2 * d * (sin(alpha) - sin(beta))

    if temp1 < 0:
        return None, None, None, mode

    p = sqrt(temp1)

    q = (beta - temp0) % (2*pi)

    return t, p, q, mode

def dubins_RSR(alpha, beta, d):

    mode = ['R', 'S', 'R']

    temp0 = atan2(cos(alpha)- cos(beta), d-sin(alpha)+sin(beta))
    t = (alpha - temp0) % (2*pi)

    temp1 = 2 + d**2 - 2*cos(alpha-beta) + 2*d*(sin(beta)-sin(alpha))
    if temp1 < 0:
        return None, None, None, mode

    p = sqrt(temp1)

    q = (-beta % (2*pi) + temp0) % (2*pi)

    return t, p, q, mode

def dubins_LSR(alpha, beta, d):

    mode=['L', 'S', 'R']

    temp0 = -2 + d**2 + 2*cos(alpha-beta) + 2*d*(sin(alpha) + sin(beta) )
    if temp0 < 0:
        return None, None, None, mode

    p = sqrt(temp0)

    temp1 = atan2( (-cos(alpha)-cos(beta)), (d+sin(alpha)+sin(beta)) )
    t = (-alpha+temp1 - atan2(-2, p)) % (2*pi)

    q = (-beta % (2*pi)+temp1-atan2(-2, p)) % (2*pi)

    return t, p, q, mode
    
def dubins_RSL(alpha, beta, d):
    
    mode = ['R', 'S', 'L']

    temp0 = d**2-2+2*cos(alpha-beta)-2*d*(sin(alpha)+sin(beta))
    if temp0 < 0:
        return None, None, None, mode

    p = sqrt(temp0)

    temp1 = atan2( (cos(alpha)+cos(beta)), (d-sin(alpha)-sin(beta) ) )
    t = (alpha - temp1 + atan2(2, p)) % (2*pi)

    q = (beta % (2*pi) - temp1 + atan2(2, p)) % (2*pi)

    return t, p, q, mode

def dubins_RLR(alpha, beta, d):

    mode = ['R', 'L', 'R']

    temp0 = (6-d**2+2*cos(alpha-beta)+2*d*(sin(alpha) - sin(beta))) / 8
    
    if abs(temp0) > 1:
        return None, None, None, mode

    p = acos(temp0)
    
    temp1 = atan2( (cos(alpha) - cos(beta)), (d-sin(alpha)+sin(beta)) )

    t = (alpha - temp1 + p/2) % (2*pi)

    q = (alpha - beta - t + p) % (2*pi)

    return t, p, q, mode

# TYPO IN PAPER 
def dubins_LRL(alpha, beta, d):
    
    mode=['L', 'R', 'L']

    temp0 = (6-d**2+2*cos(alpha-beta)+2*d*(sin(beta) - sin(alpha))) / 8 
    
    if abs(temp0) > 1:
        return None, None, None, mode

    p = acos(temp0)

    temp1 = atan2( (cos(beta)) - cos(alpha), (d+sin(alpha)-sin(beta)) )
    t = (- alpha + temp1 + p/2) % (2*pi)

    q = ( beta % (2*pi) - alpha - t  + p) % (2*pi)

    return t, p, q, mode


def relative(state1, state2):
        
    diff = state2 - state1

    dis = np.linalg.norm(diff)
    radian = atan2(diff[1, 0], diff[0, 0])
    
    return dis, radian

def wraptopi(rad):

    while rad > pi:
        rad = rad - 2 * pi
    
    while rad < -pi:
        rad = rad + 2 * pi

    return rad