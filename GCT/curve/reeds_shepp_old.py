"""
Author: Han Ruihua
Reference: paper 'Optimal paths for a car that goes both forwards and backwards', github: https://github.com/nathanlct/reeds-shepp-curves/tree/master
"""

import numpy as np
from collections import namedtuple
from math import sqrt, atan2, sin, cos, pi, inf, asin, acos

class reeds_shepp:

    def __init__(self, min_radius=1):

        self.min_r = min_radius
        self.element = namedtuple('element','len steer gear') # steer 1,0,-1, gear 1,-1
        self.path_formula1 = [self.LpSpLp, self.LpSpRp, self.LpRnLp, self.LpRpLnnRn, self.LpRnLnRp, self.LpRnRnLnRp]
        self.path_formula2 = [self.LpRnLn, self.LpRnSnLn, self.LpRnSnRn]

    # preprocess
    def preprocess(self, start_point=np.zeros((3, 1)), goal_point=np.zeros((3, 1))):
        
        ro_phi = start_point[2, 0]
        ro_matrix = np.array([[cos(ro_phi), sin(ro_phi)], 
                             [-sin(ro_phi), cos(ro_phi)]])

        diff = goal_point[0:2] - start_point[0:2]
        new_pos = ro_matrix @ diff

        new_x = new_pos[0, 0] / self.min_r
        new_y = new_pos[1, 0] / self.min_r
        new_phi = goal_point[2, 0] - start_point[2, 0]

        return new_x, new_y, new_phi

    # shortest path
    def shortest_path(self, start_point=np.zeros((3, 1)), goal_point=np.ones((3, 1)), step_size=0.01, include_gear=False):
        
        if isinstance(start_point, tuple):
            start_point = np.array([start_point]).T

        if isinstance(goal_point, tuple):
            goal_point = np.array([goal_point]).T

        x, y, phi = self.preprocess(start_point, goal_point)

        path_list1, List1 = self.symmetry_curve1(x, y, phi)
        path_list2, List2 = self.symmetry_curve2(x, y, phi)

        total_path_list = path_list1 + path_list2
        total_L_list = List1 + List2

        L_min = min(total_L_list) 
        path_min = total_path_list[total_L_list.index(L_min)]

        path_point_list = self.reeds_path_generate(start_point, path_min, step_size, include_gear)

        path_point_list.append(goal_point)

        return path_point_list

    def shortest_length(self, start_point=np.zeros((3, 1)), goal_point=np.ones((3, 1))):
        
        x, y, phi = self.preprocess(start_point, goal_point)

        path_list1, List1 = self.symmetry_curve1(x, y, phi)
        path_list2, List2 = self.symmetry_curve2(x, y, phi)

        total_L_list = List1 + List2

        L_min = round(min(total_L_list), 3)
        
        return L_min

    # calculate curves
    def symmetry_curve1(self, x, y, phi):
        
        path_list = []
        L_list = []

        for timeflip in [False, True]:
            for reflect in [False, True]:
                for cal_formula in self.path_formula1:

                    rg_flag = -1 if timeflip else 1
                    rs_flag = -1 if reflect else 1
                    
                    path, L = cal_formula(rg_flag*x, rs_flag*y, rs_flag*rg_flag*phi, timeflip=timeflip, reflect=reflect)

                    path_list.append(path.copy())
                    L_list.append(L)
        
        return path_list, L_list

    def symmetry_curve2(self, x, y, phi):
        
        path_list = []
        L_list = []

        for timeflip in [False, True]:
            for reflect in [False, True]:
                for backward in [False, True]:
                    for cal_formula in self.path_formula2:

                        rg_flag = -1 if timeflip else 1
                        rs_flag = -1 if reflect else 1
                        sx, sy, sphi = self.backward(x,y,phi) if backward else (x,y,phi)
                        
                        path, L = cal_formula(rg_flag*sx, rs_flag*sy, rs_flag*rg_flag*sphi, timeflip=timeflip, reflect=reflect, backward=backward)

                        path_list.append(path.copy())
                        L_list.append(L)
        
        return path_list, L_list

    # path generate
    def reeds_path_generate(self, start_point, path, step_size, include_gear=False):

        path_point_list = [start_point]
        end_point = None

        if len(path) == 0:
            print('no path')
            return path_point_list

        for i in range(len(path)):
            
            path_list, end_point = self.element_sample(element=path[i], start_point=start_point, step_size=step_size, include_gear=include_gear)

            path_point_list = path_point_list + path_list
            start_point = end_point

        return path_point_list

    def element_sample(self, element, start_point, step_size, include_gear=False):

        length = element.len * self.min_r
        
        if include_gear:
            if start_point.shape[0] < 4:
                add_row = np.array([[element.gear]])
                start_point = np.vstack((start_point, add_row))
            else:
                start_point[3, 0] = element.gear

        path_list = []
        cur_length = 0

        while cur_length < length:   
            
            pre_length = cur_length + step_size

            if cur_length <= length and pre_length > length:
                step_size = length - cur_length

            new_point = self.motion_acker_step(start_point, element.gear, element.steer, step_size, include_gear)

            cur_length = cur_length + step_size
            path_list.append(new_point)
            start_point = new_point

        return path_list, new_point
    

    def motion_acker_step(self, start_point, gear, steer, step_size, include_gear=False):
    
        cur_x = start_point[0, 0]
        cur_y = start_point[1, 0]
        cur_theta = start_point[2, 0]

        curvature = steer * 1/self.min_r
        
        rot_theta = abs(steer) * step_size * curvature * gear
        trans_len = (1 - abs(steer)) * step_size * gear

        rot_matrix = np.array([[cos(rot_theta), -sin(rot_theta)], [sin(rot_theta), cos(rot_theta)]])
        trans_matrix = trans_len * np.array([[cos(cur_theta)], [sin(cur_theta)]]) 

        center_x = cur_x + cos(cur_theta + steer * pi/2) * self.min_r
        center_y = cur_y + sin(cur_theta + steer * pi/2) * self.min_r
        center = np.array([[center_x], [center_y]])

        if include_gear:
            new_state = np.zeros((4, 1))
            new_state[0:2] = rot_matrix @ (start_point[0:2] - center) + center + trans_matrix
            new_state[2, 0] = self.wraptopi(cur_theta + rot_theta)
            new_state[3, 0] = gear
        else:
            new_state = np.zeros((3, 1))
            new_state[0:2] = rot_matrix @ (start_point[0:2] - center) + center + trans_matrix
            new_state[2, 0] = self.wraptopi(cur_theta + rot_theta)
        
        return new_state


    # transform
    # mode 2pi
    def M(self, theta):
        theta = theta % (2*pi)
        if theta < - pi: return theta + 2*pi
        if theta >= pi: return theta - 2*pi
        return theta

    def wraptopi(self, radian):
         # -pi to pi
        if radian > pi:
            radian2 = radian - 2 * pi
        elif radian < -pi:
            radian2 = radian + 2 * pi
        else:
            radian2 = radian

        # diff = radian1 - radian2
        # print(diff)
        return radian2 

    # polar 
    def R(self, x, y):   
       r = sqrt(x**2 + y**2)
       theta = atan2(y, x)
       return r, theta

    def backward(self, x, y, phi):
        new_x = x*cos(phi) + y*sin(phi)
        new_y = x*sin(phi) - y*cos(phi)
        return new_x, new_y, phi

    # curve formula
    # formula 8.1
    def LpSpLp(self, x, y, phi, timeflip=False, reflect=False):
        
        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        #calculate
        u, t = self.R(x - sin(phi), y-1+cos(phi))
        v = (phi - t) % (2*pi)

        # if t < 0 or t > pi or v < 0 or v> pi:
        #     return path, inf

        if t<0 or u<0 or v<0:
            return path, inf

        path.append(self.element(t, steer_flag, gear_flag)) 
        path.append(self.element(u, 0, gear_flag)) 
        path.append(self.element(v, steer_flag, gear_flag))

        L = abs(t) + abs(u) + abs(v)

        return path, L

    # formula 8.2
    def LpSpRp(self, x, y, phi, timeflip=False, reflect=False):

        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        u1, t1 = self.R(x+sin(phi), y-1-cos(phi))

        if u1**2 < 4:
            L = inf
        else:
            u = sqrt(u1**2 - 4)

            T, theta = self.R(u, 2)
            t = self.M(t1+theta)
            v = self.M(t-phi)
            L = abs(t) + abs(u) + abs(v)

            if t<0 or u<0 or v<0:
                return path, inf

            path.append(self.element(t, steer_flag*1, gear_flag)) 
            path.append(self.element(u, 0, gear_flag)) 
            path.append(self.element(v, steer_flag*-1, gear_flag))   

        return path, L

    # formula 8.3  typo in paper
    def LpRnLp(self, x, y, phi, timeflip=False, reflect=False):

        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x - sin(phi)
        eta = y - 1 + cos(phi)
        u1, theta = self.R(xi, eta)

        # if u1 ** 2 > 4:
        #     return path, inf
        if u1 > 4:
            return path, inf
        
        A = acos(u1/4)
        t = self.M(theta + pi/2 + A)
        u = self.M(pi - 2*A)
        v = self.M(phi-t-u)
        
        L = abs(t) + abs(u) + abs(v)

        if t<0 or u<0 or v<0:
            return path, inf

        path.append(self.element(t, steer_flag*1, gear_flag*1)) 
        path.append(self.element(u, steer_flag*-1, gear_flag*-1)) 
        path.append(self.element(v, steer_flag*1, gear_flag*1)) 

        return path, L

    # formula 8.4, typo in paper, 
    def LpRnLn(self, x, y, phi, timeflip=False, reflect=False, backward=False):

        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x - sin(phi)
        eta = y - 1 + cos(phi)
        u1, theta = self.R(xi, eta)

        if u1  > 4:
            return path, inf
        
        A = acos(u1/4)
        t = self.M(theta + pi/2 + A)
        u = self.M(pi - 2*A)
        v = self.M(t+u-phi)
        
        if t<0 or u<0 or v<0:
            return path, inf
        
        L = abs(t) + abs(u) + abs(v)
    
        if backward:
            path.append(self.element(v, steer_flag*1, gear_flag*-1))
            path.append(self.element(u, steer_flag*-1, gear_flag*-1))
            path.append(self.element(t, steer_flag*1, gear_flag*1))
        else:
            path.append(self.element(t, steer_flag*1, gear_flag*1)) 
            path.append(self.element(u, steer_flag*-1, gear_flag*-1)) 
            path.append(self.element(v, steer_flag*1, gear_flag*-1)) 

        return path, L
    
    # formula 8.7 typo in paper
    def LpRpLnnRn(self, x, y, phi, timeflip=False, reflect=False):

        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x + sin(phi)
        eta = y - 1 - cos(phi)
        u1, theta = self.R(xi, eta)

        if u1 > 4:
            return path, inf 
        
        if u1 <= 2:
            A = acos((u1+2)/4)
            t = self.M(theta+pi/2+A)
            u = self.M(A)
            v = self.M(phi-t+2*u)
        else:
            A = acos((u1-2)/4)
            t = self.M(theta+pi/2-A)
            u = self.M(pi-A)
            v = self.M(phi-t+2*u)
        
        if t<0 or u<0 or v<0:
            return path, inf

        L = abs(t) + 2*abs(u) + abs(v)

        path.append(self.element(t, steer_flag*1, gear_flag*1)) 
        path.append(self.element(u, steer_flag*-1, gear_flag*1)) 
        path.append(self.element(u, steer_flag*1, gear_flag*-1))
        path.append(self.element(v, steer_flag*-1, gear_flag*-1))  

        return path, L

    # formula 8.8 
    def LpRnLnRp(self, x, y, phi, timeflip=False, reflect=False):
        
        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x + sin(phi)
        eta = y - 1 - cos(phi)
        u1, theta = self.R(xi, eta)
        rho = (20 - u1**2) / 16

        if rho >= 0 and rho <= 1 and u1<=6:
            u = acos(rho)
            A = asin(2*sin(u)/u1)
            t = self.M(theta+pi/2+A)
            v = self.M(t-phi)

            if t<0 or u<0 or v<0:
                return path, inf
            
            L = abs(t) + 2*abs(u) + abs(v)

            path.append(self.element(t, steer_flag*1, gear_flag*1)) 
            path.append(self.element(u, steer_flag*-1, gear_flag*-1)) 
            path.append(self.element(u, steer_flag*1, gear_flag*-1))
            path.append(self.element(v, steer_flag*-1, gear_flag*1))  

        else:
            return path, inf

        return path, L
    
    # formula 8.9
    def LpRnSnLn(self, x, y, phi, timeflip=False, reflect=False, backward=False):
        
        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x - sin(phi)
        eta = y - 1 + cos(phi)
        rho, theta = self.R(xi, eta)
        
        if rho < 2:
            return path, inf
        
        u = sqrt(rho**2-4) -2
        A = atan2(2, u+2)
        t = self.M(theta+pi/2+A)
        v = self.M(t-phi+pi/2)

        if t<0 or u<0 or v<0:
            return path, inf
        
        L = abs(t) + pi/2 + abs(u) + abs(v)

        if backward:
            path.append(self.element(v, steer_flag*1, gear_flag*-1)) 
            path.append(self.element(u, steer_flag*0, gear_flag*-1))
            path.append(self.element(pi/2, steer_flag*-1, gear_flag*-1)) 
            path.append(self.element(t, steer_flag*1, gear_flag*1))               
        else:
            path.append(self.element(t, steer_flag*1, gear_flag*1)) 
            path.append(self.element(pi/2, steer_flag*-1, gear_flag*-1)) 
            path.append(self.element(u, steer_flag*0, gear_flag*-1))
            path.append(self.element(v, steer_flag*1, gear_flag*-1)) 

        return path, L

    # formula 8.10
    def LpRnSnRn(self, x, y, phi, timeflip=False, reflect=False, backward=False):

        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x + sin(phi)
        eta = y - 1 - cos(phi)
        rho, theta = self.R(xi, eta)

        if rho < 2:
            return path, inf

        t = self.M(theta+pi/2)
        u = rho-2
        v = self.M(phi - t -pi/2)

        if t<0 or u<0 or v<0:
            return path, inf

        L = abs(t) + pi/2 + abs(u) + abs(v)
        
        if backward:
            path.append(self.element(v, steer_flag*-1, gear_flag*-1))
            path.append(self.element(u, steer_flag*0, gear_flag*-1)) 
            path.append(self.element(pi/2, steer_flag*-1, gear_flag*-1)) 
            path.append(self.element(t, steer_flag*1, gear_flag*1))
        else:
            path.append(self.element(t, steer_flag*1, gear_flag*1)) 
            path.append(self.element(pi/2, steer_flag*-1, gear_flag*-1)) 
            path.append(self.element(u, steer_flag*0, gear_flag*-1))
            path.append(self.element(v, steer_flag*-1, gear_flag*-1))

        return path, L

    # formula 8.11 typo in paper
    def LpRnRnLnRp(self, x, y, phi, timeflip=False, reflect=False):

        path = []
        gear_flag = -1 if timeflip else 1
        steer_flag = -1 if reflect else 1

        xi = x + sin(phi)
        eta = y - 1 - cos(phi)
        rho, theta = self.R(xi, eta)

        if rho < 4:
            return path, inf

        u = sqrt(rho**2 - 4) - 4
        A = atan2(2, u+4)
        t = self.M(theta+pi/2+A)
        v = self.M(t-phi)

        if t<0 or u<0 or v<0:
            return path, inf

        L = abs(t) + pi/2 + abs(u) + pi/2 + abs(v)

        path.append(self.element(t, steer_flag*1, gear_flag*1)) 
        path.append(self.element(pi/2, steer_flag*-1, gear_flag*-1)) 
        path.append(self.element(u, steer_flag*0, gear_flag*-1))
        path.append(self.element(pi/2, steer_flag*1, gear_flag*-1))
        path.append(self.element(v, steer_flag*-1, gear_flag*1))

        return path, L

