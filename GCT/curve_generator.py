from tkinter import Scale
import numpy as np
from GCT.curve.dubins_path import generate_dubins_path
from GCT.curve.reeds_shepp import generate_reeds_shepp
import matplotlib.pyplot as plt
from math import cos, sin

class curve_generator:
    def __init__(self, select_mode='default', curve_style='line', point_style='pose') -> None:
        
        # select_mode: mode of point selection
            # 'default', use the input point_list
            # 'mouse', use mouse to select points or vectors
        
        # curve_style: 
            # line: connect the points with line
            # reeds: connect the points with reeds shepp path
            # dubins: connect the points with dubins path
            
        # point_style
            # position: x,y 2 * 1 matrix
            # pose: x, y, theta, 3*1 matrix, include orientation

        self.select_mode = select_mode
        self.curve_style = curve_style
        self.point_style = point_style
        
    def generate_curve(self, way_points =[], step_size = 0.1, min_radius=1, **kwargs):
        
        # way_points: The list of way points
        # step_size: distance between the sample points

        # min_radius: minimum turning radius for ackermann robot
        # curve: a list of points

        # curve_style: 
        #   - dubins: dubins curve (point_style must be pose)

        
        self.way_points = way_points

        curve = [ way_points[0] ]

        if self.select_mode == 'mouse':
            pass
        
        if self.curve_style == 'dubins':
            # generate dubins curve

            for i in range(len(way_points) - 1):
                start_point = way_points[i]
                end_point = way_points[i+1]
                single_curve = generate_dubins_path(start_point, end_point, min_radius, step_size)
                curve = curve + single_curve[1:]

        elif self.curve_style == 'reeds':
            # generate reeds shepp curve
            for i in range(len(way_points) - 1):
                start_point = way_points[i]
                end_point = way_points[i+1]
                single_curve = generate_reeds_shepp(start_point, end_point, min_radius, step_size, **kwargs)
                curve = curve + single_curve[1:]
            
        return curve

    def plot_curve(self, curve, style='-g', show_way_points=True, show_direction=False, range_x = [0, 10], range_y = [0, 10], **kwargs):

        fig, ax = plt.subplots()
    
        ax.set_aspect('equal')

        plt.xlim(range_x)
        plt.ylim(range_y)

        path_x_list = [p[0, 0] for p in curve]
        path_y_list = [p[1, 0] for p in curve]

        line = ax.plot(path_x_list, path_y_list, style, **kwargs)

        if show_way_points:
            px_list = [p[0, 0] for p in self.way_points]
            py_list = [p[1, 0] for p in self.way_points]

            wu_list = [cos(p[2, 0]) for p in self.way_points]
            wy_list = [sin(p[2, 0]) for p in self.way_points]
           
            ax.quiver(px_list, py_list, wu_list, wy_list, color='r', scale=20)

        if show_direction:

            u_list = [cos(p[2, 0]) for p in curve]
            y_list = [sin(p[2, 0]) for p in curve]
           
            ax.quiver(path_x_list, path_y_list, u_list, y_list, color='k', scale=35, scale_units='height')
    

if __name__ == '__main__':

    point1 = np.array([ [1], [5], [0]])
    point2 = np.array([ [5], [3], [0]])
    point3 = np.array([ [6], [5], [3]])
    point4 = np.array([ [2], [2], [2]])

    point_list = [point1, point2, point3, point4]

    cg = curve_generator(curve_style='reeds')
    curve = cg.generate_curve(point_list, 0.1, 5, include_gear=True)
    cg.plot_curve(curve, show_direction=False)

    # for i in range(len(curve) - 1):

    #     distance = round(np.linalg.norm( curve[i+1][0:2] - curve[i][0:2] ), 2)

    #     print(distance)

    plt.show()