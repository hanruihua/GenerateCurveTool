from tkinter import Scale
import numpy as np
from GCT.curve.dubins_path import generate_dubins_path
from GCT.curve.reeds_shepp import generate_reeds_shepp
import matplotlib.pyplot as plt
from math import cos, sin, atan2

class curve_generator:
    def __init__(self, select_mode='default', point_style='pose', x_limit = [0, 10], y_limit=[0, 10]) -> None:
        """
        # select_mode: mode of point selection
            # 'default', use the input point_list
            # 'mouse', use mouse to select points or vectors
                -- left click to select points and vectors
                -- key enter to generate the curve
        
            
        # point_style
            # position: x,y 2 * 1 matrix
            # pose: x, y, theta, 3*1 matrix, include orientation
        """
        self.select_mode = select_mode
        self.point_style = point_style

        self.fig, self.ax = plt.subplots()
        self.ax.set_aspect('equal')
        self.ax.set_xlim(x_limit)
        self.ax.set_ylim(y_limit)

        self.cpl = [] # click point list

    def generate_curve(self, curve_style='dubins', way_points =[], step_size = 0.1, min_radius=1, split_reverse=False, **kwargs):
        
        # way_points: The list of way points
        # step_size: distance between the sample points

        # min_radius: minimum turning radius for ackermann robot
        # curve: a list of points

        # curve_style: 
            # - line: connect the points with line
            # - reeds: connect the points with reeds shepp path
            # - dubins: connect the points with dubins path

        self.way_points = way_points

        if self.select_mode == 'default':
            return self.curve_from_waypoints(curve_style, min_radius, step_size, **kwargs)

        elif self.select_mode == 'mouse':
            # generate reference path by mouse    
            print('Using mouth to select the way point')

            def on_press(event):
                if event.key == 'enter':
                    if self.point_style == 'position':
                        self.way_points = self.cpl

                    elif self.point_style == 'pose':
                        curve = self.curve_from_waypoints(curve_style, min_radius, step_size, **kwargs)
                        self.plot_curve(curve, show_way_points=False)
                        plt.pause(0.001)
                
                if event.key == 'escape':
                    plt.close()

            self.fig.canvas.mpl_connect('button_press_event', self.onclick)
            self.fig.canvas.mpl_connect('key_press_event', on_press)
            plt.show()
        
        curve = self.curve_from_waypoints(curve_style, min_radius, step_size, **kwargs)

        if split_reverse and curve_style=='reeds':
            pass




        return curve

    def curve_from_waypoints(self, curve_style, min_radius, step_size, **kwargs):

        curve = [ self.way_points[0] ]

        if curve_style == 'dubins':
            # generate dubins curve
            for i in range(len(self.way_points) - 1):
                start_point = self.way_points[i]
                end_point = self.way_points[i+1]
                single_curve = generate_dubins_path(start_point, end_point, min_radius, step_size)
                curve = curve + single_curve[1:]

        elif curve_style == 'reeds':
            # generate reeds shepp curve
            for i in range(len(self.way_points) - 1):
                start_point = self.way_points[i]
                end_point = self.way_points[i+1]
                single_curve = generate_reeds_shepp(start_point, end_point, min_radius, step_size, **kwargs)
                curve = curve + single_curve[1:] 

        else:
            print('wrong curve type')

        return curve

    def onclick(self, event):
        print('%s click: button=%d, x=%d, y=%d, xdata=%f, ydata=%f' %
            ('double' if event.dblclick else 'single', event.button,
            event.x, event.y, event.xdata, event.ydata))

        if self.point_style == 'position':
            self.cpl.append(np.array([ [event.xdata], [event.ydata] ]))
            self.ax.scatter(event.xdata, event.ydata, c='k')
            plt.pause(0.001)
        
        elif self.point_style == 'pose':
            self.cpl.append(np.array([ [event.xdata], [event.ydata] ]))
            
            if len(self.cpl) == 1:
                self.ax.scatter(event.xdata, event.ydata, c='k')

            if len(self.cpl) == 2:
                diff = self.cpl[1] - self.cpl[0]
                theta = atan2(diff[1, 0], diff[0, 0])
                
                self.ax.quiver(self.cpl[0][0, 0], self.cpl[0][1, 0], diff[0, 0], diff[1, 0], color='r', scale=20)

                way_point = np.vstack((self.cpl[0], [theta]))
                self.way_points.append(way_point)

                self.cpl = []

            plt.pause(0.001)


    def plot_curve(self, curve, style='-g', show_way_points=True, show_direction=False, **kwargs):

        # self.ax.set_xlim(range_x)
        # self.ax.set_ylim(range_y)

        path_x_list = [p[0, 0] for p in curve]
        path_y_list = [p[1, 0] for p in curve]

        line = self.ax.plot(path_x_list, path_y_list, style, **kwargs)

        if show_way_points:
            px_list = [p[0, 0] for p in self.way_points]
            py_list = [p[1, 0] for p in self.way_points]

            wu_list = [cos(p[2, 0]) for p in self.way_points]
            wy_list = [sin(p[2, 0]) for p in self.way_points]
           
            self.ax.quiver(px_list, py_list, wu_list, wy_list, color='r', scale=20)

        if show_direction:

            u_list = [cos(p[2, 0]) for p in curve]
            y_list = [sin(p[2, 0]) for p in curve]
           
            self.ax.quiver(path_x_list, path_y_list, u_list, y_list, color='k', scale=35, scale_units='height')
    

if __name__ == '__main__':

    # point1 = np.array([ [1], [5], [0]])
    # point2 = np.array([ [5], [3], [0]])
    # point3 = np.array([ [6], [5], [3]])
    # point4 = np.array([ [2], [2], [2]])

    # point_list = [point1, point2, point3, point4]
    point_list = []

    # cg = curve_generator(select_mode='mouse', curve_style='reeds', point_style='pose')
    cg = curve_generator(select_mode='mouse', )
    curve = cg.generate_curve('dubins', point_list, 0.1, 2, include_gear=False)

    # print(curve)

    cg.plot_curve(curve, show_direction=False)

    # for i in range(len(curve) - 1):

    #     distance = round(np.linalg.norm( curve[i+1][0:2] - curve[i][0:2] ), 2)

    #     print(distance)

    plt.show()