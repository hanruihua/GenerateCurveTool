from tkinter import Scale
import numpy as np
from gctl.curve.dubins_path import generate_dubins_path
from gctl.curve.reeds_shepp import generate_reeds_shepp
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
        
        self.x_limit = x_limit
        self.y_limit = y_limit

        if select_mode =='mouse':
            self.fig, self.ax = plt.subplots()
            self.ax.set_aspect('equal')
            self.ax.set_xlim(x_limit)
            self.ax.set_ylim(y_limit)

        self.cpl = [] # click point list

    def generate_curve(self, curve_style='dubins', way_points=[], step_size = 0.1, min_radius=1, include_gear=False, **kwargs):
        
        # way_points: The list of way points
        # step_size: distance between the sample points

        # min_radius: minimum turning radius for ackermann robot
        # curve: a list of points

        # curve_style: 
            # - line: connect the points with line
            # - reeds: connect the points with reeds shepp path
            # - dubins: connect the points with dubins path
        
        # include_gear: if True, the curve should include the gear flag (-1, 1) and the points should be the 4*1 matrix.

        self.way_points = way_points
        self.pnum = len(self.way_points)

        if len(way_points) != 0: 
            self.pdim = self.way_points[0].shape
        elif self.point_style == 'position':
            self.pdim = (2, 1)
        elif self.point_style == 'pose':
            self.pdim = (3, 1)

        if self.select_mode == 'default':

            if len(way_points) == 0:
                print('Error: No waypoints !')
                return None

            curve = self.curve_from_waypoints(curve_style, min_radius, step_size, include_gear, **kwargs)

        elif self.select_mode == 'mouse':
            # generate reference path by mouse    
            print('Left-slick to select the way point, Enter to generate the curve')

            def on_press(event):
                if event.key == 'enter':
                    
                    print('press enter, generate curves')

                    if self.point_style == 'position':
                        self.way_points = self.cpl
                        self.pnum = len(self.way_points)

                    elif self.point_style == 'pose':
                        curve = self.curve_from_waypoints(curve_style, min_radius, step_size, include_gear, **kwargs)
                        self.plot_curve(curve, show_way_points=False)
                        plt.pause(0.001)
                
                if event.key == 'escape':
                    plt.close()

            self.fig.canvas.mpl_connect('button_press_event', self.onclick)
            self.fig.canvas.mpl_connect('key_press_event', on_press)
            plt.show()
        
        curve = self.curve_from_waypoints(curve_style, min_radius, step_size, include_gear, **kwargs)

        return curve

    def curve_from_waypoints(self, curve_style, min_radius, step_size, include_gear, **kwargs):
        
        curve = [ self.way_points[0] ]

        if curve_style == 'dubins':
            # generate dubins curve
            for i in range(self.pnum - 1):
                start_point = self.way_points[i]
                end_point = self.way_points[i+1]
                single_curve = generate_dubins_path(start_point, end_point, min_radius, step_size)
                curve = curve + single_curve[1:]
            
            if include_gear:
                curve = [np.vstack((point, [1])) for point in curve]

        elif curve_style == 'reeds':
            # generate reeds shepp curve
            for i in range(self.pnum - 1):
                start_point = self.way_points[i]
                end_point = self.way_points[i+1]
                single_curve = generate_reeds_shepp(start_point, end_point, min_radius, step_size, include_gear, **kwargs)
                curve = curve + single_curve[1:] 

            if include_gear and self.pdim[0] == 3:
                curve[0] = np.vstack((curve[0], [curve[1][-1, 0]]))

        elif curve_style == 'line':
            
            for i in range(self.pnum - 1):
                start_point = self.way_points[i]
                end_point = self.way_points[i+1]
                single_curve = self.generate_line(start_point, end_point, step_size)
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
                self.pnum = len(self.way_points)

                self.cpl = []

            plt.pause(0.001)


    def plot_curve(self, curve, style='-g', show_way_points=True, show_direction=False, **kwargs):

        # self.ax.set_xlim(range_x)
        # self.ax.set_ylim(range_y)

        if self.select_mode =='default':
            self.fig, self.ax = plt.subplots()
            self.ax.set_aspect('equal')
            self.ax.set_xlim(self.x_limit)
            self.ax.set_ylim(self.y_limit) 

        path_x_list = [p[0, 0] for p in curve]
        path_y_list = [p[1, 0] for p in curve]

        self.ax.plot(path_x_list, path_y_list, style, **kwargs)

        if show_way_points:

            px_list = [p[0, 0] for p in self.way_points]
            py_list = [p[1, 0] for p in self.way_points]
            
            if self.pdim[0] > 2:
                wu_list = [cos(p[2, 0]) for p in self.way_points]
                wy_list = [sin(p[2, 0]) for p in self.way_points]
                self.ax.quiver(px_list, py_list, wu_list, wy_list, color='r', scale=20)

            else:
                self.ax.scatter(px_list, py_list, c='k')

        if show_direction:
            if self.way_points[0] > 2:
                u_list = [cos(p[2, 0]) for p in curve]
                y_list = [sin(p[2, 0]) for p in curve]
                self.ax.quiver(path_x_list, path_y_list, u_list, y_list, color='k', scale=35, scale_units='height')

            else:
                print('No direction for 2D way points')

    def generate_line(self, start_point, end_point, step_size):

        single_curve = [start_point]
        cur_len = 0
        
        diff = end_point - start_point
        length = np.linalg.norm(diff)
        direction = diff / length

        while cur_len <= length:
            cur_len += step_size
            new_point = start_point + cur_len * direction
            single_curve.append(new_point)
            
        single_curve.append(end_point)

        return single_curve

if __name__ == '__main__':

    point_list = []

    cg = curve_generator(select_mode='mouse', )
    curve = cg.generate_curve('dubins', point_list, 0.1, 2, include_gear=False)

    if curve is not None:
        cg.plot_curve(curve, show_direction=False)

    plt.show()