
class curve_generator:
    def __init__(self, way_points, select_mode='default', curve_style='line', point_style='state',) -> None:
        # way_points: The list of way points
        
        # select_mode: mode of point selection
            # 'default', use the input point_list
            # 'mouse', use mouse to select points (vectors)
        
        # curve_style: 
            # line: connect the points with line
            # reeds: connect the points with reeds shepp path
            # dubins: connect the points with dubins path
            # cubic: connect the points with cubic spline
        # point style: 
            # waypoint: x,y 2 * 1 matrix
            # state: x, y, theta, 3*1 matrix

        # point: x, y, theta
        self.way_points = way_points
        



    # def 