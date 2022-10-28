
class curve_generator:
    def __init__(self, way_points=[], select_mode='default', curve_style='line', min_radius=0, point_style='pose') -> None:
        
        # way_points: The list of way points
        
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

        self.way_points = way_points
        self.select_mode = select_mode
        self.curve_style = curve_style
        self.point_style = point_style
        self.min_radius = min_radius
    
    def generate_curve(self, step_size):
        
        if self.select_mode == 'default':
            pass
        
        elif self.select_mode == 'mouse':
            pass
        
        if self.curve_style == 'dubins':
            pass

    # def 