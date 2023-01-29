from GCT.curve_generator import curve_generator
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':

    point1 = np.array([ [1], [5], [0]])
    point2 = np.array([ [5], [3], [0]])
    point3 = np.array([ [6], [5], [3]])
    point4 = np.array([ [2], [2], [2]])

    point_list = [point1, point2, point3, point4]

    cg = curve_generator()

    curve = cg.generate_curve('reeds', point_list, 0.1, 2, include_gear=False)
    # curve = cg.generate_curve('reeds', point_list, 0.1, 2, include_gear=True)

    if curve is not None: cg.plot_curve(curve, show_direction=False)

    plt.show()